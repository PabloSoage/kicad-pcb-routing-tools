"""pairnego_mp.py in out PAIR[,PAIR...] [pairnego.py's options] [--jobs 12]
pairnego.py's negotiated-congestion router with each round's re-routes run in parallel: the pairs
still in conflict are split in two halves, each half routed at once by a pool of processes against
the occupancy left by everything else (shared memory), then the occupancy is updated. Pairs with
no conflict stay where they are. Same footprints, same overlap test, same clean-up and writing as
pairnego.py (whose functions this runs). About ten times faster per round on 32 cores."""
import os, sys, time, random, shutil, tempfile
import multiprocessing as mp
from multiprocessing import shared_memory
HERE = os.path.dirname(os.path.abspath(__file__))
SRC = open(os.path.join(HERE, "pairnego.py"), encoding="utf-8").read().split("\npairs = [")[0]
SRC = SRC.replace("A = ap.parse_args()", 'ap.add_argument("--jobs", type=int, default=12)\nap.add_argument("--seed", type=int, default=1)\nap.add_argument("--patience", type=int, default=6)\nA = ap.parse_args()')
SRC = SRC.replace('B = g["board"]()', "B = None")
L = {"__name__": "pairnego", "__file__": os.path.join(HERE, "pairnego.py")}
ARGV = sys.argv[:]
exec(compile(SRC, "pairnego.py", "exec"), L)
sys.argv = ARGV         # pairnego.py rewrites it for pairroute.py; the spawned workers need ours
A, LAYERS, NX, NY, G, X0, Y0 = L["A"], L["LAYERS"], L["NX"], L["NY"], L["G"], L["X0"], L["Y0"]
np, LineString, PL = L["np"], L["LineString"], L["PL"]
KEYS = [("occ", l) for l in LAYERS] + [("hist", l) for l in LAYERS] + [("occh", l) for l in LAYERS] + [("occf", "F.Cu")]


def arrays(names):
    shms, out = [], {}
    for k, nm in zip(KEYS, names):
        s = shared_memory.SharedMemory(name=nm); shms.append(s)
        out.setdefault(k[0], {})[k[1]] = np.ndarray((NX, NY), dtype=np.float32, buffer=s.buf)
    return shms, out




def centre_cells(r):
    cells = []
    for zin, pts in PL.zones(r["centre"]): cells += L["line_cells"](pts)
    return cells + [(int(round((v[0] - X0) / G)), int(round((v[1] - Y0) / G))) for v in r["vias"]]


W = {}


def winit(names, tmpdir):
    W["shm"], W["a"] = arrays(names)
    base = os.path.splitext(A.inp)[0]
    tmp = os.path.join(tmpdir, f"w{os.getpid()}")
    for ext in (".kicad_pro", ".kicad_dru"):
        if os.path.exists(base + ext): shutil.copy(base + ext, tmp + ext)
    L["g"]["a"].out = tmp + ".kicad_pcb"
    L["B"] = L["g"]["board"]()
    W["S"] = {}


def minus(arr, fp):
    out = {l: arr[l].copy() for l in arr}
    if fp:
        for l, (i0, j0, m) in fp.items():
            if l in out and m.size: out[l][i0:i0 + m.shape[0], j0:j0 + m.shape[1]] -= m
    return out


def wroute(task):
    p, pfac, fp, fph, fpf = task
    if p not in W["S"]: W["S"][p] = L["setup"](p)
    if W["S"][p] is None: return p, None
    a = W["a"]
    occ = minus(a["occ"], fp)
    L["occh"] = minus(a["occh"], fph); L["occf"] = minus(a["occf"], fpf)
    r = L["route_one"](p, W["S"][p], occ, a["hist"], pfac)
    if r is None: return p, None
    return p, (r, L["footprint"](r), L["footprint"](r, hard=True), L["fan_fp"](r))


def main():
    random.seed(A.seed)
    T0 = time.time()
    shms = [shared_memory.SharedMemory(create=True, size=NX * NY * 4) for _ in KEYS]
    keep, a = arrays([s.name for s in shms])      # keep the handles: a closed handle unmaps the arrays
    for d in a.values():
        for x in d.values(): x[:] = 0
    tmpdir = tempfile.mkdtemp(prefix="nego_", dir=os.path.dirname(os.path.abspath(A.out)))
    pool = mp.get_context("spawn").Pool(A.jobs, initializer=winit, initargs=([s.name for s in shms], tmpdir))
    L["B"] = L["g"]["board"]()
    pairs = ["/" + p.strip("/") for p in A.pairs.split(",")]
    S = {}
    for p in pairs:
        s = L["setup"](p)
        if s is None: print(p, "no via pair near its far pads"); continue
        S[p] = s
    print(f"set up {len(S)} pairs ({time.time() - T0:.0f} s)", flush=True)
    add = L["add_fp"]
    R, FP, FPH, FPF = {}, {}, {}, {}

    def put(p, res):
        if p in FP:
            add(a["occ"], FP.pop(p), -1); add(a["occh"], FPH.pop(p), -1); add(a["occf"], FPF.pop(p), -1); R.pop(p)
        if res is None: return
        R[p], FP[p], FPH[p], FPF[p] = res
        add(a["occ"], FP[p], +1); add(a["occh"], FPH[p], +1); add(a["occf"], FPF[p], +1)

    def conflicts(p, r, mark):
        n = 0
        for (i, j) in centre_cells(r):
            l = r["layer"]; i0, j0, m = FPH[p][l]
            mine = m[i - i0, j - j0] if 0 <= i - i0 < m.shape[0] and 0 <= j - j0 < m.shape[1] else 0
            if a["occh"][l][i, j] - mine > 0:
                n += 1
                if mark: a["hist"][l][i, j] += 1.0
        if L["fan_lines"](r):
            i0, j0, m = FPF[p]["F.Cu"]
            for (i, j) in [c for q in L["fan_lines"](r) for c in L["line_cells"](q)]:
                mine = m[i - i0, j - j0] if 0 <= i - i0 < m.shape[0] and 0 <= j - j0 < m.shape[1] else 0
                if a["occf"]["F.Cu"][i, j] - mine > 0:
                    n += 1
                    if mark and "F.Cu" in a["hist"]: a["hist"]["F.Cu"][i, j] += 1.0
        return n

    todo = list(S); pfac = 0.5; BEST = (10 ** 9, {}, {}); best_it = 0
    for it in range(A.iters):
        random.shuffle(todo)
        batches = [todo] if it == 0 else [todo[0::2], todo[1::2]]
        for b in batches:
            res = pool.map(wroute, [(p, pfac, FP.get(p), FPH.get(p), FPF.get(p)) for p in b], chunksize=1)
            for p, r in res: put(p, r)
        bad = {p: n for p, r in R.items() if (n := conflicts(p, r, True))}
        unrouted = [p for p in S if p not in R]
        print(f"round {it + 1}: routed {len(R)}/{len(S)}, overlapping {len(bad)} {sorted(bad)}, pfac {pfac:.1f} ({time.time() - T0:.0f} s)", flush=True)
        if len(bad) + len(unrouted) < BEST[0]:
            BEST = (len(bad) + len(unrouted), dict(R), dict(bad)); best_it = it
        if not bad and not unrouted: break
        if it - best_it >= A.patience or os.path.exists(A.out + ".stop"):
            print(f"no better round in {it - best_it} rounds: stopping", flush=True); break
        todo = sorted(bad) + unrouted
        pfac *= 1.8
    pool.close(); pool.join(); shutil.rmtree(tmpdir, ignore_errors=True)

    # clean-up, as pairnego.py: the best round's occupancy, each pair in conflict re-routed with walls
    _, Rb, bad = BEST
    for d in ("occ", "occh", "occf"):
        for x in a[d].values(): x[:] = 0
    R.clear(); FP.clear(); FPH.clear(); FPF.clear()
    for p, r in Rb.items(): put(p, (r, L["footprint"](r), L["footprint"](r, hard=True), L["fan_fp"](r)))
    L["occh"], L["occf"] = a["occh"], a["occf"]
    for p in sorted(bad, key=bad.get) + [q for q in S if q not in R]:
        if p in R: put(p, None)
        wall = {l: a["occ"][l] + 1000.0 * a["occh"][l] for l in LAYERS}
        r = L["route_one"](p, S[p], wall, {l: np.zeros((NX, NY), dtype=np.float32) for l in LAYERS}, 1.0)
        if r is not None:
            put(p, (r, L["footprint"](r), L["footprint"](r, hard=True), L["fan_fp"](r)))
            n = conflicts(p, r, False)
            if n == 0: print(f"clean-up: {p} re-routed clean on {r['layer']}", flush=True); continue
            put(p, None)
        print(f"clean-up: {p} left unrouted", flush=True)

    # write the copper (pairnego.py's writer)
    g = L["g"]; t = g["t"]; items = []
    for p, r in R.items():
        lg = PL.legs(r["centre"], r["p_left"], r["layer"])
        vp, vn = r["ends"]; fpp, fnp = r["fan"]
        for k, net, b0, v1, fpath in (("P", p + "_P", S[p]["bP"], vp, fpp), ("N", p + "_N", S[p]["bN"], vn, fnp)):
            parts = lg[k]
            segs = [([b0, parts[0][0][0]], parts[0][1])] + parts + [([parts[-1][0][-1], v1], parts[-1][1])]
            for pts, w in segs: items += PL.seg_text(net, r["layer"], w, pts)
            if r["layer"] != "F.Cu":
                items += PL.seg_text(net, "F.Cu", g["FIN_W"], fpath)
                items.append(PL.via_text(net, *v1))
        print(f"{p}: {r['layer']}, {LineString(r['centre']).length:.1f} mm, {len(r['centre']) - 2} bends", flush=True)
    t = PL.append(t, items)
    open(A.out, "w", encoding="utf-8", newline="\n").write(t)
    print("written:", len(R), "of", len(pairs), f"({time.time() - T0:.0f} s)")
    del a
    for s in keep: s.close()
    for s in shms: s.close(); s.unlink()


if __name__ == "__main__":
    main()
