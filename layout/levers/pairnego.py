"""pairnego.py in out PAIR[,PAIR...] [--iters 30] [--layers In5.Cu,In2.Cu]
Routes the LVDS pairs together by negotiated congestion (PathFinder, McMurchie & Ebeling 1995) on
pairroute.py's pieces: every pair is routed with the others' current routes as a cost, not as an
obstacle; where two pairs still overlap the cost there grows (present factor each round, history
for good), until no pair overlaps another. Then the copper is written as pairroute.py does.

Overlap is measured on the centre lines: pair B's centre line must keep off pair A's centre line
by both half-widths plus the pair-to-pair clearance (0.42 mm under the BGA, 0.64 mm outside), and
off A's far vias by 0.225 + 0.15 + B's half-width. Text edit, no pcbnew; DRC after."""
import os, sys, re, math, time, argparse
HERE = os.path.dirname(os.path.abspath(__file__))
ap = argparse.ArgumentParser()
ap.add_argument("inp"); ap.add_argument("out"); ap.add_argument("pairs")
ap.add_argument("--iters", type=int, default=30)
ap.add_argument("--layers", default="In5.Cu,In2.Cu,F.Cu")
ap.add_argument("--far-r", type=float, default=3.0, help="far via pairs within this of the far pads (6 mm if none)")
ap.add_argument("--pen-cap", type=float, default=0, help="cap on a cell's congestion cost (0: none); keeps A* inside its budget")
ap.add_argument("--maxn", type=int, default=12, help="far via-pair candidates per pair")
ap.add_argument("--only", action="append", default=[], help="REGEX=LAYER[+LAYER]: pairs matching use only those layers")
A = ap.parse_args()
# pairroute's functions, without its main loop
sys.argv = ["pairroute.py", A.inp, A.out, "X", "--layers", A.layers]
src = open(os.path.join(HERE, "pairroute.py"), encoding="utf-8").read().split("\nok = []")[0]
src = src.replace("HERE = os.path.dirname(os.path.abspath(__file__))", f"HERE = {HERE!r}")
g = {"__name__": "pairroute", "__file__": os.path.join(HERE, "pairroute.py")}
exec(compile(src, "pairroute.py", "exec"), g)
PL = g["PL"]; np = g["np"]; shapely = g["shapely"]; LineString = g["LineString"]; Point = g["Point"]
CU = g["CU"]; DIRI = g["DIRI"]

G = PL.G
X0, Y0, X1, Y1 = 140.0, 78.0, 200.0, 190.0
NX, NY = int(round((X1 - X0) / G)) + 1, int(round((Y1 - Y0) / G)) + 1
LAYERS = A.layers.split(",")
HALF_IN = max(PL.half(PL.NECK), PL.half(PL.GEOMS["F.Cu"][1]))
HALF_OUT = max(PL.half(PL.FULL), PL.half(PL.GEOMS["F.Cu"][0]))
R_IN = 2 * HALF_IN + PL.PAIR_CLR["neck"]
R_OUT = 2 * HALF_OUT + PL.PAIR_CLR["full"]
R_VIA = 0.225 + 0.15 + HALF_OUT
# hard limits (DRC): what counts as an overlap; the PAIR_CLR spacing above is only priced
R_IN_H = 2 * HALF_IN + PL.NECK["clr"]
R_OUT_H = 2 * HALF_OUT + PL.FULL["clr"]

B = g["board"]()


def setup(pair):
    """Static per-pair data: terminals, far candidates, per-layer Field over its box."""
    own = {pair + "_P", pair + "_N"}
    pads = {}
    for it in B.items:
        if it.kind == "pad" and it.net in own and it.data[0] not in g["TERMS"] and it.layer in ("F.Cu", "B.Cu"):
            pads.setdefault(it.net, []).append((it.data[0], (round(it.data[2], 4), round(it.data[3], 4))))
    ball = {n: [p for r, p in v if r == "U1101"][0] for n, v in pads.items()}
    far = {n: [p for r, p in v if r != "U1101"] for n, v in pads.items()}
    fP, fN = far[pair + "_P"][0], far[pair + "_N"][0]
    cands = g["far_candidates"](B, pair, fP, fN, own, maxn=A.maxn, R=A.far_r)
    if not cands:       # far pads apart or crowded (the link's AC caps): a longer single-ended fan-in
        cands = g["far_candidates"](B, pair, fP, fN, own, R=6.0)
        if cands: print(pair, f"far vias within 6 mm (fan-in {cands[0][0]:.1f} mm)")
    if not cands: return None
    bP, bN = ball[pair + "_P"], ball[pair + "_N"]
    xs = [bP[0], fP[0], fN[0]]; ys = [bP[1], fP[1], fN[1]]
    box = (max(X0, min(xs) - 6), max(Y0, min(ys) - 6), min(X1, max(xs) + 6), min(Y1, max(ys) + 6))
    ispair = lambda n: bool(g["LV"].match(n)) and n not in own
    Fs = {l: PL.Field(B, l, own, box, ispair) for l in LAYERS}
    return dict(own=own, bP=bP, bN=bN, fP=fP, fN=fN, cands=cands, Fs=Fs)


def gslice(F):
    """F's grid as a slice of the global occupancy grid."""
    i0 = int(round((F.box[0] - X0) / G)); j0 = int(round((F.box[1] - Y0) / G))
    return slice(i0, i0 + F.nx), slice(j0, j0 + F.ny)


def footprint(route, hard=False):
    """Global cells another pair's centre line may not use, per layer, for a route."""
    layer, centre, vias = route["layer"], route["centre"], route["vias"]
    shapes = []
    for zin, pts in PL.zones(centre):
        r = (R_IN_H if zin else R_OUT_H) if hard else (R_IN if zin else R_OUT)
        shapes.append(LineString(pts).buffer(r, 8))
    via_sh = [Point(v).buffer(R_VIA, 16) for v in vias]
    out = {}
    for l in LAYERS:
        sh = shapes + via_sh if l == layer else via_sh       # far vias are through
        if not sh:
            out[l] = (0, 0, np.zeros((0, 0), dtype=bool)); continue
        u = shapely.ops.unary_union(sh)
        bx0, by0, bx1, by1 = u.bounds
        i0 = max(0, int((bx0 - X0) / G)); i1 = min(NX - 1, int((bx1 - X0) / G) + 1)
        j0 = max(0, int((by0 - Y0) / G)); j1 = min(NY - 1, int((by1 - Y0) / G) + 1)
        XX, YY = np.meshgrid(X0 + np.arange(i0, i1 + 1) * G, Y0 + np.arange(j0, j1 + 1) * G, indexing="ij")
        m = shapely.contains_xy(u, XX, YY)
        out[l] = (i0, j0, m)
    return out


def add_fp(occ, fp, s):
    for l, (i0, j0, m) in fp.items():
        occ[l][i0:i0 + m.shape[0], j0:j0 + m.shape[1]] += s * m


FIN_W = g["FIN_W"]


def line_cells(pts):
    L = LineString(pts); out = []
    for k in range(int(L.length / G) + 1):
        q = L.interpolate(k * G); out.append((int(round((q.x - X0) / G)), int(round((q.y - Y0) / G))))
    return out


def fcu_legs(r):
    """A pair routed on F.Cu: its two legs' point lists (ends at the pads included)."""
    lg = PL.legs(r["centre"], r["p_left"], r["layer"])
    out = []
    for k, v1 in (("P", r["ends"][0]), ("N", r["ends"][1])):
        pts = [q for part, w in lg[k] for q in part] + [v1]
        out.append(pts)
    return out


def fan_lines(r):
    """F.Cu lines of a route that other pairs' F.Cu copper must keep off: fan-ins, or the legs on F.Cu."""
    if r["layer"] == "F.Cu": return fcu_legs(r)
    return [f for f in r["fan"] if f]


def fan_fp(r):
    """F.Cu cells another pair's fan-in centre line may not use (DRC: both half-widths + 0.15)."""
    if r["layer"] == "F.Cu":    # a pair on F.Cu: its legs, for the other pairs' fan-ins
        u = shapely.ops.unary_union([LineString(q).buffer(0.06 + 0.15 + FIN_W / 2, 8) for q in fcu_legs(r)])
    elif not r["fan"][0]: return {"F.Cu": (0, 0, np.zeros((0, 0), dtype=bool))}
    else:
        u = shapely.ops.unary_union([LineString(f).buffer(FIN_W + 0.15, 8) for f in r["fan"]] +
                                    [Point(v).buffer(0.225 + 0.15 + FIN_W / 2, 16) for v in r["vias"]])
    bx0, by0, bx1, by1 = u.bounds
    i0 = max(0, int((bx0 - X0) / G)); i1 = min(NX - 1, int((bx1 - X0) / G) + 1)
    j0 = max(0, int((by0 - Y0) / G)); j1 = min(NY - 1, int((by1 - Y0) / G) + 1)
    XX, YY = np.meshgrid(X0 + np.arange(i0, i1 + 1) * G, Y0 + np.arange(j0, j1 + 1) * G, indexing="ij")
    return {"F.Cu": (i0, j0, shapely.contains_xy(u, XX, YY))}


occf = {"F.Cu": np.zeros((NX, NY), dtype=np.float32)}
FPF = {}
CC = {}


def cand_cells(pair, c):
    k = (pair, c[1], c[2])
    if k not in CC:
        CC[k] = (line_cells(c[3]) + line_cells(c[4]) if c[3] else [],
                 [(int(round((v[0] - X0) / G)), int(round((v[1] - Y0) / G))) for v in (c[1], c[2])])
    return CC[k]


def cand_clear(pair, c):
    """No other pair's fan-in on this candidate's fan-ins, no other pair's DRC footprint on its vias."""
    fc, vc = cand_cells(pair, c)
    if any(occf["F.Cu"][i, j] > 0 for i, j in fc): return False
    return not any(occh[l][i, j] > 0 for i, j in vc for l in LAYERS)


def self_ok(r, S):
    """The pair's P copper (ball via to far via, legs and end jogs) keeps 0.095 mm off its N copper."""
    from shapely.ops import unary_union
    lg = PL.legs(r["centre"], r["p_left"], r["layer"])
    geo = {}
    for k, b0, v1 in (("P", S["bP"], r["ends"][0]), ("N", S["bN"], r["ends"][1])):
        parts = lg[k]
        segs = [([b0, parts[0][0][0]], parts[0][1])] + parts + [([parts[-1][0][-1], v1], parts[-1][1])]
        sh = [LineString(q).buffer(w / 2, 8) for q, w in segs if len(q) > 1 and LineString(q).length > 1e-6]
        sh.append(Point(b0).buffer(0.225, 16))          # the ball's via in pad
        if r["layer"] != "F.Cu": sh.append(Point(v1).buffer(0.225, 16))
        geo[k] = unary_union(sh)
    return geo["P"].distance(geo["N"]) >= 0.095


def route_one(pair, S, occ, hist, pfac):
    best = None
    lays = LAYERS
    for spec in A.only:
        rx, _, ly = spec.partition("=")
        if re.search(rx, pair): lays = ly.split("+")
    for layer in lays:
        F = S["Fs"][layer]; si, sj = gslice(F)
        pen = (occ[layer][si, sj] * pfac + hist[layer][si, sj]).astype(np.float32)
        if A.pen_cap: pen = np.minimum(pen, A.pen_cap)
        pen *= 4.0
        full, neck = PL.geoms(layer)
        if layer == "F.Cu":     # the far pads are on F.Cu: the pair ends on them, no vias
            far = [(0.0, S["fP"], S["fN"], None, None)]
        else:
            far = [c for c in S["cands"] if cand_clear(pair, c)] or S["cands"]
        for sign in (1, -1):
            c0, u0, ax0 = g["terminal"](S["bP"], S["bN"], neck, sign)
            p_left = PL.left_of(c0, (c0[0] + u0[0], c0[1] + u0[1]), S["bP"])
            goals = {}; meta = {}
            for cost, vp, vn, fpp, fnp in far:
                # a far via may not sit on another pair's copper: priced like the centre line
                for s2 in (1, -1):
                    c1, u1, ax1 = g["terminal"](vp, vn, full, s2)
                    if layer == "F.Cu":     # out past the pads' ends (QFN pins 0.8 mm long, cap pads 0.6)
                        c1 = (round(round((c1[0] + u1[0] * 0.45) / G) * G, 4), round(round((c1[1] + u1[1] * 0.45) / G) * G, 4))
                    arr = (-u1[0], -u1[1])
                    if PL.left_of((c1[0] - arr[0], c1[1] - arr[1]), c1, vp) != p_left: continue
                    cell = F.cell(*c1)
                    if not F.free(*cell): continue
                    goals.setdefault(cell, set()).add(DIRI[arr]); meta[(cell, DIRI[arr])] = (vp, vn, cost, fpp, fnp)
            if not goals: continue
            st = F.cell(*c0)
            cand = None
            for _try in range(4):       # a route whose own legs touch (a loop at the far vias) loses its goal
                path, c = PL.astar(F, [(st[0], st[1], DIRI[u0])], goals, pen=pen)
                if not path: break
                (gi, gj), (pi, pj) = path[-1], path[-2]
                d = DIRI[(int(np.sign(gi - pi)), int(np.sign(gj - pj)))]
                vp, vn, fcost, fpp, fnp = meta[((gi, gj), d)]
                # the far vias' own penalty (another pair's centre line under them)
                vpen = 0 if layer == "F.Cu" else sum(occ[l][int(round((v[0] - X0) / G)), int(round((v[1] - Y0) / G))] for v in (vp, vn) for l in LAYERS) * pfac * 4
                total = c * G + fcost + vpen * G
                r = dict(cost=total, layer=layer, centre=PL.simplify([F.xy(i, j) for i, j in path]), p_left=p_left,
                         vias=(vp, vn) if layer != "F.Cu" else (), ends=(vp, vn), fan=(fpp, fnp))
                if self_ok(r, S): cand = r; break
                goals.pop((gi, gj), None)
                if not goals: break
            if cand and (best is None or cand["cost"] < best["cost"]): best = cand
    return best


pairs = ["/" + p.strip("/") for p in A.pairs.split(",")]
T0 = time.time()
S = {}
for p in pairs:
    s = setup(p)
    if s is None: print(p, "no via pair near its far pads"); continue
    S[p] = s
print(f"set up {len(S)} pairs ({time.time() - T0:.0f} s)", flush=True)
occ = {l: np.zeros((NX, NY), dtype=np.float32) for l in LAYERS}
occh = {l: np.zeros((NX, NY), dtype=np.float32) for l in LAYERS}
FPH = {}
hist = {l: np.zeros((NX, NY), dtype=np.float32) for l in LAYERS}
R = {}; FP = {}
pfac = 0.5
BEST = (10 ** 9, {}, {})
for it in range(A.iters):
    for p in S:
        if p in FP: add_fp(occ, FP[p], -1); add_fp(occh, FPH[p], -1); add_fp(occf, FPF[p], -1)
        r = route_one(p, S[p], occ, hist, pfac)
        if r is None:
            R.pop(p, None); FP.pop(p, None); FPH.pop(p, None); FPF.pop(p, None); continue
        R[p] = r; FP[p] = footprint(r); add_fp(occ, FP[p], +1)
        FPH[p] = footprint(r, hard=True); add_fp(occh, FPH[p], +1)
        FPF[p] = fan_fp(r); add_fp(occf, FPF[p], +1)
    # overlaps: a pair's centre-line cells inside another pair's footprint
    bad = {}
    for p, r in R.items():
        own = {}
        add_fp_tmp = FP[p]
        for l in LAYERS:
            i0, j0, m = add_fp_tmp[l]
            # occupancy by the others = occ - own
            sub = occ[l][i0:i0 + m.shape[0], j0:j0 + m.shape[1]] - m
        cells = []
        for zin, pts in PL.zones(r["centre"]):
            L = LineString(pts)
            for k in range(int(L.length / G) + 1):
                q = L.interpolate(k * G); cells.append((int(round((q.x - X0) / G)), int(round((q.y - Y0) / G))))
        for v in r["vias"]:
            cells.append((int(round((v[0] - X0) / G)), int(round((v[1] - Y0) / G))))
        n = 0
        for (i, j) in cells:
            for l in ([r["layer"]] if (i, j) not in [] else LAYERS):
                i0, j0, m = FPH[p][l]
                mine = m[i - i0, j - j0] if 0 <= i - i0 < m.shape[0] and 0 <= j - j0 < m.shape[1] else 0
                if occh[l][i, j] - mine > 0:
                    n += 1; hist[l][i, j] += 1.0
                    if os.environ.get("NEGODBG") and n <= 2:
                        who = [q for q in FPH if q != p and 0 <= i - FPH[q][l][0] < FPH[q][l][2].shape[0] and 0 <= j - FPH[q][l][1] < FPH[q][l][2].shape[1] and FPH[q][l][2][i - FPH[q][l][0], j - FPH[q][l][1]]]
                        print(f"   {p} {l} ({X0 + i * G:.2f}, {Y0 + j * G:.2f}) in {who}", flush=True)
        if fan_lines(r):
            i0, j0, m = FPF[p]["F.Cu"]
            for (i, j) in [c for q in fan_lines(r) for c in line_cells(q)]:
                mine = m[i - i0, j - j0] if 0 <= i - i0 < m.shape[0] and 0 <= j - j0 < m.shape[1] else 0
                if occf["F.Cu"][i, j] - mine > 0:
                    n += 1
                    if os.environ.get("NEGODBG") and n <= 2: print(f"   {p} fan-in ({X0 + i * G:.2f}, {Y0 + j * G:.2f})", flush=True)
        if n: bad[p] = n
    unrouted = [p for p in S if p not in R]
    print(f"round {it + 1}: routed {len(R)}/{len(S)}, overlapping {len(bad)} {sorted(bad)[:6]}, pfac {pfac:.1f} ({time.time() - T0:.0f} s)", flush=True)
    if len(bad) + len(unrouted) < BEST[0]:
        BEST = (len(bad) + len(unrouted), dict(R), dict(bad))
    if not bad and not unrouted: break
    pfac *= 1.8
_, R, bad = BEST      # the round with the fewest pairs in conflict


def centre_cells(r):
    cells = []
    for zin, pts in PL.zones(r["centre"]):
        L = LineString(pts)
        for k in range(int(L.length / G) + 1):
            q = L.interpolate(k * G); cells.append((int(round((q.x - X0) / G)), int(round((q.y - Y0) / G))))
    for v in r["vias"]:
        cells.append((int(round((v[0] - X0) / G)), int(round((v[1] - Y0) / G))))
    return cells


# clean-up: rebuild the occupancy of that round, then each pair still in conflict is taken out and
# routed again with the others' DRC footprints as walls (a large cost); it stays only if clean
occ = {l: np.zeros((NX, NY), dtype=np.float32) for l in LAYERS}
occh = {l: np.zeros((NX, NY), dtype=np.float32) for l in LAYERS}
FP = {p: footprint(r) for p, r in R.items()}; FPH = {p: footprint(r, hard=True) for p, r in R.items()}
occf = {"F.Cu": np.zeros((NX, NY), dtype=np.float32)}
FPF = {p: fan_fp(r) for p, r in R.items()}
for p in R: add_fp(occ, FP[p], +1); add_fp(occh, FPH[p], +1); add_fp(occf, FPF[p], +1)
for p in sorted(bad, key=bad.get) + [q for q in S if q not in R]:
    if p in R:
        add_fp(occ, FP.pop(p), -1); add_fp(occh, FPH.pop(p), -1); add_fp(occf, FPF.pop(p), -1); R.pop(p)
    wall = {l: occ[l] + 1000.0 * occh[l] for l in LAYERS}
    r = route_one(p, S[p], wall, {l: np.zeros((NX, NY), dtype=np.float32) for l in LAYERS}, 1.0)
    n = -1
    if r is not None:
        n = sum(1 for (i, j) in centre_cells(r) if occh[r["layer"]][i, j] > 0)
        n += sum(1 for q in fan_lines(r) for (i, j) in line_cells(q) if occf["F.Cu"][i, j] > 0)
    if n == 0:
        R[p] = r; FP[p] = footprint(r); FPH[p] = footprint(r, hard=True); FPF[p] = fan_fp(r)
        add_fp(occ, FP[p], +1); add_fp(occh, FPH[p], +1); add_fp(occf, FPF[p], +1)
        bad.pop(p, None); print(f"clean-up: {p} re-routed clean on {r['layer']}", flush=True)
    else:
        bad.pop(p, None); print(f"clean-up: {p} left unrouted ({n} cells in conflict)", flush=True)

# write the copper
t = g["t"]
items = []
for p, r in R.items():
    if p in bad: print(p, "left out: still overlapping"); continue
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
print("written:", len([p for p in R if p not in bad]), "of", len(pairs))
