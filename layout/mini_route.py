"""mini_route.py board out net [net ...] [--rip REGEX] [--layers ...] [--width W]
A small multi-layer A* router for the last connections (11 Oct 2026).

For each net, its copper islands (pads, tracks, vias, zone outlines) are joined
one at a time: the smallest island to the rest, on a 0.05 mm raster of the
board around the two. Each layer's raster marks what the new track may not
touch: other nets' copper grown by the pairwise class clearance and half the
track width, holes, track keep-outs, the edge. A via may go where its pad
clears every layer and its drill every hole. With --rip, the tracks and vias of
matching nets are not obstacles but cost extra per step; any such net the new
copper touches is ripped whole and routed again afterwards, by the same search,
without ripping (if it fails, it stays unrouted and the run says so).
KiCad's DRC is the judge of the result."""
import sys, os, re, math, uuid, argparse, heapq
sys.path.insert(0, os.path.dirname(__file__))
import numpy as np
import shapely
from shapely.geometry import Point, LineString, box
from shapely.strtree import STRtree
from geo import Board, CU

ap = argparse.ArgumentParser()
ap.add_argument("board"); ap.add_argument("out"); ap.add_argument("nets", nargs="+")
ap.add_argument("--layers", default="F.Cu,In2.Cu,B.Cu", help="not L4: a track there cuts the +3V3_D background")
ap.add_argument("--costs", default="1,1.5,2")
ap.add_argument("--width", type=float, default=None, help="track width (default: 0.1, LVDS 0.117, supplies 0.15)")
ap.add_argument("--step", type=float, default=0.05)
ap.add_argument("--margin", type=float, default=0.02)
ap.add_argument("--pad", type=float, default=4.0, help="mm round the two islands' bounding box")
ap.add_argument("--via-cost", type=float, default=2.0)
ap.add_argument("--rip", default=None, help="regex, or DEFAULT for every Default-class signal net")
ap.add_argument("--rip-cost", type=float, default=0.6, help="mm added per step on rippable copper")
ap.add_argument("--max-rip", type=int, default=3, help="most nets one joint may rip")
ap.add_argument("--depth", type=int, default=2, help="how deep ripped nets may rip others in turn")
a = ap.parse_args()
LAYERS = a.layers.split(","); COST = dict(zip(LAYERS, map(float, a.costs.split(","))))
ST, MG = a.step, a.margin
VIA_R, VIA_DR = 0.225, 0.1
text = open(a.board, encoding="utf-8").read()
pro = os.path.splitext(a.board)[0] + ".kicad_pro"


def load():
    return Board(a.board_tmp if hasattr(a, "board_tmp") else a.board, pro=pro)


def width_for(B, net):
    if a.width: return a.width
    c = B.netclass(net)
    return {"LVDS100": 0.117, "RF50": 0.183, "Power": 0.15}.get(c, 0.1)


def islands(B, net):
    its = [i for i in B.items if i.net == net]
    zs = [(l, p.buffer(-0.35)) for n, l, p in B.zones if n == net]
    N = len(its) + len(zs); par = list(range(N))
    def f(i):
        while par[i] != i: par[i] = par[par[i]]; i = par[i]
        return i
    by = {}
    for k, it in enumerate(its): by.setdefault((it.kind, it.uuid), []).append(k)
    for ks in by.values():
        for k in ks[1:]: par[f(k)] = f(ks[0])
    tr = STRtree([it.geom for it in its]) if its else None
    for k, it in enumerate(its):
        for j in tr.query(it.geom.buffer(1e-4)):
            if j > k and its[j].layer == it.layer and it.geom.distance(its[j].geom) < 1e-4: par[f(j)] = f(k)
        if it.kind in ("via", "pad"):
            for z, (l, p) in enumerate(zs):
                if l == it.layer and p.contains(it.geom.centroid): par[f(len(its) + z)] = f(k)
    groups = {}
    for k in range(N): groups.setdefault(f(k), []).append(k)
    out = []
    for ks in groups.values():
        g = {"items": [its[k] for k in ks if k < len(its)], "zones": [zs[k - len(its)] for k in ks if k >= len(its)]}
        g["pads"] = {it.uuid for it in g["items"] if it.kind == "pad"}
        out.append(g)
    # only islands with pads matter, plus zone islands (a pad reaching a zone is enough)
    return [g for g in out if g["pads"] or g["zones"]]


def bounds(g):
    gs = [it.geom for it in g["items"]] + [z for _, z in g["zones"]]
    return shapely.union_all(gs).bounds if gs else None


def route_join(B, net, src, dst, rip):
    """A* from island src to island dst. Returns (segments, vias, ripped nets) or None."""
    W = width_for(B, net)
    sb, db = bounds(src), bounds(dst)
    if dst["zones"]:   # a plane: search only round the source
        x0, y0, x1, y1 = sb[0] - a.pad, sb[1] - a.pad, sb[2] + a.pad, sb[3] + a.pad
    else:
        x0, y0 = min(sb[0], db[0]) - a.pad, min(sb[1], db[1]) - a.pad
        x1, y1 = max(sb[2], db[2]) + a.pad, max(sb[3], db[3]) + a.pad
    nx, ny = int((x1 - x0) / ST) + 1, int((y1 - y0) / ST) + 1
    X, Y = np.meshgrid(x0 + np.arange(nx) * ST, y0 + np.arange(ny) * ST)   # [j, i]
    area = box(x0 - 1, y0 - 1, x1 + 1, y1 + 1)
    def isrip(n):
        if not rip or n == net or n in PROTECT or not n: return False
        if rip == "DEFAULT":   # any single-ended signal net of the Default class
            return B.netclass(n) == "Default" and n != "GND" and not n.startswith(("+", "-", "unconnected-"))
        return bool(re.search(rip, n))

    def raster(geoms):
        if not geoms: return np.zeros(X.shape, bool)
        u = shapely.union_all(geoms)
        return shapely.contains_xy(u, X, Y)

    hard, soft = {}, {}
    for l in CU:
        hg, sg = [], []
        for k in B.tree[l].query(area):
            it = B.by_layer[l][k]
            if it.net == net and net: continue
            r = B.clr(net, it.net) + W / 2 + MG
            (sg if it.kind != "pad" and isrip(it.net) else hg).append(it.geom.buffer(r, 4))
        for lay, poly, fl in B.keepouts:
            if lay == l and fl.get("tracks") == "not_allowed": hg.append(poly.buffer(W / 2))
        hard[l], soft[l] = raster(hg), raster(sg)
    holes_h, holes_s = [], []
    for k in B.htree.query(area):
        p, r, n = B.holes[k]
        if n == net: continue
        (holes_s if k in B.via_holes and isrip(n) else holes_h).append(p.buffer(r + 0.2 + W / 2 + MG, 8))
    hh, hs = raster(holes_h), raster(holes_s)
    edge = shapely.union_all([e.buffer(0.3 + W / 2 + MG) for e in B.edge])
    he = shapely.contains_xy(edge, X, Y)
    # vias: pad clears every layer, drill clears every hole by 0.25
    vh, vs = np.zeros(X.shape, bool), np.zeros(X.shape, bool)
    for l in CU:
        vg_h, vg_s = [], []
        for k in B.tree[l].query(area):
            it = B.by_layer[l][k]
            if it.net == net and net: continue
            r = B.clr(net, it.net) + VIA_R + MG
            (vg_s if it.kind != "pad" and isrip(it.net) else vg_h).append(it.geom.buffer(r, 4))
        for lay, poly, fl in B.keepouts:
            if lay == l and fl.get("vias") == "not_allowed": vg_h.append(poly)
        vh |= raster(vg_h); vs |= raster(vg_s)
    hv_h, hv_s = [], []
    for k in B.htree.query(area):
        p, r, n = B.holes[k]
        d = r + VIA_DR + 0.25 + MG                          # drill to drill, any net
        if n != net: d = max(d, r + VIA_R + 0.2 + MG)       # via pad to another net's hole
        (hv_s if k in B.via_holes and isrip(n) else hv_h).append(p.buffer(d, 8))
    vh |= raster(hv_h) | shapely.contains_xy(shapely.union_all([e.buffer(0.3 + VIA_R + MG) for e in B.edge]), X, Y)
    vs |= raster(hv_s)
    for l in CU:
        hard[l] = hard[l] | hh | he
        soft[l] = soft[l] | hs
    # sources and goals
    def cells_of(g, layer):
        gs = [it.geom for it in g["items"] if it.layer == layer]
        return raster(gs) if gs else np.zeros(X.shape, bool)
    srcm = {l: cells_of(src, l) for l in LAYERS}
    dstm = {l: cells_of(dst, l) for l in CU}
    zonem = np.zeros(X.shape, bool)
    for l, z in dst["zones"]: zonem |= shapely.contains_xy(z, X, Y)
    srcz = np.zeros(X.shape, bool)
    for l, z in src["zones"]: srcz |= shapely.contains_xy(z, X, Y)
    L = len(LAYERS)
    INF = np.float32(1e9)
    g = np.full((L, ny, nx), INF, np.float32)
    par = np.full((L, ny, nx), -1, np.int64)
    # goal test: a track cell on a dst cell, or a via cell landing on dst on any layer / a dst zone
    dst_any = zonem.copy()
    for l in CU: dst_any |= dstm[l]
    pq = []
    for li, l in enumerate(LAYERS):
        for j, i in zip(*np.nonzero(srcm[l] | srcz & ~hard[l])):
            g[li, j, i] = 0; heapq.heappush(pq, (0.0, li, j, i))
    if not pq: return None
    # heuristic: distance to the nearest dst cell (cheap bound: 0 when dst is a zone)
    if zonem.any(): H = lambda j, i: 0.0
    else:
        dj, di = np.nonzero(dst_any)
        dpts = np.stack([di, dj], 1).astype(np.float32)
        cx, cy = dpts[:, 0].mean(), dpts[:, 1].mean()
        rad = np.sqrt(((dpts - [cx, cy]) ** 2).sum(1)).max()
        H = lambda j, i: max(0.0, math.hypot(i - cx, j - cy) - rad) * ST
    DIRS = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142)]
    goal = None
    nexp = 0
    while pq:
        f_, li, j, i = heapq.heappop(pq)
        gc = g[li, j, i]
        if f_ - H(j, i) > gc + 1e-6: continue
        nexp += 1
        if nexp > 6_000_000: break
        l = LAYERS[li]
        if dstm[l][j, i] and not srcm[l][j, i]:
            goal = (li, j, i); break
        for dx, dy, w in DIRS:
            ii, jj = i + dx, j + dy
            if not (0 <= ii < nx and 0 <= jj < ny) or hard[l][jj, ii]: continue
            nc = gc + ST * w * COST[l] + (a.rip_cost * w if soft[l][jj, ii] else 0)
            if nc < g[li, jj, ii]:
                g[li, jj, ii] = nc; par[li, jj, ii] = (li * ny + j) * nx + i
                heapq.heappush(pq, (nc + H(jj, ii), li, jj, ii))
        if not vh[j, i]:
            vc = a.via_cost + (a.rip_cost * 4 if vs[j, i] else 0)
            if dst_any[j, i] and not any(srcm[x][j, i] for x in LAYERS):
                if not srcz[j, i]:
                    # a via here lands on the target: finish with a via
                    goal = ("via", li, j, i); break
            for lj, l2 in enumerate(LAYERS):
                if lj == li or hard[l2][j, i]: continue
                nc = gc + vc
                if nc < g[lj, j, i]:
                    g[lj, j, i] = nc; par[lj, j, i] = (li * ny + j) * nx + i
                    heapq.heappush(pq, (nc + H(j, i), lj, j, i))
    if goal is None:
        print(f"    search: {nexp} expansions, grid {nx}x{ny}, src cells {sum(int(m.sum()) for m in srcm.values())}, dst cells {int(dst_any.sum())}", flush=True)
        return None
    endvia = goal[0] == "via"
    if endvia: goal = goal[1:]
    # walk back
    path = [goal]
    while True:
        li, j, i = path[-1]
        p = par[li, j, i]
        if p < 0: break
        path.append((int(p // (ny * nx)), int(p // nx % ny), int(p % nx)))
    path.reverse()
    pt = lambda j, i: (round(float(x0 + i * ST), 4), round(float(y0 + j * ST), 4))
    segs, vias = [], []
    run = [path[0]]
    def flush(run):
        if len(run) < 2: return
        pts = [run[0]]
        for k in range(1, len(run) - 1):
            d1 = (run[k][1] - pts[-1][1], run[k][2] - pts[-1][2]); d2 = (run[k + 1][1] - run[k][1], run[k + 1][2] - run[k][2])
            if d1[0] * d2[1] - d1[1] * d2[0] != 0: pts.append(run[k])
        pts.append(run[-1])
        for k in range(len(pts) - 1):
            segs.append((LAYERS[run[0][0]], pt(pts[k][1], pts[k][2]), pt(pts[k + 1][1], pts[k + 1][2])))
    for k in range(1, len(path)):
        if path[k][0] != path[k - 1][0]:
            flush(run); vias.append(pt(path[k][1], path[k][2])); run = [path[k]]
        else: run.append(path[k])
    flush(run)
    if endvia: vias.append(pt(path[-1][1], path[-1][2]))
    # which rippable nets does the new copper touch?
    ripped = set()
    if rip:
        geo_new = [(l, LineString([p, q]).buffer(W / 2)) for l, p, q in segs] + \
                  [(l, Point(v).buffer(VIA_R)) for v in vias for l in CU]
        for l, gn in geo_new:
            for k in B.tree[l].query(gn.buffer(0.3)):
                it = B.by_layer[l][k]
                if it.kind != "pad" and isrip(it.net) and gn.distance(it.geom) < B.clr(net, it.net) + 1e-3:
                    ripped.add(it.net)
        for v in vias:
            for k in B.htree.query(Point(v).buffer(0.6)):
                p, r, n = B.holes[k]
                if k in B.via_holes and isrip(n) and Point(v).distance(p) - r - VIA_DR < 0.25: ripped.add(n)
    return segs, vias, ripped, W


def emit(segs, vias, net, W):
    s = ""
    for l, p, q in segs:
        if p == q: continue
        s += (f'\n\t(segment\n\t\t(start {p[0]} {p[1]})\n\t\t(end {q[0]} {q[1]})\n\t\t(width {W})\n'
              f'\t\t(layer "{l}")\n\t\t(net "{net}")\n\t\t(uuid "{uuid.uuid4()}")\n\t)')
    for v in vias:
        s += (f'\n\t(via\n\t\t(at {v[0]} {v[1]})\n\t\t(size 0.45)\n\t\t(drill 0.2)\n'
              f'\t\t(layers "F.Cu" "B.Cu")\n\t\t(net "{net}")\n\t\t(uuid "{uuid.uuid4()}")\n\t)')
    return s


def rip_nets(t, nets):
    return re.sub(r"\n\t\((?:segment|via|arc)\n.*?\n\t\)",
                  lambda m: "" if re.search(r'\(net "([^"]*)"\)', m.group(0)).group(1) in nets else m.group(0),
                  t, flags=re.S)


tmp = a.out + ".work.kicad_pcb"
open(tmp, "w", encoding="utf-8", newline="\n").write(text)
a.board_tmp = tmp


def route_net(net, rip, budget=8):
    """Join all islands of net. Returns (ok, ripped nets)."""
    global text
    allripped = set()
    for _ in range(budget):
        B = load()
        isl = islands(B, net)
        if len(isl) <= 1: return True, allripped
        isl.sort(key=lambda g: (len(g["pads"]) + 1000 * len(g["zones"])))
        src = isl[0]
        rest = {"items": [it for g in isl[1:] for it in g["items"]], "zones": [z for g in isl[1:] for z in g["zones"]]}
        rest["pads"] = {p for g in isl[1:] for p in g["pads"]}
        r = route_join(B, net, src, rest, rip)
        if r is None:
            print(f"  {net}: no path from {sorted(src['pads'])[:3]}", flush=True)
            return False, allripped
        segs, vias, ripped, W = r
        if len(ripped) > a.max_rip:
            print(f"  {net}: path would rip {len(ripped)} nets, skipped", flush=True)
            return False, allripped
        t = rip_nets(text, ripped) if ripped else text
        i = t.find("\n\t(zone")
        text = t[:i] + emit(segs, vias, net, W) + t[i:]
        open(tmp, "w", encoding="utf-8", newline="\n").write(text)
        allripped |= ripped
        print(f"  {net}: joined {sorted(src['pads'])[:3]}: {len(segs)} segments, {len(vias)} vias"
              + (f", ripped {sorted(ripped)}" if ripped else ""), flush=True)
    return False, allripped


PROTECT = set()


def save(t_):
    global text
    text = t_
    open(tmp, "w", encoding="utf-8", newline="\n").write(text)


def try_net(net, depth, ind=""):
    """Route net; ripped nets are routed again in turn. All or nothing: if a ripped
    net cannot be routed again, the board goes back to what it was."""
    print(f"{ind}{net}", flush=True)
    ok, _ = route_net(net, None)
    if ok: return True
    if not a.rip or depth <= 0: return False
    before = text
    PROTECT.add(net)
    try:
        ok, ripped = route_net(net, a.rip)
        if ok:
            for rn in sorted(ripped):
                if not try_net(rn, depth - 1, ind + "  "):
                    ok = False; break
        if not ok and text != before:
            print(f"{ind}  {net}: rolled back", flush=True)
            save(before)
        return ok
    finally:
        PROTECT.discard(net)


done, failed = [], []
for net in a.nets:
    (done if try_net(net, a.depth) else failed).append(net)
B = load()
left = [n for n in a.nets if len(islands(B, n)) > 1]
open(a.out, "w", encoding="utf-8", newline="\n").write(text)
os.remove(tmp)
print(f"done {len(done)}; still incomplete: {len(left)} {left}")
