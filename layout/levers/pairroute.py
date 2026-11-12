"""pairroute.py in out PAIR[,PAIR...] [--layers In5.Cu,In2.Cu] [--box x0 y0 x1 y1] [--dry]
Routes LVDS pairs as coupled pairs (pairlib): from the pair's two FPGA ball vias (pairprep.py put
them there) on an inner layer to a pair of vias placed near the far pads, then a short fan-in on
F.Cu to those pads. Pairs are routed in the order given; each sees the ones before it.

Far end: the pads of the pair's nets that are not on U1101 and not its bottom-side termination
(U501/U601 pins, the link's AC capacitors). The via pair is searched in a 4 mm radius: two vias
0.6-1.0 mm apart on a horizontal or vertical axis, each clear of other nets by 0.15 mm on every
layer, each reached from its pad by one straight F.Cu track (0.1 mm) clear of other copper.

Prints per pair: layer, length, vias; writes the board (text edit, no pcbnew). DRC after."""
import os, sys, re, math, argparse
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from geo import Board, CU
import numpy as np
import shapely
from shapely.geometry import Point, LineString
from shapely.ops import unary_union
import pairlib as PL

ap = argparse.ArgumentParser()
ap.add_argument("inp"); ap.add_argument("out"); ap.add_argument("pairs")
ap.add_argument("--layers", default="In5.Cu,In2.Cu")
ap.add_argument("--box", nargs=4, type=float, default=[140, 80, 200, 190])
ap.add_argument("--dry", action="store_true")
ap.add_argument("--limit", type=int, default=3_000_000)
a = ap.parse_args()
PL.BGA = (150.0, 142.0, 166.0, 158.0)
TERMS = {f"R11{k:02d}" for k in range(1, 15)} | {"R101", "R102", "R103", "R104", "R105", "R106"}
LV = re.compile(r"^/(ADCR_|ADCS_|LINK_RX_|LINK_TX_|CLK_ADC|CLK_FPGA_|CONTROL/FPGA_CLK_)")
FIN_W = 0.1
DIRI = {d: k for k, d in enumerate(PL.DIRS)}

t = open(a.inp, encoding="utf-8").read()
added = []          # (geom, net, layer) of this run


def board():
    open(a.out, "w", encoding="utf-8", newline="\n").write(t)
    return Board(a.out)


def via_ok_grid(B, own, x0, y0, x1, y1):
    """Boolean grid of legal via centres (0.45 via, 0.2 drill) for a net of the pair."""
    xs = np.arange(round(x0 / PL.G), round(x1 / PL.G) + 1) * PL.G
    ys = np.arange(round(y0 / PL.G), round(y1 / PL.G) + 1) * PL.G
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    win = shapely.geometry.box(x0 - 1, y0 - 1, x1 + 1, y1 + 1)
    parts = []
    for l in CU:
        for it in B.by_layer[l]:
            if it.net in own or not it.geom.intersects(win): continue
            parts.append(it.geom.buffer(0.225 + max(0.15, B.clr("/ADCR_D0_P", it.net)) + 0.003, 8))
    for g, n, l in added:
        if n not in own and g.intersects(win): parts.append(g.buffer(0.225 + 0.15 + 0.003, 8))
    for p, r, n in B.holes:
        if win.contains(p):
            parts.append(p.buffer(r + 0.1 + 0.25 + 0.003, 12) if n not in own else p.buffer(r + 0.1 + 0.25 + 0.003, 12))
    ok = ~shapely.contains_xy(unary_union(parts), X, Y)
    # board edge: 0.3 mm copper to edge
    edge = unary_union(B.edge)
    ok &= ~shapely.contains_xy(edge.buffer(0.225 + 0.3), X, Y)
    return xs, ys, ok


def fanin_ok(B, net, own, p, v):
    g = LineString([p, v]).buffer(FIN_W / 2, 8)
    for k in B.tree["F.Cu"].query(g.buffer(0.3)):
        it = B.by_layer["F.Cu"][k]
        if it.net in own: continue
        if it.geom.distance(g) < max(0.15, B.clr(net, it.net)): return False
    for gg, n, l in added:
        if l == "F.Cu" and n not in own and gg.distance(g) < 0.15: return False
    return not B.hole_conflicts(g, net)


def fanin_path(B, net, own, p, v, extra=()):
    """Shortest F.Cu track (0.1 mm, 8 directions, 0.05 mm grid) from pad centre p to via v,
    clear of other copper by max(0.15, class), inside a window round the two. None if blocked."""
    x0, y0 = min(p[0], v[0]) - 2.5, min(p[1], v[1]) - 2.5
    x1, y1 = max(p[0], v[0]) + 2.5, max(p[1], v[1]) + 2.5
    win = shapely.geometry.box(x0, y0, x1, y1)
    parts = []
    for k in B.tree["F.Cu"].query(win.buffer(0.5)):
        it = B.by_layer["F.Cu"][k]
        if it.net in own: continue
        parts.append(it.geom.buffer(FIN_W / 2 + max(0.15, B.clr(net, it.net)) + 0.003, 8))
    for gg, n, l in list(added) + list(extra):
        if l == "F.Cu" and n not in own and gg.intersects(win.buffer(0.5)): parts.append(gg.buffer(FIN_W / 2 + 0.153, 8))
    for k in B.htree.query(win.buffer(0.5)):
        q, r, n = B.holes[k]
        if n not in own: parts.append(q.buffer(r + 0.2 + FIN_W / 2 + 0.003, 12))
    G = PL.G
    nx, ny = int((x1 - x0) / G) + 1, int((y1 - y0) / G) + 1
    X, Y = np.meshgrid(x0 + np.arange(nx) * G, y0 + np.arange(ny) * G, indexing="ij")
    blk = shapely.contains_xy(unary_union(parts), X, Y) if parts else np.zeros_like(X, dtype=bool)
    si, sj = int(round((p[0] - x0) / G)), int(round((p[1] - y0) / G))
    ti, tj = int(round((v[0] - x0) / G)), int(round((v[1] - y0) / G))
    blk[si, sj] = False; blk[ti, tj] = False
    import heapq
    dist = {(si, sj): 0.0}; came = {}; pq = [(0.0, si, sj)]
    while pq:
        c, i, j = heapq.heappop(pq)
        if (i, j) == (ti, tj): break
        if c > dist.get((i, j), 1e9): continue
        for di, dj in PL.DIRS:
            a2, b2 = i + di, j + dj
            if not (0 <= a2 < nx and 0 <= b2 < ny) or blk[a2, b2]: continue
            nc = c + (1.4142 if di and dj else 1.0) + (0.3 if came.get((i, j)) and (came[(i, j)][2] != (di, dj)) else 0)
            if nc < dist.get((a2, b2), 1e9):
                dist[(a2, b2)] = nc; came[(a2, b2)] = (i, j, (di, dj)); heapq.heappush(pq, (nc, a2, b2))
    if (ti, tj) not in dist: return None
    path = [(ti, tj)]
    while path[-1] != (si, sj): c = came[path[-1]]; path.append((c[0], c[1]))
    pts = [(round(x0 + i * G, 4), round(y0 + j * G, 4)) for i, j in reversed(path)]
    pts[0] = p; pts[-1] = v
    return PL.simplify(pts)


def far_candidates(B, pair, pP, pN, own, maxn=12, R=3.0):
    win = (min(pP[0], pN[0]) - R, min(pP[1], pN[1]) - R, max(pP[0], pN[0]) + R, max(pP[1], pN[1]) + R)
    xs, ys, okp = via_ok_grid(B, {pair + "_P"}, *win)
    xs, ys, okn = via_ok_grid(B, {pair + "_N"}, *win)
    P = [(round(float(xs[i]), 3), round(float(ys[j]), 3)) for i, j in zip(*np.nonzero(okp))]
    N = {(round(float(xs[i]), 3), round(float(ys[j]), 3)) for i, j in zip(*np.nonzero(okn))}
    cand = []
    for vp in P:
        dp = math.dist(vp, pP)
        if dp > R or dp < 0.3: continue
        for ax in ((1, 0), (0, 1), (-1, 0), (0, -1)):
            for sep in (0.6, 0.7, 0.8, 1.0):
                vn = (round(vp[0] + ax[0] * sep, 3), round(vp[1] + ax[1] * sep, 3))
                if vn not in N: continue
                dn = math.dist(vn, pN)
                if 0.3 <= dn <= R: cand.append((dp + dn, vp, vn))
    cand.sort()
    out = []; tried = 0
    for c, vp, vn in cand:
        if len(out) >= maxn or tried > 300: break
        if any(math.dist(vp, o[1]) < 0.3 for o in out): continue       # spread the candidates
        tried += 1
        fp = fanin_path(B, pair + "_P", {pair + "_P"}, pP, vp,
                        extra=[(Point(vn).buffer(0.225, 16), pair + "_N", "F.Cu")])
        if not fp: continue
        fn = fanin_path(B, pair + "_N", {pair + "_N"}, pN, vn,
                        extra=[(LineString(fp).buffer(FIN_W / 2, 8), pair + "_P", "F.Cu"), (Point(vp).buffer(0.225, 16), pair + "_P", "F.Cu")])
        if not fn: continue
        L = LineString(fp).length + LineString(fn).length
        out.append((L, vp, vn, fp, fn))
    out.sort(key=lambda o: o[0])
    return out


def terminal(vp, vn, g, sign):
    """Centre-line end cell and leaving direction for a via pair (vp, vn), on the side 'sign'.
    The direction is the grid direction nearest to the axis's normal."""
    ax = ((vn[0] - vp[0]), (vn[1] - vp[1])); L = math.hypot(*ax); ax = (ax[0] / L, ax[1] / L)
    n = (-ax[1] * sign, ax[0] * sign)
    u = max(PL.DIRS, key=lambda d: (d[0] * n[0] + d[1] * n[1]) / math.hypot(*d))
    un = (u[0] / math.hypot(*u), u[1] / math.hypot(*u))
    l0 = max(0.1, (L - PL.pitch(g)) / 2)
    m = ((vp[0] + vn[0]) / 2, (vp[1] + vn[1]) / 2)
    c = (round(round((m[0] + un[0] * l0) / PL.G) * PL.G, 4), round(round((m[1] + un[1] * l0) / PL.G) * PL.G, 4))
    return c, u, ax


def route_pair(pair):
    global t
    own = {pair + "_P", pair + "_N"}
    B = board()
    pads = {}
    for it in B.items:
        if it.kind == "pad" and it.net in own and it.data[0] not in TERMS and it.layer in ("F.Cu", "B.Cu"):
            pads.setdefault(it.net, []).append((it.data[0], (round(it.data[2], 4), round(it.data[3], 4))))
    ball = {n: [p for r, p in v if r == "U1101"][0] for n, v in pads.items()}
    far = {n: [p for r, p in v if r != "U1101"] for n, v in pads.items()}
    if len(far[pair + "_P"]) != 1 or len(far[pair + "_N"]) != 1:
        print(pair, "far pads not one each:", far); return False
    fP, fN = far[pair + "_P"][0], far[pair + "_N"][0]
    cands = far_candidates(B, pair, fP, fN, own)
    if not cands:
        print(pair, "no via pair near", fP, fN); return False
    import time
    best = None; T0 = time.time()
    for layer in a.layers.split(","):
        xs = [ball[pair + "_P"][0], fP[0]]; ys = [ball[pair + "_P"][1], fP[1]]
        box = (max(a.box[0], min(xs) - 6), max(a.box[1], min(ys) - 6), min(a.box[2], max(xs) + 6), min(a.box[3], max(ys) + 6))
        ispair = lambda n: bool(LV.match(n)) and n not in own
        F = PL.Field(B, layer, own, box, ispair, extra=[(g, n) for g, n, l in added if l == layer])
        bP, bN = ball[pair + "_P"], ball[pair + "_N"]
        for sign in (1, -1):
            c0, u0, ax0 = terminal(bP, bN, PL.NECK, sign)
            p_left = PL.left_of(c0, (c0[0] + u0[0], c0[1] + u0[1]), bP)
            goals = {}; meta = {}
            for cost, vp, vn, fpp, fnp in cands:
                for s2 in (1, -1):
                    c1, u1, ax1 = terminal(vp, vn, PL.FULL, s2)
                    arr = (-u1[0], -u1[1])           # travel direction when arriving
                    pl = PL.left_of((c1[0] - arr[0], c1[1] - arr[1]), c1, vp)
                    if pl != p_left: continue
                    cell = F.cell(*c1)
                    goals.setdefault(cell, set()).add(DIRI[arr]); meta[(cell, DIRI[arr])] = (vp, vn, c1, cost, fpp, fnp)
            if not goals: continue
            st = F.cell(*c0)
            path, c = PL.astar(F, [(st[0], st[1], DIRI[u0])], goals, limit=a.limit)
            print(f"   {pair} {layer} side {sign}: {'found' if path else 'none'} ({time.time() - T0:.0f} s)", flush=True)
            if not path: continue
            # which goal was reached, arriving in which direction
            (gi, gj), (pi, pj) = path[-1], path[-2]
            d = DIRI[(int(np.sign(gi - pi)), int(np.sign(gj - pj)))]
            vp, vn, c1, fcost, fpp, fnp = meta[((gi, gj), d)]
            total = c * PL.G + fcost
            if best is None or total < best[0]:
                best = (total, layer, [F.xy(i, j) for i, j in path], p_left, (bP, bN), (vp, vn), (fpp, fnp), c0, c1)
    if not best:
        print(pair, "no route"); return False
    total, layer, centre, p_left, (bP, bN), (vp, vn), (fpp, fnp), c0, c1 = best
    centre = PL.simplify(centre)
    lg = PL.legs(centre, p_left)
    items = []; geoms = []
    for k, net, b0, v1 in (("P", pair + "_P", bP, vp), ("N", pair + "_N", bN, vn)):
        parts = lg[k]
        first, last = parts[0][0][0], parts[-1][0][-1]
        segs = [([b0, first], parts[0][1])] + parts + [([last, v1], parts[-1][1])]
        for pts, w in segs:
            items += PL.seg_text(net, layer, w, pts); geoms.append((LineString(pts).buffer(w / 2, 8), net, layer))
        fpath = fpp if k == "P" else fnp
        items += PL.seg_text(net, "F.Cu", FIN_W, fpath); geoms.append((LineString(fpath).buffer(FIN_W / 2, 8), net, "F.Cu"))
        items.append(PL.via_text(net, *v1)); geoms += [(Point(v1).buffer(0.225, 16), net, l) for l in CU]
    t = PL.append(t, items); added.extend(geoms)
    L = sum(LineString(p).length for p, w in lg["P"])
    print(f"{pair}: {layer}, centre line {LineString(centre).length:.1f} mm, {len(centre) - 2} bends, far vias ({vp[0]:g}, {vp[1]:g}) ({vn[0]:g}, {vn[1]:g})", flush=True)
    return True


ok = []
for p in a.pairs.split(","):
    if route_pair("/" + p.strip("/")): ok.append(p)
open(a.out, "w", encoding="utf-8", newline="\n").write(t)
print("routed:", len(ok), "of", len(a.pairs.split(",")), ":", " ".join(ok))
