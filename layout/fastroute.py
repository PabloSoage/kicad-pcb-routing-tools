"""Usage:  python layout/fastroute.py in out NET x y LAYER x0 y0 x1 y1 [--step 0.05] [--width 0.1]
       [--layers F.Cu,In2.Cu,B.Cu] [--via-cost 1.0] [--extra 0.02] [--wp x y LAYER]... [--cost x0 y0 x1 y1 C]... [--layer-cost LAYER C]...
handroute.py on a raster: the obstacles of each layer (other nets' copper grown by the netclass
clearance plus half the track, holes, keep-outs, edge) and the via spots are drawn once into grids
at --step, so the A* only looks pixels up. It routes from (x, y) on LAYER through the waypoints in
order (--wp: a point the path must pass, to make a leg longer for length matching) to any copper of
NET outside the start point's island, inside the box. --cost adds C per mm inside a box (steer
away from an area), --layer-cost C per mm on a layer (In3.Cu, the power layer, only where needed). The path is then checked against the exact geometry (geo.Board, as
handroute.py) and any conflict is printed; run the DRC on the result all the same."""
import os, sys, re, math, heapq, uuid, argparse
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from geo import Board, CU
import numpy as np
from PIL import Image, ImageDraw
from scipy.ndimage import distance_transform_edt, binary_erosion
import shapely
from shapely.geometry import Point, LineString, Polygon, box as sbox
from shapely.ops import unary_union

ap = argparse.ArgumentParser()
ap.add_argument("inp"); ap.add_argument("out"); ap.add_argument("net")
ap.add_argument("x", type=float); ap.add_argument("y", type=float); ap.add_argument("layer")
ap.add_argument("box", nargs=4, type=float)
ap.add_argument("--step", type=float, default=0.05)
ap.add_argument("--width", type=float, default=0.1)
ap.add_argument("--layers", default="F.Cu,In2.Cu,B.Cu")
ap.add_argument("--via-cost", type=float, default=1.0)
ap.add_argument("--extra", type=float, default=0.02)
ap.add_argument("--wp", nargs=3, action="append", default=[])
ap.add_argument("--cost", nargs=5, type=float, action="append", default=[])
ap.add_argument("--box-cost", nargs=6, action="append", default=[], help="x0 y0 x1 y1 C LAYER: C per mm in the box on that layer")
ap.add_argument("--layer-cost", nargs=2, action="append", default=[], help="LAYER C: C extra per mm on that layer")
a = ap.parse_args()
B = Board(a.inp)
LAY = a.layers.split(",")
W, VS, VD, ST, EX = a.width, 0.45, 0.2, a.step, a.extra
x0, y0, x1, y1 = a.box
NX, NY = int(round((x1 - x0) / ST)) + 1, int(round((y1 - y0) / ST)) + 1
BOX = sbox(x0 - 1, y0 - 1, x1 + 1, y1 + 1)


def raster(geoms):
    """Boolean grid, True where a grid point lies inside any geometry. Pillow's fill also marks
    pixels just outside an edge, so every pixel near an edge is tested again exactly."""
    im = Image.new("1", (NX, NY), 0); d = ImageDraw.Draw(im)
    band = Image.new("1", (NX, NY), 0); e = ImageDraw.Draw(band)
    polys = []
    def px(c): return [((x - x0) / ST, (y - y0) / ST) for x, y in c]
    for g in geoms:
        if g.is_empty: continue
        for p in getattr(g, "geoms", [g]):
            if p.geom_type != "Polygon" or len(p.exterior.coords) < 4: continue
            polys.append(p)
            d.polygon(px(p.exterior.coords), fill=1, outline=1)
            e.line(px(p.exterior.coords), fill=1, width=3)   # holes in an obstacle stay blocked
    m = np.array(im, dtype=bool)
    if not polys: return m
    jj, ii = np.nonzero(m & np.array(band, dtype=bool))
    u = unary_union(polys)
    shapely.prepare(u)
    inside = shapely.contains_xy(u, x0 + ii * ST, y0 + jj * ST)
    m[jj[~inside], ii[~inside]] = False
    return m


def grid_pt(x, y): return int(round((x - x0) / ST)), int(round((y - y0) / ST))
def xy(i, j): return round(x0 + i * ST, 4), round(y0 + j * ST, 4)


# islands of the net (as handroute.py): the start island is not a target
mine = [i for i in B.items if i.net == a.net and i.geom.intersects(BOX)]
grp = list(range(len(mine)))
def find(k):
    while grp[k] != k: grp[k] = grp[grp[k]]; k = grp[k]
    return k
for i in range(len(mine)):
    for j in range(i + 1, len(mine)):
        p, q = mine[i], mine[j]
        if (p.layer == q.layer and p.geom.intersects(q.geom)) or (p.kind in ("via", "pad") and p.uuid == q.uuid):
            grp[find(i)] = find(j)
sp = Point(a.x, a.y)
start_g = {find(k) for k, i in enumerate(mine) if i.layer == a.layer and i.geom.distance(sp) < 1e-6}
print("islands", len({find(k) for k in range(len(mine))}), "start group", start_g)

# obstacles per layer, for the track centre line
blk, tgt = {}, {}
holes = [(p, r, n) for p, r, n in B.holes if n != a.net and BOX.contains(p)]
# the edge: a closed outline is the board; the usable area is its inside less the edge clearance
OUT = [Polygon(e.coords) for e in B.edge if e.is_ring]
OPEN = [e for e in B.edge if not e.is_ring and e.intersects(BOX.buffer(1))]
def outside(r):
    """Grid of points too close to the edge for copper reaching r from its centre line."""
    m = np.zeros((NY, NX), bool)
    if OUT: m |= ~raster([o.buffer(-(0.3 + r)) for o in OUT])
    if OPEN: m |= raster([e.buffer(0.3 + r) for e in OPEN])
    return m
for l in LAY:
    g = [it.geom.buffer(B.clr(a.net, it.net) + W / 2 + EX, 8) for it in B.by_layer[l]
         if it.net != a.net and it.geom.intersects(BOX)]
    g += [p.buffer(r + 0.2 + W / 2 + EX, 12) for p, r, n in holes]
    g += [poly.buffer(W / 2 + EX) for L, poly, f in B.keepouts if L == l and f.get("tracks") == "not_allowed"]
    blk[l] = raster(g) | outside(W / 2 + EX)
    tgt[l] = raster([i.geom.buffer(-0.01) for k, i in enumerate(mine) if i.layer == l and find(k) not in start_g])
# via spots: clear of other copper on every layer, of every hole, of via keep-outs and the edge
vg = [it.geom.buffer(B.clr(a.net, it.net) + VS / 2 + EX, 8) for it in B.items
      if it.net != a.net and it.geom.intersects(BOX)]
vg += [p.buffer(r + VD / 2 + 0.25 + EX, 12) for p, r, n in B.holes if BOX.contains(p)]
vg += [p.buffer(r + 0.2 + VS / 2 + EX, 12) for p, r, n in holes]
vg += [poly.buffer(VS / 2 + EX) for L, poly, f in B.keepouts if f.get("vias") == "not_allowed"]
vblk = raster(vg) | outside(VS / 2 + EX)
extra_cost = np.zeros((NY, NX))
for cx0, cy0, cx1, cy1, c in a.cost:
    i0, j0 = grid_pt(cx0, cy0); i1, j1 = grid_pt(cx1, cy1)
    extra_cost[max(j0, 0):j1 + 1, max(i0, 0):i1 + 1] += c
LI = {l: k for k, l in enumerate(LAY)}
LCOST = [0.0] * len(LAY)
for l, c in a.layer_cost: LCOST[LI[l]] = float(c)
LBOX = [np.zeros((NY, NX)) for _ in LAY]
for *bx, c, l in a.box_cost:
    i0, j0 = grid_pt(float(bx[0]), float(bx[1])); i1, j1 = grid_pt(float(bx[2]), float(bx[3]))
    LBOX[LI[l]][max(j0, 0):j1 + 1, max(i0, 0):i1 + 1] += float(c)
DIRS = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]


def astar(start, goal_mask, goal_any, used):
    """start (i, j, layer index) -> a cell where goal_mask[layer][j, i] (or any layer if goal_any)."""
    gm = np.zeros((NY, NX), bool)
    for m in goal_mask: gm |= m
    hgrid = distance_transform_edt(~gm) * ST if gm.any() else np.zeros((NY, NX))
    openq = [(hgrid[start[1], start[0]], 0.0, start)]
    came, cost = {start: None}, {start: 0.0}
    n = 0
    while openq:
        f, c, u = heapq.heappop(openq)
        if c > cost[u] + 1e-9: continue
        n += 1
        i, j, k = u
        if u != start and goal_mask[k][j, i]: return u, came, n
        for di, dj in DIRS:
            ni, nj = i + di, j + dj
            if not (0 <= ni < NX and 0 <= nj < NY): continue
            b = blk[LAY[k]]
            if b[nj, ni] or (u != start and b[j, i]): continue
            if di and dj and (b[j, ni] or b[nj, i]): continue   # no squeezing between two corners
            if (ni, nj, k) in used: continue
            step = ST * (1.4142 if di and dj else 1.0) * (1 + extra_cost[nj, ni] + LCOST[k] + LBOX[k][nj, ni])
            pu = came.get(u)
            if pu and pu[2] == k and (i - pu[0], j - pu[1]) != (di, dj): step += 0.02
            v = (ni, nj, k)
            if c + step < cost.get(v, 1e18) - 1e-9:
                cost[v] = c + step; came[v] = u
                heapq.heappush(openq, (c + step + hgrid[nj, ni], c + step, v))
        if not vblk[j, i] and u != start:
            for k2 in range(len(LAY)):
                if k2 == k or blk[LAY[k2]][j, i]: continue
                v = (i, j, k2)
                if c + a.via_cost < cost.get(v, 1e18) - 1e-9:
                    cost[v] = c + a.via_cost; came[v] = u
                    heapq.heappush(openq, (c + a.via_cost + hgrid[j, i], c + a.via_cost, v))
    return None, came, n


si, sj = grid_pt(a.x, a.y)
cur = (si, sj, LI[a.layer])
path, used = [cur], set()
for wx, wy, wl in a.wp:
    m = [np.zeros((NY, NX), bool) for _ in LAY]
    wi, wj = grid_pt(float(wx), float(wy)); m[LI[wl]][wj, wi] = True
    end, came, n = astar(cur, m, False, used)
    print("waypoint", wx, wy, wl, "expanded", n, "found" if end else "NO PATH")
    if not end: sys.exit(2)
    leg = []
    while end != cur: leg.append(end); end = came[end]
    leg.reverse(); path += leg; used |= set(leg); cur = path[-1]
end, came, n = astar(cur, [tgt[l] for l in LAY], True, used)
print("target expanded", n)
if not end:
    print("NO PATH"); sys.exit(2)
leg = []
while end != cur: leg.append(end); end = came[end]
leg.reverse(); path += leg
P = [(*xy(i, j), LAY[k]) for i, j, k in path]
P[0] = (a.x, a.y, a.layer)
# a path that ends on (or a grid step off) a pad's edge goes on to the pad's centre (KiCad counts it connected then)
for it in mine:
    if it.kind == "pad" and it.layer == P[-1][2] and it.geom.distance(Point(P[-1][:2])) < ST:
        P.append((round(it.data[2], 4), round(it.data[3], 4), P[-1][2])); break

# string-pulling: on each layer run, jump to the farthest later point reachable by one clear
# segment at a multiple of 45 degrees (the grid's jitter goes, the octilinear look stays)
def clear(l, p, q):
    g = LineString([p, q]).buffer(W / 2, 8)
    return not (B.conflicts(l, g, a.net) or B.hole_conflicts(g, a.net) or B.keepout_hit(l, g, "tracks")
                or not B.edge_ok(g))
def octo(p, q):
    dx, dy = abs(q[0] - p[0]), abs(q[1] - p[1])
    return dx < 1e-6 or dy < 1e-6 or abs(dx - dy) < 1e-6
Q, i = [P[0]], 0
while i < len(P) - 1:
    j = i + 1
    if P[j][2] == P[i][2]:
        k = j
        while k + 1 < len(P) and P[k + 1][2] == P[i][2]: k += 1
        for m in range(k, i, -1):
            if m == i + 1 or (octo(P[i], P[m]) and clear(P[i][2], P[i][:2], P[m][:2])):
                j = m; break
    Q.append(P[j]); i = j
P = Q

# merge into straight runs and vias
segs, vias = [], []
run = [P[0]]
for p in P[1:]:
    q = run[-1]
    if p[2] != q[2]:
        if len(run) > 1: segs.append((run[0], run[-1]))
        vias.append((p[0], p[1])); run = [p]; continue
    if len(run) >= 2:
        r = run[-2]
        d1 = (q[0] - r[0], q[1] - r[1]); d2 = (p[0] - q[0], p[1] - q[1])
        if abs(d1[0] * d2[1] - d1[1] * d2[0]) < 1e-9:
            run[-1] = p; continue
        segs.append((run[0], run[-1])); run = [run[-1], p]; continue
    run.append(p)
if len(run) > 1: segs.append((run[0], run[-1]))

# exact check
bad = 0
for s, e in segs:
    g = LineString([s[:2], e[:2]]).buffer(W / 2, 8)
    c = B.conflicts(s[2], g, a.net) + B.hole_conflicts(g, a.net)
    if c or B.keepout_hit(s[2], g, "tracks") or not B.edge_ok(g):
        bad += 1; print("  CONFLICT", s, e, [getattr(i, "net", "hole") for i in c][:4])
for v in vias:
    g = Point(v).buffer(VS / 2, 16)
    c = [i for l in CU for i in B.conflicts(l, g, a.net)] + B.hole_conflicts(g, a.net, hole=(Point(v), VD / 2))
    if c or any(B.keepout_hit(l, g, "vias") for l in CU): bad += 1; print("  VIA CONFLICT", v)
L = sum(math.hypot(e[0] - s[0], e[1] - s[1]) for s, e in segs)
print(f"PATH {len(segs)} segments, {len(vias)} vias, {L:.2f} mm, {bad} conflicts")
txt = ""
for s, e in segs:
    txt += (f'\t(segment\n\t\t(start {s[0]:.6f} {s[1]:.6f})\n\t\t(end {e[0]:.6f} {e[1]:.6f})\n\t\t(width {W})\n'
            f'\t\t(layer "{s[2]}")\n\t\t(net "{a.net}")\n\t\t(uuid "{uuid.uuid4()}")\n\t)\n')
for x, y in vias:
    txt += (f'\t(via\n\t\t(at {x:.6f} {y:.6f})\n\t\t(size {VS})\n\t\t(drill {VD})\n\t\t(layers "F.Cu" "B.Cu")\n'
            f'\t\t(net "{a.net}")\n\t\t(uuid "{uuid.uuid4()}")\n\t)\n')
t = B.t
k = t.index("\n\t(segment\n")
open(a.out, "w", encoding="utf-8", newline="\n").write(t[:k + 1] + txt + t[k + 1:])
print("written", a.out)
