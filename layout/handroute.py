"""Usage:  python layout/handroute.py in out NET x y LAYER x0 y0 x1 y1 [--step 0.05] [--width 0.1] [--layers F.Cu,In2.Cu,B.Cu]
       [--via-cost 1.0] [--extra 0.005]
A* for one connection of NET: from (x, y) on LAYER to any copper of NET not touching the start
point's island, inside the box, on the given signal layers with through vias. Every step is checked
against the board's netclass clearances (geo.Board: tracks, pads, vias, holes, keep-outs, edge).
Zones are not obstacles (they refill round the new copper). Writes the path into out.
--soft lets the path cross other nets' tracks and vias (never pads) at a cost and lists them in
out.crossed, to find the fewest items to move by hand; --rip deletes those of the given nets that
the path hits. Run with the router's venv (shapely). It closed the last two connections (10 Nov)."""
import os, sys, re, math, heapq, uuid, argparse
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from geo import Board, CU
from shapely.geometry import Point, LineString
from shapely.ops import unary_union

ap = argparse.ArgumentParser()
ap.add_argument("inp"); ap.add_argument("out"); ap.add_argument("net")
ap.add_argument("x", type=float); ap.add_argument("y", type=float); ap.add_argument("layer")
ap.add_argument("box", nargs=4, type=float)
ap.add_argument("--step", type=float, default=0.05)
ap.add_argument("--width", type=float, default=0.1)
ap.add_argument("--layers", default="F.Cu,In2.Cu,B.Cu")
ap.add_argument("--via-cost", type=float, default=1.0)
ap.add_argument("--extra", type=float, default=0.005)
ap.add_argument("--soft", default=None, help="regex: tracks/vias of these nets may be crossed at --soft-cost per step; reported, not removed")
ap.add_argument("--soft-cost", type=float, default=3.0)
ap.add_argument("--rip", default=None, help="regex: tracks/vias of these nets are not obstacles; those the path hits are deleted")
a = ap.parse_args()
B = Board(a.inp, ignore=a.rip)
LAY = a.layers.split(",")
W, VS, VD, ST, EX = a.width, 0.45, 0.2, a.step, a.extra
x0, y0, x1, y1 = a.box

# islands of the net: union per layer, connected through vias/through pads (same point on all layers)
from shapely.geometry import box as _box
BOX = _box(x0, y0, x1, y1)
mine = [i for i in B.items if i.net == a.net and i.geom.intersects(BOX)]
groups = list(range(len(mine)))
def find(k):
    while groups[k] != k:
        groups[k] = groups[groups[k]]; k = groups[k]
    return k
for i in range(len(mine)):
    for j in range(i + 1, len(mine)):
        p, q = mine[i], mine[j]
        same = p.layer == q.layer and p.geom.intersects(q.geom)
        thru = p.kind in ("via", "pad") and p.uuid == q.uuid   # one via or pad on several layers
        if same or thru: groups[find(i)] = find(j)
sp = Point(a.x, a.y)
start_g = {find(k) for k, i in enumerate(mine) if i.layer == a.layer and i.geom.distance(sp) < 1e-6}
target = {l: unary_union([i.geom for k, i in enumerate(mine) if i.layer == l and find(k) not in start_g])
          for l in CU}
print("islands", len({find(k) for k in range(len(mine))}), "start group", start_g)


VIAC = {}
for i in B.items:
    if i.kind == "via": VIAC[(round(i.data[0], 4), round(i.data[1], 4))] = i
def soft_split(items, holes):
    """(hard?, soft uuids) for conflicting items and hole points."""
    hits = set()
    for it in items:
        if a.soft and it.kind in ("seg", "via") and re.search(a.soft, it.net): hits.add(it.uuid)
        else: return True, hits
    for hp in holes:
        v = VIAC.get((round(hp.x, 4), round(hp.y, 4)))
        if a.soft and v is not None and re.search(a.soft, v.net): hits.add(v.uuid)
        else: return True, hits
    return False, hits


def seg_ok(l, p, q):
    g = LineString([p, q]).buffer(W / 2, 8)
    if B.keepout_hit(l, g, "tracks") or not B.edge_ok(g): return None
    hard, hits = soft_split(B.conflicts(l, g, a.net, EX), B.hole_conflicts(g, a.net, extra=EX))
    return None if hard else hits


vcache = {}
def via_ok(p):
    if p in vcache: return vcache[p]
    g = Point(p).buffer(VS / 2, 16)
    res = None
    if all(not B.keepout_hit(l, g, "vias") for l in CU) and B.edge_ok(g):
        its = [it for l in CU for it in B.conflicts(l, g, a.net, EX)]
        hard, hits = soft_split(its, B.hole_conflicts(g, a.net, hole=(Point(p), VD / 2), extra=EX))
        res = None if hard else hits
    vcache[p] = res; return res


def snap(v): return round(round(v / ST) * ST, 4)
s0 = (a.x, a.y, a.layer)
goal_pts = [g.representative_point() for g in target.values() if not g.is_empty]
def h(n):
    return min(Point(n[0], n[1]).distance(g) for g in target.values() if not g.is_empty)
dirs = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
openq = [(h(s0), 0.0, s0)]
came, cost = {s0: None}, {s0: 0.0}
found = None; n = 0
while openq:
    f, c, u = heapq.heappop(openq)
    if c > cost.get(u, 1e9) + 1e-9: continue
    n += 1
    if n % 20000 == 0: print("expanded", n, "best h", round(f - c, 3), flush=True)
    ux, uy, ul = u
    if u != s0 and not target[ul].is_empty and target[ul].distance(Point(ux, uy)) < 1e-6:
        found = u; break
    nbrs = []
    if u == s0:   # leave the start point in any direction onto the grid
        bx, by = snap(ux), snap(uy)
        for dx in (-ST, 0, ST):
            for dy in (-ST, 0, ST):
                nbrs.append(((round(bx + dx, 4), round(by + dy, 4), ul), None))
    else:
        for dx, dy in dirs:
            nbrs.append(((round(ux + dx * ST, 4), round(uy + dy * ST, 4), ul), None))
        for l in LAY:
            if l != ul: nbrs.append(((ux, uy, l), "via"))
    for v, kind in nbrs:
        vx, vy, vl = v
        if not (x0 <= vx <= x1 and y0 <= vy <= y1): continue
        if kind == "via":
            hits = via_ok((vx, vy))
            if hits is None: continue
            step = a.via_cost + a.soft_cost * len(hits)
        else:
            hits = seg_ok(vl, (ux, uy), (vx, vy))
            if hits is None: continue
            step = math.hypot(vx - ux, vy - uy) + a.soft_cost * len(hits)
            # small bend penalty keeps the path straight
            pu = came.get(u)
            if pu and pu[2] == ul and (pu[0], pu[1]) != (ux, uy):
                d1 = (ux - pu[0], uy - pu[1]); d2 = (vx - ux, vy - uy)
                if abs(d1[0] * d2[1] - d1[1] * d2[0]) > 1e-9: step += 0.02
        nc = c + step
        if nc < cost.get(v, 1e9) - 1e-9:
            cost[v] = nc; came[v] = u
            heapq.heappush(openq, (nc + h(v), nc, v))
print("expanded", n)
if not found:
    print("NO PATH"); sys.exit(2)
path = []
u = found
while u: path.append(u); u = came[u]
path.reverse()
# merge collinear runs, emit segments and vias
segs, vias = [], []
run = [path[0]]
for p in path[1:]:
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
crossed = set()
for k in range(1, len(path)):
    u, v = path[k - 1], path[k]
    h_ = via_ok((v[0], v[1])) if u[2] != v[2] else seg_ok(v[2], (u[0], u[1]), (v[0], v[1]))
    crossed |= h_ or set()
if crossed:
    byid = {i.uuid: i for i in B.items}
    from collections import Counter
    print("CROSSES", dict(Counter(byid[u].net for u in crossed)))
    open(a.out + ".crossed", "w").write("\n".join(sorted(crossed)))
L = sum(math.hypot(e[0] - s[0], e[1] - s[1]) for s, e in segs)
print(f"PATH {len(segs)} segments, {len(vias)} vias, {L:.2f} mm")
txt = ""
for s, e in segs:
    print(f"  {s[2]} ({s[0]:.3f}, {s[1]:.3f}) -> ({e[0]:.3f}, {e[1]:.3f})")
    txt += (f'\t(segment\n\t\t(start {s[0]:.6f} {s[1]:.6f})\n\t\t(end {e[0]:.6f} {e[1]:.6f})\n\t\t(width {W})\n'
            f'\t\t(layer "{s[2]}")\n\t\t(net "{a.net}")\n\t\t(uuid "{uuid.uuid4()}")\n\t)\n')
for x, y in vias:
    print(f"  via ({x:.3f}, {y:.3f})")
    txt += (f'\t(via\n\t\t(at {x:.6f} {y:.6f})\n\t\t(size {VS})\n\t\t(drill {VD})\n\t\t(layers "F.Cu" "B.Cu")\n'
            f'\t\t(net "{a.net}")\n\t\t(uuid "{uuid.uuid4()}")\n\t)\n')
t = B.t
if a.rip:   # delete ripped-net tracks/vias that now conflict with the new copper
    from shapely.geometry import LineString as LS
    new = [(s[2], LS([s[:2], e[:2]]).buffer(W / 2, 8)) for s, e in segs] + [(l, Point(v).buffer(VS / 2, 16)) for v in vias for l in CU]
    full = Board(a.inp)
    gone = set()
    for i in full.items:
        if i.kind == "pad" or not re.search(a.rip, i.net): continue
        for l, g in new:
            if i.layer == l and g.distance(i.geom) < full.clr(a.net, i.net) + EX: gone.add(i.uuid)
    def drop(m):
        u = re.search(r'\(uuid "([^"]+)"\)', m.group(0)).group(1)
        return "" if u in gone else m.group(0)
    t = re.sub(r"\n\t\((?:segment|via|arc)\n.*?\n\t\)", drop, t, flags=re.S)
    print("ripped", len(gone), "items of", a.rip)
k = t.index("\n\t(segment\n")
open(a.out, "w", encoding="utf-8", newline="\n").write(t[:k + 1] + txt + t[k + 1:])
print("written", a.out)
