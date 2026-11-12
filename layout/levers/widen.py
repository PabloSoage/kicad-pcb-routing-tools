"""widen.py in out [--cls LVDS100] [--w F.Cu=0.183,B.Cu=0.183,In2.Cu=0.175,In3.Cu=0.175]
          [--fallback 0.15] [--coupled 0.25]
Loosely coupled differential pairs: a leg that runs apart from its partner is a single-ended
line, so its width sets Z0 and the pair's impedance is about 2 x Z0. The router laid the LVDS
legs at 0.117 mm (59 ohm, 117-119 ohm differential); this widens them to the layer's 50-ohm
width (field_solver.py: 0.183 on L1/L6, 0.175 on L3/L4) where the clearances allow, else to
--fallback, else leaves them. Each segment is judged in 0.5 mm pieces and split, so a leg is
necked only where something is in the way. A segment whose partner leg runs within --coupled mm
(edge to edge) on the same layer is a coupled stretch and is left alone.
Checks: the class clearance to other nets' copper on the layer (pads, tracks, vias, and pieces
widened in this pass), 0.2 to other nets' holes, 0.3 to the board edge. Text edit."""
import os, sys, re, math, uuid, argparse, collections
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from geo import Board
from shapely.geometry import LineString

ap = argparse.ArgumentParser()
ap.add_argument("inp"); ap.add_argument("out")
ap.add_argument("--cls", default="LVDS100")
ap.add_argument("--w", default="F.Cu=0.183,B.Cu=0.183,In2.Cu=0.175,In3.Cu=0.175")
ap.add_argument("--fallback", default="0.15")
ap.add_argument("--coupled", type=float, default=0.25)
ap.add_argument("--net", default=None, help="regex: only these nets of the class (e.g. a supply's trunk)")
a = ap.parse_args()
B = Board(a.inp); t = B.t
W = {k: float(v) for k, v in (s.split("=") for s in a.w.split(","))}
FB = [float(x) for x in a.fallback.split(",") if x]
CL = B.classes[a.cls]["clearance"]
PIECE = 0.5
mine = [it for it in B.items if it.kind == "seg" and B.netclass(it.net) == a.cls and (not a.net or re.search(a.net, it.net))]
partner = lambda n: n[:-2] + ("_N" if n.endswith("_P") else "_P") if n.endswith(("_P", "_N")) else None
legs = collections.defaultdict(list)
for it in mine: legs[(it.net, it.layer)].append(it)
new = collections.defaultdict(list)          # layer -> (net, geom) widened in this pass
res = collections.Counter()
split = {}                                   # uuid -> [(start, end, width)]


def fits(L, net, g):
    for k in B.tree[L].query(g.buffer(0.3)):
        o = B.by_layer[L][k]
        if o.net == net: continue
        if g.distance(o.geom) < max(CL, B.clr(net, o.net)) - 1e-6: return False
    for n2, g2 in new[L]:
        if n2 != net and g.distance(g2) < CL - 1e-6: return False
    return not B.hole_conflicts(g, net) and B.edge_ok(g)


for it in sorted(mine, key=lambda i: -math.dist(i.data[0], i.data[1])):
    (p0, p1, w0), L = it.data, it.layer
    ln = math.dist(p0, p1)
    gap = min([o.geom.distance(it.geom) for o in legs.get((partner(it.net), L), [])] + [9])
    if gap <= a.coupled or L not in W or w0 >= W[L] - 1e-6 or ln < 1e-6:
        res["kept"] += ln; continue
    n = max(1, round(ln / PIECE))
    pts = [(p0[0] + (p1[0] - p0[0]) * k / n, p0[1] + (p1[1] - p0[1]) * k / n) for k in range(n + 1)]
    cands = [W[L]] + [x for x in FB if w0 < x < W[L]]
    ws = [next((w for w in cands if fits(L, it.net, LineString([pts[k], pts[k + 1]]).buffer(w / 2, 8))), w0)
          for k in range(n)]
    runs = []
    for k, w in enumerate(ws):
        if runs and runs[-1][2] == w: runs[-1][1] = k + 1
        else: runs.append([k, k + 1, w])
    out = []
    for s, e, w in runs:          # the run as one line (round ends included) must clear too
        if w != w0 and not fits(L, it.net, LineString([pts[s], pts[e]]).buffer(w / 2, 8)): w = w0
        if out and out[-1][2] == w: out[-1][1] = pts[e]
        else: out.append([pts[s], pts[e], w])
    for s, e, w in out:
        res["narrow" if w == w0 else f"{w:.3f}"] += math.dist(s, e)
        if w != w0: new[L].append((it.net, LineString([s, e]).buffer(w / 2, 8)))
    if any(w != w0 for s, e, w in out): split[it.uuid] = out


def sub(m):
    b = m.group(0)
    u = re.search(r'\(uuid "([^"]+)"\)', b)
    if not u or u.group(1) not in split: return b
    net = re.search(r'\(net "([^"]*)"\)', b).group(1); L = re.search(r'\(layer "([^"]+)"\)', b).group(1)
    return "".join(f'\n\t(segment\n\t\t(start {s[0]:.6f} {s[1]:.6f})\n\t\t(end {e[0]:.6f} {e[1]:.6f})\n\t\t(width {w})\n'
                   f'\t\t(layer "{L}")\n\t\t(net "{net}")\n\t\t(uuid "{uuid.uuid4()}")\n\t)' for s, e, w in split[u.group(1)])


t = re.sub(r"\n\t\(segment\n.*?\n\t\)", sub, t, flags=re.S)
open(a.out, "w", encoding="utf-8", newline="\n").write(t)
tot = sum(res.values())
print("length by outcome (mm):", {k: round(v) for k, v in sorted(res.items())}, "of", round(tot))
