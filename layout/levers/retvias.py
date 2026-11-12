"""retvias.py in out [--within 1.0] [--search 2.0] [--nets REGEX]
Return vias for the LVDS pairs' layer changes: for every via of a pair net that is not a ball's via
in pad, if no GND via is within --within mm, one GND through via (0.45/0.2) is added at the
nearest spot within --search mm that clears every other net's copper by its class clearance on
the signal layers and other holes by 0.25 mm. A pair's two vias share one return via when it is
within reach of both. The GND pours (In1, In4, In6 on 8 layers; In1, In4 on 6) pick it up when the
zones are refilled. Prints what it added and what found no spot. Text edit."""
import os, sys, re, math, argparse
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from geo import Board, CU
from shapely.geometry import Point
import pairlib as PL

ap = argparse.ArgumentParser()
ap.add_argument("inp"); ap.add_argument("out")
ap.add_argument("--within", type=float, default=1.0); ap.add_argument("--search", type=float, default=2.0)
ap.add_argument("--nets", default=r"^/(ADCR_(D\d|DCO|OR)|ADCS_(D\d|DCO|FCO)|LINK_RX_(D\d|CLK)|LINK_TX_(D\d|CLK)|CONTROL/FPGA_CLK)_[PN]$")
A = ap.parse_args()
B = Board(A.inp)
NETS = re.compile(A.nets)
PLANES = {"In1.Cu", "In4.Cu", "In6.Cu"} if '"In6.Cu"' in B.t[:3000] else {"In1.Cu", "In4.Cu"}
balls = {(round(it.data[2], 3), round(it.data[3], 3)) for it in B.by_layer["F.Cu"] if it.kind == "pad" and it.data[0] == "U1101"}
vias = {}
for it in B.items:
    if it.kind == "via" and it.layer == "F.Cu":
        vias.setdefault(it.net, []).append((round(it.data[0], 3), round(it.data[1], 3)))
gnd = [Point(p) for p in vias.get("GND", [])]
todo = [(n, p) for n, ps in vias.items() if NETS.match(n) for p in ps if p not in balls]
added = []


def ok(x, y):
    v = Point(x, y).buffer(0.225, 16)
    for l in CU:
        if l in PLANES: continue
        for k in B.tree[l].query(v.buffer(0.4)):
            it = B.by_layer[l][k]
            if it.net == "GND": continue
            if it.geom.distance(v) < max(B.clr("GND", it.net), 0.15) + 0.005: return False
    for i in B.htree.query(Point(x, y).buffer(1.0)):
        p, r, n = B.holes[i]
        if p.distance(Point(x, y)) < r + 0.1 + 0.25: return False
    for q in added:
        if math.dist(q, (x, y)) < 0.45 + 0.25: return False
    return any(Point(x, y).within(e.convex_hull) for e in B.edge) and all(Point(x, y).distance(e) > 0.5 for e in B.edge)


miss = []
st = 0.05; n = int(A.search / st)
OFFS = sorted((math.hypot(i * st, j * st), i * st, j * st) for i in range(-n, n + 1) for j in range(-n, n + 1) if math.hypot(i * st, j * st) <= A.search)
for net, (x, y) in sorted(todo):
    near = min([Point(x, y).distance(g) for g in gnd] + [math.dist((x, y), q) for q in added] + [99])
    if near <= A.within: continue
    best = None
    for d, dx, dy in OFFS:
        if ok(round(x + dx, 3), round(y + dy, 3)): best = (d, round(x + dx, 3), round(y + dy, 3)); break
    if best:
        added.append((best[1], best[2])); print(f"{net} via ({x}, {y}): GND via at ({best[1]}, {best[2]}), {best[0]:.2f} mm (nearest was {near:.2f})")
    else:
        miss.append((net, x, y, near)); print(f"{net} via ({x}, {y}): no spot within {A.search} mm (nearest GND via {near:.2f} mm)")
t = PL.append(B.t, [PL.via_text("GND", x, y) for x, y in added])
open(A.out, "w", encoding="utf-8", newline="\n").write(t)
print(f"return vias added: {len(added)}; layer changes with no spot: {len(miss)}")
