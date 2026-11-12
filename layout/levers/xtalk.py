"""xtalk.py board
Legs of different pairs side by side: every fast segment (LVDS100, RF50, clocks) walked in 0.25 mm
steps against other pairs' segments on its layer. Per pair of nets and layer: length under 0.55 mm
(3 widths), length under 0.25 mm, smallest gap. Coupling per gap: SIGNAL_INTEGRITY 3.5."""
import os, sys, re, math, collections
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from geo import Board
from shapely.geometry import LineString
from shapely.strtree import STRtree
B = Board(sys.argv[1])
hs = lambda n: n and (B.netclass(n) in ("LVDS100", "RF50") or (re.search("CLK", n) and not n.startswith("+")))
base = lambda n: re.sub(r"_[PN]$", "", n)
LAY = ("F.Cu", "In2.Cu", "In5.Cu", "B.Cu") if '"In6.Cu"' in B.t[:3000] else ("F.Cu", "In2.Cu", "In3.Cu", "B.Cu")
segs = [i for i in B.items if i.kind == "seg" and hs(i.net) and i.layer in LAY]
tree = {L: STRtree([i.geom for i in B.by_layer[L]]) for L in LAY}
runs = collections.defaultdict(lambda: [0.0, 9.9, 0.0])   # len<0.55, min gap, len<0.25
for it in segs:
    p0, p1, w = it.data; ln = math.dist(p0, p1)
    if ln < 0.3: continue
    n = max(1, int(ln / 0.25))
    for k in range(n):
        q0 = [p0[i] + (p1[i]-p0[i]) * k / n for i in (0, 1)]; q1 = [p0[i] + (p1[i]-p0[i]) * (k+1) / n for i in (0, 1)]
        pc = LineString([q0, q1]).buffer(w/2, cap_style=2); seen = {}
        for j in tree[it.layer].query(pc.buffer(0.6)):
            o = B.by_layer[it.layer][j]
            if o.kind != "seg" or not hs(o.net) or base(o.net) == base(it.net): continue
            d = o.geom.distance(pc)
            if d < 0.55: seen[o.net] = min(seen.get(o.net, 9), d)
        for on, d in seen.items():
            r = runs[(it.net, on, it.layer)]; r[0] += ln / n; r[1] = min(r[1], d)
            if d < 0.25: r[2] += ln / n
out = sorted(runs.items(), key=lambda kv: -kv[1][2] * 3 - kv[1][0])
for (a, b, L), (l, g, l2) in out:
    if l >= 3 and a < b: print(f"{a:28s} {b:28s} {L:7s} <0.55: {l:5.1f} mm  <0.25: {l2:5.1f} mm  min gap {g:.2f}")
