"""audit.py board [--classes RF50,LVDS100] [--out report.txt]
High-speed checks a reviewer runs on a finished board (text parse, no pcbnew):
  A. reference plane: every RF50/LVDS segment on L1, L3, L4, L6 is checked against the GND plane
     it sits next to (L1, L3 -> L2; L4, L6 -> L5), with that plane filled the way KiCad fills it
     (zone outline minus other nets' copper + clearance, min-thickness opening). Reported: the
     length whose strip (trace width + 2 x 0.1 mm) is not wholly over copper, per net, with
     where. (L3 sits 0.13 mm under L2 and 0.93 mm over L4, so L2 is its reference.)
  B. return vias: every via of those nets that changes reference plane gets the distance to the
     nearest GND via; over 1.0 mm is reported.
  C. spacing: other-net copper along RF50/LVDS segments; a parallel run closer than 3 x width
     (edge to edge) for more than 2 mm is reported (crosstalk).
  D. acute angles: two segments of a net meeting at under 90 degrees on a layer (acid traps).
Prints a summary per check and the worst items."""
import os, sys, re, math, argparse, collections
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from geo import Board
from shapely.geometry import Point, Polygon, MultiPolygon, LineString
from shapely.ops import unary_union
from shapely.strtree import STRtree

ap = argparse.ArgumentParser()
ap.add_argument("board"); ap.add_argument("--classes", default="RF50,LVDS100")
ap.add_argument("--extra", default=r"CLK", help="other nets to include by regex")
a = ap.parse_args()
B = Board(a.board); t = B.t
CLS = set(a.classes.split(","))
xre = re.compile(a.extra) if a.extra else None
hs = lambda n: n and (B.netclass(n) in CLS or (xre and xre.search(n) and not n.startswith("+")))
EIGHT = '"In6.Cu"' in t[:3000]           # JLC08161H-2116A (8 layers) or JLC06161H-2116A (6)
REF = ({"F.Cu": "In1.Cu", "In2.Cu": "In1.Cu", "In5.Cu": "In6.Cu", "B.Cu": "In6.Cu"} if EIGHT else
       {"F.Cu": "In1.Cu", "In2.Cu": "In1.Cu", "In3.Cu": "In4.Cu", "B.Cu": "In4.Cu"})


def fill(layer, net="GND"):
    polys = []
    for m in re.finditer(r"\n\t\(zone\n(.*?)\n\t\)", t, re.S):
        b = m.group(1)
        if "(keepout" in b or f'(layer "{layer}")' not in b or f'(net "{net}")' not in b: continue
        cl = float(re.search(r"\(connect_pads[^)]*?\(clearance ([\d.]+)\)", b, re.S).group(1))
        mt = float(re.search(r"\(min_thickness ([\d.]+)\)", b).group(1))
        pts = re.search(r"\(polygon\s*\(pts(.*?)\)\s*\)", b, re.S).group(1)
        poly = Polygon([tuple(map(float, q)) for q in re.findall(r"\(xy ([-\d.]+) ([-\d.]+)", pts)])
        items = B.by_layer[layer]
        cut = [it.geom.buffer(max(cl, B.clr(net, it.net)), 8) for it in items if it.net != net]
        cut += [p.buffer(r + 0.25, 8) for p, r, n in B.holes if n != net]
        f = poly.difference(unary_union(cut)).buffer(-mt / 2, 8).buffer(mt / 2, 8)
        polys.append(f)
    return unary_union(polys)


segs = [it for it in B.items if it.kind == "seg" and hs(it.net)]
print(f"{len(segs)} high-speed segments, {len({s.net for s in segs})} nets")

# ---- A
planes = {L: fill(L) for L in set(REF.values())}
own = collections.defaultdict(list)         # net -> its vias and pads (a void there is the via's own antipad)
for it in B.items:
    if it.kind in ("via", "pad") and it.layer in ("F.Cu",) or it.kind == "pad": own[it.net].append(it.geom)
owntree = {n: STRtree(g) for n, g in own.items()}
runsA = []; totA = collections.Counter()
for it in segs:
    if it.layer not in REF: continue
    (p0, p1, w) = it.data; ln = math.dist(p0, p1)
    if ln < 1e-6: continue
    pl = planes[REF[it.layer]]
    n = max(1, int(ln / 0.1)); cur = 0.0; start = None
    for k in range(n + 1):
        void = False
        if k < n:
            q0 = (p0[0] + (p1[0] - p0[0]) * k / n, p0[1] + (p1[1] - p0[1]) * k / n)
            q1 = (p0[0] + (p1[0] - p0[0]) * (k + 1) / n, p0[1] + (p1[1] - p0[1]) * (k + 1) / n)
            strip = LineString([q0, q1]).buffer(w / 2 + 0.1, cap_style=2)
            totA[it.net] += ln / n
            near_own = it.net in owntree and any(own[it.net][j].distance(strip) < 0.5 for j in owntree[it.net].query(strip.buffer(0.6)))
            void = not pl.contains(strip) and not near_own
        if void:
            if start is None: start = q0
            cur += ln / n
        else:
            if cur > 1.0: runsA.append((cur, it.net, it.layer, start))
            cur = 0.0; start = None
print("\nA. reference plane: continuous runs over a void of the adjacent GND plane longer than 1 mm")
for v, n, L, s in sorted(runsA, reverse=True)[:40]:
    print(f"   {n:28s} {L:7s} {v:4.1f} mm from ({s[0]:.2f}, {s[1]:.2f})")
print(f"   {len(runsA)} runs, {sum(r[0] for r in runsA):.0f} mm in all, of {sum(totA.values()):.0f} mm")

# ---- B
gv = [it.data[:2] for it in B.items if it.kind == "via" and it.layer == "F.Cu" and it.net == "GND"]
gtree = STRtree([Point(p) for p in gv])
print("\nB. layer changes without a GND via within 1.0 mm")
nb = 0; nv = 0; hist = collections.Counter()
for it in B.items:
    if it.kind != "via" or it.layer != "F.Cu" or not hs(it.net): continue
    ls = {s.layer for s in segs if s.net == it.net and s.geom.intersects(it.geom)}
    refs = {REF.get(l) for l in ls}
    if len(refs) < 2: continue
    nv += 1
    p = Point(it.data[:2]); d = p.distance(Point(gv[gtree.nearest(p)]))   # true nearest (was capped at 9 beyond 3 mm)
    lim = 1.0 if B.netclass(it.net) == "RF50" else 3.0       # lambda/20 at 1.2 GHz (300 ps edges) = 6 mm
    hist[min(int(d), 3)] += 1
    if d > lim:
        nb += 1; print(f"   {it.net:28s} via ({it.data[0]:.2f}, {it.data[1]:.2f}) {sorted(ls)}  nearest GND via {d:.2f} mm")
print(f"   {nb} of {nv} reference-changing vias over the limit (RF50 1 mm, others 3 mm); nearest GND via <1/1-2/2-3/>3 mm: {[hist[k] for k in range(4)]}")

# ---- C
print("\nC. parallel runs closer than 3 x width for more than 2 mm")
tree = {L: STRtree([i.geom for i in B.by_layer[L]]) for L in REF}
runs = collections.Counter()
for it in segs:
    if it.layer not in REF: continue
    (p0, p1, w) = it.data; ln = math.dist(p0, p1)
    if ln < 2: continue
    band = LineString([p0, p1]).buffer(w / 2 + 3 * w, cap_style=2)
    for k in tree[it.layer].query(band):
        o = B.by_layer[it.layer][k]
        if o.net == it.net or o.kind != "seg": continue
        if B.netclass(it.net) == "LVDS100" and o.net[:-2] == it.net[:-2]: continue     # its own partner
        if o.net == "GND": continue
        ov = o.geom.intersection(band)
        if ov.is_empty: continue
        L = ov.length / 2 if ov.geom_type.endswith("Polygon") else 0
        L = max(L, 0)
        if L > 2: runs[(it.net, o.net, it.layer)] += L
for (n1, n2, L), v in sorted(runs.items(), key=lambda x: -x[1])[:40]:
    print(f"   {n1:26s} ~ {n2:26s} {L:7s} {v:5.1f} mm")
print(f"   {len(runs)} pairs of nets")

# ---- D
print("\nD. acute angles (< 90 deg) between segments of a net")
ends = collections.defaultdict(list)
for it in B.items:
    if it.kind != "seg": continue
    (p0, p1, w) = it.data
    if math.dist(p0, p1) < 0.05: continue
    for a_, b_ in ((p0, p1), (p1, p0)):
        ends[(it.net, it.layer, round(a_[0], 3), round(a_[1], 3))].append((b_[0] - a_[0], b_[1] - a_[1]))
nd = 0
for (n, L, x, y), vs in ends.items():
    for i in range(len(vs)):
        for j in range(i + 1, len(vs)):
            u, v = vs[i], vs[j]
            c = (u[0] * v[0] + u[1] * v[1]) / (math.hypot(*u) * math.hypot(*v))
            if c > 0.05:      # angle < ~87 deg
                nd += 1
                if nd <= 25: print(f"   {n:28s} {L:7s} ({x}, {y}) {math.degrees(math.acos(min(1, c))):4.0f} deg")
print(f"   {nd} acute junctions")
