"""irdrop.py board U RAIL:amps[:source_ref] ... [--outer-um 35] [--inner-um 17.5]
DC drop from a rail's source (its buck inductor or LDO, or source_ref) to the nearest and the
farthest ball of U on that rail, along the least-resistance copper path (tracks: rho L / (w t);
vias: 0.2 mm drill, 25 um plating, 1.42 mm; an In3 (L4) fill: its sheet resistance x the
straight distance across it / an assumed 2 mm-wide current path). Parallel paths are ignored, so
the figure is an upper bound. The full rail current is assumed through the path.
The In3 fill is modelled as in pdn2.py (outline minus foreign copper and clearance)."""
import os, sys, re, math, heapq, argparse, collections
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from geo import Board
from shapely.geometry import Point, Polygon, MultiPolygon
from shapely.ops import unary_union
from shapely.strtree import STRtree

ap = argparse.ArgumentParser()
ap.add_argument("board"); ap.add_argument("u"); ap.add_argument("rails", nargs="+")
ap.add_argument("--outer-um", type=float, default=35); ap.add_argument("--inner-um", type=float, default=17.5)
a = ap.parse_args()
B = Board(a.board); t = B.t
RHO = 1.72e-8
TH = {"F.Cu": a.outer_um, "B.Cu": a.outer_um}
R_VIA = RHO * 1.42e-3 / (math.pi * ((0.1e-3 + 25e-6) ** 2 - 0.1e-3 ** 2))
RS_IN = RHO / (a.inner_um * 1e-6)

# In3 fill (as pdn2.py)
zones = []
for m in re.finditer(r"\n\t\(zone\n(.*?)\n\t\)", t, re.S):
    b = m.group(1)
    if "(keepout" in b or '(layer "In3.Cu")' not in b: continue
    n = re.search(r'\(net "([^"]*)"\)', b).group(1)
    pr = re.search(r"\(priority (\d+)\)", b); pr = int(pr.group(1)) if pr else 0
    cl = float(re.search(r"\(connect_pads[^)]*?\(clearance ([\d.]+)\)", b, re.S).group(1))
    mt = float(re.search(r"\(min_thickness ([\d.]+)\)", b).group(1))
    pts = re.search(r"\(polygon\s*\(pts(.*?)\)\s*\)", b, re.S).group(1)
    zones.append((pr, n, Polygon([tuple(map(float, q)) for q in re.findall(r"\(xy ([-\d.]+) ([-\d.]+)", pts)]), cl, mt))
zones.sort(key=lambda z: -z[0])
in3 = B.by_layer["In3.Cu"]; in3tree = B.tree["In3.Cu"]
own_vias = collections.defaultdict(list)
for it in in3:
    if it.kind == "via": own_vias[it.net].append(it.geom)
fills = []; taken = []
for pr, n, poly, cl, mt in zones:
    cut = [it.geom.buffer(max(cl, B.clr(n, it.net)), 8) for i in in3tree.query(poly) for it in [in3[i]] if it.net != n]
    cut += [g.buffer(cl, 8) for g, tn in taken if tn != n]
    f = (poly.difference(unary_union(cut)) if cut else poly).buffer(-mt / 2, 8).buffer(mt / 2, 8)
    comps = list(f.geoms) if isinstance(f, MultiPolygon) else ([f] if not f.is_empty else [])
    keep = [c for c in comps if any(c.intersects(v) for v in own_vias[n])]
    fills += [(n, c) for c in keep]
    if keep: taken.append((unary_union(keep), n))
merged = []
for n in {n for n, c in fills}:
    u = unary_union([c for m, c in fills if m == n])
    merged += [(n, c) for c in (u.geoms if isinstance(u, MultiPolygon) else [u])]
fills = merged


def anchors(it):
    if it.kind == "seg": return [it.data[0], it.data[1]]
    if it.kind == "via": return [it.data[:2]]
    return [(it.data[2], it.data[3])]


def build(net):
    its = [it for it in B.items if it.net == net]
    G = collections.defaultdict(list)
    def add(u, v, r): G[u].append((v, r)); G[v].append((u, r))
    node = {}
    for k, it in enumerate(its):
        node[k] = [("v", it.data[0], it.data[1])] if it.kind == "via" else [("p", it.layer, round(x, 4), round(y, 4)) for x, y in anchors(it)]
        if it.kind == "seg":
            (p0, p1, w) = it.data
            add(node[k][0], node[k][1], RHO * math.dist(p0, p1) * 1e-3 / (w * 1e-3 * TH.get(it.layer, a.inner_um) * 1e-6))
    for L in ("F.Cu", "In1.Cu", "In2.Cu", "In3.Cu", "In4.Cu", "In5.Cu", "In6.Cu", "B.Cu"):
        idx = [k for k, it in enumerate(its) if it.layer == L]
        if not idx: continue
        tr = STRtree([its[k].geom for k in idx])
        for k in idx:
            for j in tr.query(its[k].geom):
                j = idx[j]
                if j <= k or not its[k].geom.intersects(its[j].geom): continue
                via_hop = its[k].kind == "via" or its[j].kind == "via"
                ws = [x.data[2] for x in (its[k], its[j]) if x.kind == "seg"] or [0.3]
                for u in node[k]:
                    for v in node[j]:
                        pu = u[1:3] if u[0] == "v" else u[2:4]; pv = v[1:3] if v[0] == "v" else v[2:4]
                        r = RHO * math.dist(pu, pv) * 1e-3 / (min(ws) * 1e-3 * TH.get(L, a.inner_um) * 1e-6)
                        add(u, v, r + (R_VIA / 2 if via_hop else 0))
    for c, (n, poly) in enumerate(fills):
        if n != net: continue
        for k, it in enumerate(its):
            if it.layer == "In3.Cu" and it.kind == "via" and it.geom.intersects(poly):
                add(node[k][0], ("fill", c, it.data[0], it.data[1]), 0.0)
        # across the fill: between every pair of its vias, sheet R x distance / 2 mm
        vs = [it.data[:2] for it in its if it.layer == "In3.Cu" and it.kind == "via" and it.geom.intersects(poly)]
        for i in range(len(vs)):
            for j in range(i + 1, len(vs)):
                d = math.dist(vs[i], vs[j])
                if d < 25 or len(vs) < 400: add(("fill", c) + tuple(vs[i]), ("fill", c) + tuple(vs[j]), RS_IN * d / 2.0)
    pads = collections.defaultdict(list)
    for k, it in enumerate(its):
        if it.kind == "pad": pads[(it.data[0], it.data[1])] += node[k]
    return G, pads


def dij(G, srcs):
    best = {s: 0.0 for s in srcs}; pq = [(0.0, s) for s in srcs]; heapq.heapify(pq)
    while pq:
        d, u = heapq.heappop(pq)
        if best.get(u, 1e9) < d: continue
        for v, c in G[u]:
            if d + c < best.get(v, 1e9): best[v] = d + c; heapq.heappush(pq, (d + c, v))
    return best


for spec in a.rails:
    rail, amps, *src = spec.split(":"); amps = float(amps)
    G, pads = build(rail)
    if src: sp = [n for (ref, num), ns in pads.items() if ref == src[0] for n in ns]; sname = src[0]
    else:
        cand = [(ref, num) for (ref, num) in pads if ref.startswith("L") or ref.startswith("FB")]
        sname = cand[0][0] if cand else "?"; sp = [n for k in cand[:1] for n in pads[k]]
    best = dij(G, sp)
    balls = {(ref, num): min(best.get(n, 1e9) for n in ns) for (ref, num), ns in pads.items() if ref == a.u}
    rs = sorted(balls.values())
    unreached = [k[1] for k, v in balls.items() if v >= 1e9]
    rs = [r for r in rs if r < 1e9]
    print(f"{rail} from {sname}, {amps*1e3:.0f} mA: path R to the balls {rs[0]*1e3:.1f}-{rs[-1]*1e3:.1f} mOhm,"
          f" drop {amps*rs[0]*1e3:.1f}-{amps*rs[-1]*1e3:.1f} mV" + (f"; unreached {unreached}" if unreached else ""))
