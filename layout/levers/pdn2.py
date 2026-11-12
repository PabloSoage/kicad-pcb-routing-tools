"""pdn2.py board U RAIL[:target_ohm] ... [--fmax 2e8] [--out DIR] [--zone NET:x0,y0,x1,y1,...]
PDN impedance seen by a BGA's balls, from the board's copper, with the In3 (L4) pours filled
the way KiCad fills them (successor of pdn.py, whose end-point graph broke on tracks that meet
mid-segment and whose pours lost a vertex):

  * In3 fill: each zone's outline, minus higher-priority fills (+ clearance), minus foreign
    copper on In3 (+ clearance), opened by min_thickness/2; islands with none of their own
    net's vias are dropped (KiCad's island removal). In1/In4 (GND) are ideal.
  * connectivity by overlap, as KiCad sees it: two items of the net touch -> their anchor
    points (track ends, via centre, pad centre) are joined by the straight distance;
  * cost = 0.4 nH/mm of track + via barrel 0.4 nH per 1.42 mm of height between the layers
    used + nothing across a fill (a plane);
  * each cap with one pad on RAIL and one on GND: path RAIL pad -> the balls of U on RAIL:
    through an L4 fill, the fill-to-ball paths of all the balls in parallel (the die draws on
    every ball at once; --nearest: the one nearest ball, the older, pessimistic figure), path GND pad -> nearest GND via, plus that via down to L2 (top) or L5 (bottom);
  * branch = ESR + j w (ESL + paths) + 1/(j w C x derating), all in parallel with a
    VRM of 5 mOhm + 20 nH.
--zone adds a trial In3 zone (priority above all) before filling: plan an island first.
Prints worst |Z| and the band over the target, each cap's path; writes pdn_RAIL.csv."""
import os, sys, re, math, heapq, argparse, collections
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from geo import Board
from shapely.geometry import Point, Polygon, MultiPolygon
from shapely.ops import unary_union
from shapely.strtree import STRtree

ap = argparse.ArgumentParser()
ap.add_argument("board"); ap.add_argument("u"); ap.add_argument("rails", nargs="+")
ap.add_argument("--fmax", type=float, default=2e8); ap.add_argument("--out", default=".")
ap.add_argument("--zone", action="append", default=[])
ap.add_argument("-v", action="store_true")
ap.add_argument("--nearest", action="store_true", help="old model: the path to the nearest ball only")
a = ap.parse_args()
B = Board(a.board); t = B.t
LMM = 0.4e-9
# depth of each copper layer (mm). JLC08161H-2116C (8 layers): L4 PWR is In3, GND planes In1, In4, In6;
# JLC06161H-2116A (6 layers): PWR In3, GND In1, In4
EIGHT = '"In6.Cu"' in t[:3000]
Z = ({"F.Cu": 0.0, "In1.Cu": 0.119, "In2.Cu": 0.249, "In3.Cu": 0.615, "In4.Cu": 0.745, "In5.Cu": 1.111, "In6.Cu": 1.241, "B.Cu": 1.360}
     if EIGHT else {"F.Cu": 0.0, "In1.Cu": 0.116, "In2.Cu": 0.246, "In3.Cu": 1.176, "In4.Cu": 1.306, "B.Cu": 1.422})
GLAST = "In6.Cu" if EIGHT else "In4.Cu"
LVIA_MM = 0.4e-9 / 1.422

# ---- In3 fill
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
for k, spec in enumerate(a.zone):
    n, _, c = spec.partition(":"); v = list(map(float, c.split(",")))
    zones.append((1000 + k, n, Polygon(list(zip(v[::2], v[1::2]))), 0.3, 0.2))
zones.sort(key=lambda z: -z[0])
in3 = B.by_layer["In3.Cu"]
in3tree = B.tree["In3.Cu"]
fills = []          # (net, polygon component)
taken = []          # (fill geometry, clearance) of higher-priority zones
own_vias = collections.defaultdict(list)
for it in in3:
    if it.kind == "via": own_vias[it.net].append(it.geom)
for pr, n, poly, cl, mt in zones:
    cut = [it.geom.buffer(max(cl, B.clr(n, it.net)), 8) for i in in3tree.query(poly) for it in [in3[i]] if it.net != n]
    cut += [g.buffer(cl, 8) for g, c, tn in taken if tn != n]     # same-net zones merge
    for p, r, hn in B.holes:              # NPTH / foreign holes
        if hn != n and poly.distance(p) < 1: cut.append(p.buffer(r + 0.25, 8))
    f = poly.difference(unary_union(cut)) if cut else poly
    f = f.buffer(-mt / 2, 8).buffer(mt / 2, 8)
    comps = list(f.geoms) if isinstance(f, MultiPolygon) else ([f] if not f.is_empty else [])
    keep = [c for c in comps if any(c.intersects(v) for v in own_vias[n])]
    for c in keep: fills.append((n, c))
    if keep: taken.append((unary_union(keep), cl, n))
merged = []         # overlapping same-net fills are one piece of copper
for n in {n for n, c in fills}:
    u = unary_union([c for m, c in fills if m == n])
    merged += [(n, c) for c in (u.geoms if isinstance(u, MultiPolygon) else [u])]
fills = merged
if a.v:
    for n, c in fills: print(f"fill {n:10s} area {c.area:8.1f} mm2  bounds {tuple(round(x, 1) for x in c.bounds)}")


# ---- graph
def anchors(it):
    if it.kind == "seg": return [it.data[0], it.data[1]]
    if it.kind == "via": return [it.data[:2]]
    return [(it.data[2], it.data[3])]


def build(net):
    its = [it for it in B.items if it.net == net]
    G = collections.defaultdict(list)
    def add(u, v, c):
        G[u].append((v, c)); G[v].append((u, c))
    node = {}
    for k, it in enumerate(its):
        if it.kind == "via":
            node[k] = [("v", it.data[0], it.data[1], it.layer)]
        else:
            node[k] = [("p", it.layer, round(x, 4), round(y, 4)) for x, y in anchors(it)]
            if it.kind == "seg": add(node[k][0], node[k][1], math.dist(*anchors(it)) * LMM)
    # via barrels
    vx = collections.defaultdict(list)
    for k, it in enumerate(its):
        if it.kind == "via": vx[it.data[:2]].append(it.layer)
    for (x, y), ls in vx.items():
        ls = sorted(ls, key=Z.get)
        for l1, l2 in zip(ls, ls[1:]): add(("v", x, y, l1), ("v", x, y, l2), (Z[l2] - Z[l1]) * LVIA_MM)
    # overlaps per layer
    for L in Z:
        idx = [k for k, it in enumerate(its) if it.layer == L]
        if not idx: continue
        tr = STRtree([its[k].geom for k in idx])
        for k in idx:
            for j in tr.query(its[k].geom):
                j = idx[j]
                if j <= k or not its[k].geom.intersects(its[j].geom): continue
                for u in node[k]:
                    for v in node[j]:
                        pu = u[1:3] if u[0] == "v" else u[2:4]; pv = v[1:3] if v[0] == "v" else v[2:4]
                        add(u, v, math.dist(pu, pv) * LMM)
    # In3 fills of this net
    for c, (n, poly) in enumerate(fills):
        if n != net: continue
        for k, it in enumerate(its):
            if it.layer == "In3.Cu" and it.geom.intersects(poly):
                for u in node[k]: add(u, ("fill", c), 0.0)
    pads = collections.defaultdict(list)
    for k, it in enumerate(its):
        if it.kind == "pad": pads[(it.data[0], it.data[1])] += node[k]
    return G, pads


def dijkstra(G, srcs, targets):
    best = {s: 0.0 for s in srcs}; pq = [(0.0, s) for s in srcs]; heapq.heapify(pq)
    while pq:
        d, u = heapq.heappop(pq)
        if u in targets: return d, u
        if best.get(u, 1e9) < d: continue
        for v, c in G[u]:
            if d + c < best.get(v, 1e9):
                best[v] = d + c; heapq.heappush(pq, (d + c, v))
    return None, None


def dijkstra_all(G, srcs):
    best = {s: 0.0 for s in srcs}; pq = [(0.0, s) for s in srcs]; heapq.heapify(pq)
    while pq:
        d, u = heapq.heappop(pq)
        if best.get(u, 1e9) < d: continue
        for v, c in G[u]:
            if d + c < best.get(v, 1e9):
                best[v] = d + c; heapq.heappush(pq, (d + c, v))
    return best


def cap_model(lib, val):
    v = val.lower().replace("µ", "u")
    m = re.match(r"([\d.]+)\s*([pnu])", v); C = float(m[1]) * {"p": 1e-12, "n": 1e-9, "u": 1e-6}[m[2]]
    if "7343" in lib: return C * 0.9, 25e-3, 2.0e-9
    if "1210" in lib: return C * 0.5, 3e-3, 0.9e-9
    if "1206" in lib: return C * 0.5, 4e-3, 0.8e-9
    if "0805" in lib: return C * 0.55, 6e-3, 0.6e-9
    if "0603" in lib: return C * 0.8, 10e-3, 0.5e-9
    return C * 0.8, 20e-3, 0.4e-9


fpinfo = {}
for m in re.finditer(r"\n\t\(footprint \"([^\"]+)\"\n(.*?)\n\t\)", t, re.S):
    lib, b = m.groups()
    ref = re.search(r'"Reference" "([^"]+)"', b).group(1)
    fpinfo[ref] = (lib, re.search(r'"Value" "([^"]*)"', b).group(1), re.search(r'\t\t\(layer "([FB])', b).group(1))
GG, gpads = build("GND")
gtargets = set()
for (x, y, l) in [(n[1], n[2], n[3]) for n in GG if n[0] == "v"]:
    pass
gvia_end = {}      # GND via node on F/B -> cost to the plane on that side
for n in list(GG):
    if n[0] == "v" and n[3] in ("F.Cu", "B.Cu"):
        gtargets.add(n); gvia_end[n] = (Z["In1.Cu"] - Z["F.Cu"] if n[3] == "F.Cu" else Z["B.Cu"] - Z[GLAST]) * LVIA_MM
freqs = [10 ** (5 + k * (math.log10(a.fmax) - 5) / 200) for k in range(201)]
for spec in a.rails:
    rail, _, tz = spec.partition(":"); tz = float(tz) if tz else None
    G, pads = build(rail)
    balls = {n for (ref, num), ns in pads.items() if ref == a.u for n in ns}
    caps = collections.defaultdict(dict)
    for (ref, num), ns in pads.items():
        if ref.startswith("C"): caps[ref]["r"] = ns
    for (ref, num), ns in gpads.items():
        if ref in caps: caps[ref]["g"] = ns
    # the die draws through all its balls at once: from an L4 fill, the balls' paths are in
    # parallel (1 / sum 1/L_i over the balls); a cap reaching a fill gets that, not the one ball
    ballsets = [ns for (ref, num), ns in pads.items() if ref == a.u]
    lpar = {}
    for c, (n, poly) in enumerate(fills):
        if n != rail: continue
        dd = dijkstra_all(G, [("fill", c)])
        ls = [min(dd.get(x, 1e9) for x in ns) for ns in ballsets]
        ls = [l for l in ls if l < 1e8]
        if ls: lpar[c] = 1 / sum(1 / max(l, 1e-13) for l in ls)
    br = []; unreached = []
    for ref, d in caps.items():
        if "g" not in d: continue
        lr, _ = dijkstra(G, d["r"], balls)
        if lr is not None and lpar and not a.nearest:
            dc = dijkstra_all(G, d["r"])
            lr = min([lr] + [dc[("fill", c)] + lp for c, lp in lpar.items() if ("fill", c) in dc])
        if lr is None: unreached.append(ref); continue
        lg, gn = dijkstra(GG, d["g"], gtargets)
        lg = (lg + gvia_end[gn]) if lg is not None else 0.0
        lib, val, side = fpinfo[ref]
        C, R, esl = cap_model(lib, val)
        br.append((ref, val, C, R, esl + lr + lg, lr, lg))
    rows = []
    for f in freqs:
        w = 2 * math.pi * f
        y = 1 / complex(5e-3, w * 20e-9)
        for ref, val, C, R, L, *_ in br: y += 1 / complex(R, w * L - 1 / (w * C))
        rows.append((f, abs(1 / y)))
    zmax = max(rows, key=lambda r: r[1]); over = [f for f, z in rows if tz and z > tz]
    zband = max(z for f, z in rows if f <= 10e6)      # UG483's band: 100 kHz to 10 MHz
    print(f"\n{rail}: {len(br)} capacitors, |Z| max {zband*1e3:.0f} mOhm in 0.1-10 MHz, {zmax[1]*1e3:.0f} mOhm at {zmax[0]/1e6:.1f} MHz"
          + (f"; target {tz*1e3:.0f} mOhm: " + (f"exceeded {min(over)/1e6:.2f}-{max(over)/1e6:.1f} MHz" if over else "met everywhere") if tz else "")
          + (f"; not reaching the balls: {' '.join(sorted(unreached))}" if unreached else ""))
    for ref, val, C, R, L, lr, lg in sorted(br, key=lambda b: b[4]):
        print(f"   {ref:6s} {val:8s} loop {L*1e9:5.2f} nH (to balls {lr*1e9:.2f}, to GND plane {lg*1e9:.2f})")
    with open(os.path.join(a.out, f"pdn_{rail.strip('+')}.csv"), "w") as fh:
        fh.write("f_Hz,Z_ohm\n" + "".join(f"{f:.4g},{z:.5g}\n" for f, z in rows))
