"""freespot.py board REF x0 y0 x1 y1 [--ignore REGEX] [--step 0.25]
Positions in a box where the 0402 REF fits on F.Cu: pads clear of other copper by 0.12,
courtyard clear of other F-side courtyards, and a via spot within 1.2 mm of each pad that
clears copper on every layer. Prints the best ones (most room first)."""
import os, sys, re, math, argparse
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from geo import Board, CU
from shapely.geometry import Point, box
from shapely.ops import unary_union

ap = argparse.ArgumentParser()
ap.add_argument("board"); ap.add_argument("ref"); ap.add_argument("box", nargs=4, type=float)
ap.add_argument("--ignore", default=None, help="nets whose tracks/vias are not obstacles (ripped)")
ap.add_argument("--side", default="F")
ap.add_argument("--step", type=float, default=0.25)
a = ap.parse_args()
B = Board(a.board, ignore=a.ignore)
t = B.t
# F-side courtyards of the other footprints
crt = []
for fp in re.finditer(r"\n\t\(footprint .*?\n\t\)", t, re.S):
    f = fp.group(0)
    ref = re.search(r'\(property "Reference" "([^"]+)"', f).group(1)
    if ref == a.ref or f'\n\t\t(layer "{a.side}.Cu")' not in f:
        continue
    at = re.search(r"\n\t\t\(at ([-\d.]+) ([-\d.]+)(?: ([-\d.]+))?\)", f)
    X, Y, A = float(at.group(1)), float(at.group(2)), math.radians(float(at.group(3) or 0))
    pts = []
    # one element at a time: the lazy match must not run on from a silk line into the next element
    for g in re.finditer(r"\(fp_(?:line|rect)\s*\(start ([-\d.]+) ([-\d.]+)\)\s*\(end ([-\d.]+) ([-\d.]+)\)"
                         r"(?:(?!\(fp_).)*?\(layer \"" + a.side + r"\.CrtYd\"\)", f, re.S):
        for px, py in ((float(g.group(1)), float(g.group(2))), (float(g.group(3)), float(g.group(4)))):
            pts.append((X + px * math.cos(A) + py * math.sin(A), Y - px * math.sin(A) + py * math.cos(A)))
    if pts:
        xs, ys = zip(*pts); crt.append((ref, box(min(xs), min(ys), max(xs), max(ys))))
from shapely.geometry import Polygon
eg = re.search(r'\(gr_poly\s*\(pts(.*?)\)\s*\)(?:(?!\(gr_).)*?"Edge.Cuts"', t, re.S)
EDGE = Polygon([tuple(map(float, q)) for q in re.findall(r"\(xy ([-\d.]+) ([-\d.]+)", eg.group(1))])
IN_CY, IN_VIA = EDGE.buffer(-0.3), EDGE.buffer(-0.5)
crt_u = unary_union([c for _, c in crt])
mine = [i for i in B.items if i.kind == "pad" and i.uuid.startswith(a.ref + ".")]
nets = {i.uuid.split(".")[1]: i.net for i in mine}
holes = unary_union([p.buffer(r) for p, r, n in B.holes])


def via_ok(x, y, net):
    v = Point(x, y).buffer(0.225)
    if not IN_VIA.contains(v): return False
    for l in CU:
        if l in ("In1.Cu", "In4.Cu"):   # GND planes: the fill clears round a via
            continue
        for k in B.tree[l].query(v.buffer(0.2)):
            it = B.by_layer[l][k]
            if it.net != net and not it.uuid.startswith(a.ref + ".") and it.geom.distance(v) < 0.12: return False
    return holes.distance(Point(x, y)) > 0.25 + 0.1


res = []
x = a.box[0]
while x <= a.box[2]:
    y = a.box[1]
    while y <= a.box[3]:
        for rot in (0, 90, 180, 270):
            r = math.radians(rot)
            ok = True; room = 9.0
            cy = box(-0.93, -0.47, 0.93, 0.47)
            from shapely import affinity
            cy = affinity.translate(affinity.rotate(cy, -rot, origin=(0, 0)), x, y)
            if cy.intersects(crt_u) or not IN_CY.contains(cy): ok = False
            vias = {}
            for num, px in (("1", -0.48), ("2", 0.48)):
                gx, gy = x + px * math.cos(r), y - px * math.sin(r)
                pad = affinity.rotate(box(gx - 0.28, gy - 0.3, gx + 0.28, gy + 0.3), -rot, origin=(gx, gy))
                near = [B.by_layer[a.side + ".Cu"][k] for k in B.tree[a.side + ".Cu"].query(pad.buffer(1))]
                dd = min([i.geom.distance(pad) for i in near if i.net != nets[num] and not i.uuid.startswith(a.ref + ".")] + [1])
                if dd < 0.12: ok = False
                room = min(room, dd)
                if not ok: break
                best = None
                for k in range(16):
                    for rr in (0.5, 0.8, 1.2):
                        vx, vy = gx + rr * math.cos(k * math.pi / 8), gy + rr * math.sin(k * math.pi / 8)
                        if via_ok(vx, vy, nets[num]):
                            best = (round(vx, 2), round(vy, 2)); break
                    if best: break
                if not best: ok = False; break
                vias[num] = best
            if ok:
                res.append((round(room, 2), round(x, 2), round(y, 2), rot, vias))
        y += a.step
    x += a.step
res.sort(key=lambda r: -r[0])
for r in res[:int(os.environ.get("FS_N","60"))]:
    print(*r)
print(len(res), "spots")
