"""Board copper as shapely geometry, per layer and net (text parse, no pcbnew)."""
import re, math, json, fnmatch, os
from shapely.geometry import Point, LineString, Polygon, box
from shapely import affinity
from shapely.strtree import STRtree

CU = ["F.Cu", "In1.Cu", "In2.Cu", "In3.Cu", "In4.Cu", "In5.Cu", "In6.Cu", "B.Cu"]


def expand_layers(spec):
    out = []
    for l in re.findall(r'"([^"]+)"', spec):
        if l == "*.Cu": out += CU
        elif l == "F&B.Cu": out += ["F.Cu", "B.Cu"]
        elif l in CU: out.append(l)
    return out


class Item:
    __slots__ = ("kind", "net", "layer", "geom", "uuid", "data")

    def __init__(s, kind, net, layer, geom, uuid=None, data=None):
        s.kind, s.net, s.layer, s.geom, s.uuid, s.data = kind, net, layer, geom, uuid, data


class Board:
    def __init__(s, path, pro=None, ignore=None):
        """ignore: a regex; tracks and vias of matching nets are left out (they may be ripped)."""
        s.path = path
        s.t = t = open(path, encoding="utf-8").read()
        pro = pro or os.path.splitext(path)[0] + ".kicad_pro"
        p = json.load(open(pro, encoding="utf-8"))
        s.classes = {c["name"]: c for c in p["net_settings"]["classes"]}
        s.patterns = p["net_settings"].get("netclass_patterns") or []
        s.items = []       # copper items
        s.holes = []       # (Point, radius, net)
        s.via_holes = set()
        s.keepouts = []    # (layer, Polygon, flags)
        s.zones = []       # (net, layer, Polygon)
        for m in re.finditer(r"\n\t\((segment|via|arc)\n(.*?)\n\t\)", t, re.S):
            k, b = m.groups()
            net = re.search(r'\(net "([^"]*)"\)', b); net = net.group(1) if net else ""
            uid = re.search(r'\(uuid "([^"]+)"\)', b).group(1)
            if k == "via":
                x, y = map(float, re.search(r"\(at ([-\d.]+) ([-\d.]+)\)", b).groups())
                sz = float(re.search(r"\(size ([\d.]+)", b).group(1))
                dr = float(re.search(r"\(drill ([\d.]+)", b).group(1))
                for l in CU:   # through vias only on this board
                    s.items.append(Item("via", net, l, Point(x, y).buffer(sz / 2, 16), uid, (x, y, sz)))
                s.holes.append((Point(x, y), dr / 2, net)); s.via_holes.add(len(s.holes) - 1)
            else:
                a = tuple(map(float, re.search(r"\(start ([-\d.]+) ([-\d.]+)\)", b).groups()))
                e = tuple(map(float, re.search(r"\(end ([-\d.]+) ([-\d.]+)\)", b).groups()))
                pts = [a, e]
                if k == "arc":
                    mid = tuple(map(float, re.search(r"\(mid ([-\d.]+) ([-\d.]+)\)", b).groups()))
                    pts = [a, mid, e]
                w = float(re.search(r"\(width ([\d.]+)\)", b).group(1))
                L = re.search(r'\(layer "([^"]+)"\)', b).group(1)
                s.items.append(Item("seg", net, L, LineString(pts).buffer(w / 2, 8), uid, (a, e, w)))
        for fp in re.finditer(r"\n\t\(footprint .*?\n\t\)", t, re.S):
            fp = fp.group(0)
            ref = re.search(r'\(property "Reference" "([^"]+)"', fp).group(1)
            at = re.search(r"\n\t\t\(at ([-\d.]+) ([-\d.]+)(?: ([-\d.]+))?\)", fp)
            X, Y, A = float(at.group(1)), float(at.group(2)), float(at.group(3) or 0)
            ra = math.radians(A)
            for pm in re.finditer(r"\n\t\t\(pad \"([^\"]*)\" (\w+) (\w+)(.*?)\n\t\t\)", fp, re.S):
                num, typ, shape, b = pm.groups()
                net = re.search(r'\(net "([^"]*)"\)', b); net = net.group(1) if net else ""
                pa = re.search(r"\(at ([-\d.]+) ([-\d.]+)(?: ([-\d.]+))?\)", b)
                px, py, pang = float(pa.group(1)), float(pa.group(2)), float(pa.group(3) or 0)
                gx, gy = X + px * math.cos(ra) + py * math.sin(ra), Y - px * math.sin(ra) + py * math.cos(ra)
                sw, sh = map(float, re.search(r"\(size ([\d.]+) ([\d.]+)\)", b).groups())
                if shape == "circle":
                    g = Point(gx, gy).buffer(sw / 2, 16)
                else:
                    g = affinity.rotate(box(gx - sw / 2, gy - sh / 2, gx + sw / 2, gy + sh / 2), -pang, origin=(gx, gy))
                    if shape == "oval":
                        r = min(sw, sh) / 2
                        g = g.buffer(-r).buffer(r, 8) if not g.buffer(-r).is_empty else g
                dm = re.search(r"\(drill (?:oval )?([\d.]+)", b)
                if dm:
                    s.holes.append((Point(gx, gy), float(dm.group(1)) / 2, net))
                ls = re.search(r"\(layers ([^)]*)\)", b)
                for l in expand_layers(ls.group(1) if ls else ""):
                    if typ == "np_thru_hole": continue
                    s.items.append(Item("pad", net, l, g, f"{ref}.{num}", (ref, num, gx, gy)))
        for z in re.finditer(r"\n\t\(zone\n(.*?)\n\t\)", t, re.S):
            b = z.group(1)
            net = re.search(r'\(net "([^"]*)"\)', b); net = net.group(1) if net else ""
            ls = expand_layers(re.search(r"\(layers? ([^)]*)\)", b).group(1))
            pts = re.search(r"\(polygon\s*\(pts(.*?)\)\s*\)", b, re.S).group(1)
            # no closing paren: the search above ends inside the last point's "(xy x y)"
            poly = Polygon([tuple(map(float, q)) for q in re.findall(r"\(xy ([-\d.]+) ([-\d.]+)", pts)])
            if "(keepout" in b:
                flags = dict(re.findall(r"\((tracks|vias|pads|copperpour|footprints) (\w+)\)", b))
                for l in ls: s.keepouts.append((l, poly, flags))
            else:
                for l in ls: s.zones.append((net, l, poly))
        edge = []
        for m in re.finditer(r"\n\t\(gr_(line|arc|rect) .*?\n\t\)", t, re.S):
            g = m.group(0)
            if '"Edge.Cuts"' not in g: continue
            a = tuple(map(float, re.search(r"\(start ([-\d.]+) ([-\d.]+)\)", g).groups()))
            e = tuple(map(float, re.search(r"\(end ([-\d.]+) ([-\d.]+)\)", g).groups()))
            if m.group(1) == "rect":
                edge.append(LineString([a, (e[0], a[1]), e, (a[0], e[1]), a]))
            else:
                edge.append(LineString([a, e]))
        for m in re.finditer(r"\n\t\(gr_poly\n.*?\n\t\)", t, re.S):   # the outline is one polygon
            g = m.group(0)
            if '"Edge.Cuts"' not in g: continue
            pts = [tuple(map(float, q)) for q in re.findall(r"\(xy ([-\d.]+) ([-\d.]+)", g)]
            if len(pts) > 1: edge.append(LineString(pts + pts[:1]))
        s.edge = edge
        if '"In6.Cu"' not in t[:3000]:                # a 6-layer board: "*.Cu" pads stop at In4
            s.items = [i for i in s.items if i.layer not in ("In5.Cu", "In6.Cu")]
        if ignore:
            s.items = [i for i in s.items if i.kind == "pad" or not re.search(ignore, i.net)]
            s.holes = [h for k, h in enumerate(s.holes) if k not in s.via_holes or not re.search(ignore, h[2])]
        s.by_layer = {l: [i for i in s.items if i.layer == l] for l in CU}
        s.tree = {l: STRtree([i.geom for i in s.by_layer[l]]) for l in CU}
        s.htree = STRtree([h[0] for h in s.holes])

    def netclass(s, net):
        for p in s.patterns:
            if fnmatch.fnmatchcase(net, p["pattern"]): return p["netclass"]
        return "Default"

    def clr(s, a, b):
        return max(s.classes[s.netclass(a)]["clearance"], s.classes[s.netclass(b)]["clearance"])

    def conflicts(s, layer, geom, net, extra=0.0):
        """Other-net copper on layer closer than the clearance (zones excluded)."""
        out = []
        for i in s.tree[layer].query(geom.buffer(0.25 + extra)):
            it = s.by_layer[layer][i]
            if it.net == net and net: continue
            if geom.distance(it.geom) < s.clr(net, it.net) + extra: out.append(it)
        return out

    def hole_conflicts(s, geom, net, hole=None, extra=0.0):
        """Copper too close to other holes (0.2), and a new hole too close to any hole (0.25)."""
        out = []
        for i in s.htree.query(geom.buffer(0.5)):
            p, r, n = s.holes[i]
            if n != net and geom.distance(p) - r < 0.2 + extra: out.append(p)
            if hole is not None and hole[0].distance(p) - r - hole[1] < 0.25 + extra: out.append(p)
        return out

    def keepout_hit(s, layer, geom, what):
        return any(l == layer and f.get(what) == "not_allowed" and poly.intersects(geom)
                   for l, poly, f in s.keepouts)

    def edge_ok(s, geom):
        return all(geom.distance(e) >= 0.3 for e in s.edge)
