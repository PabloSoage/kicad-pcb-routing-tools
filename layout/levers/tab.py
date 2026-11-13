"""tab.py in out X0 Y0 X1 Y1: grow the board by a rectangle on its east edge (x = X0): the
Edge.Cuts polygon and every zone whose outline runs along that edge (the GND planes) get the same
four corners. Text edit. (for a connector's tab, e.g. an RJ45)"""
import re, sys
inp, out, x0, y0, x1, y1 = sys.argv[1], sys.argv[2], *map(float, sys.argv[3:7])
t = open(inp, encoding="utf-8").read()
def grow(pts_txt):
    pts = [(float(a), float(b)) for a, b in re.findall(r"\(xy ([-\d.]+) ([-\d.]+)\)", pts_txt)]
    for i in range(len(pts)):
        a, b = pts[i], pts[(i + 1) % len(pts)]
        if abs(a[0] - x0) < 1e-6 and abs(b[0] - x0) < 1e-6 and min(a[1], b[1]) <= y0 and max(a[1], b[1]) >= y1:
            down = b[1] > a[1]
            ins = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)] if down else [(x0, y1), (x1, y1), (x1, y0), (x0, y0)]
            pts = pts[:i + 1] + ins + pts[i + 1:]
            return pts, True
    return pts, False
n = 0
def fix_edge(m):
    global n
    b = m.group(0)
    if '"Edge.Cuts"' not in b: return b
    pm = re.search(r"\(pts\s*((?:\(xy [^)]*\)\s*)+)\)", b, re.S)
    pts, ok = grow(pm.group(1))
    if not ok: return b
    n += 1
    return b[:pm.start(1)] + "\n\t\t\t" + " ".join(f"(xy {x:g} {y:g})" for x, y in pts) + "\n\t\t" + b[pm.end(1):]
t = re.sub(r"\n\t\(gr_poly\b.*?\n\t\)", fix_edge, t, flags=re.S)
def fix_zone(m):
    global n
    z = m.group(0)
    pm = re.search(r"\(polygon\s*\(pts\s*((?:\(xy [^)]*\)\s*)+)\)", z, re.S)
    if not pm: return z
    pts, ok = grow(pm.group(1))
    if not ok: return z
    n += 1
    z = z[:pm.start(1)] + "\n\t\t\t\t" + " ".join(f"(xy {x:g} {y:g})" for x, y in pts) + "\n\t\t\t" + z[pm.end(1):]
    return re.sub(r"\n\t\t\(filled_polygon.*?\n\t\t\)", "", z, flags=re.S)   # refill later
t = re.sub(r"\n\t\(zone\b.*?\n\t\)", fix_zone, t, flags=re.S)
open(out, "w", encoding="utf-8", newline="\n").write(t)
print("grown:", n, "outlines")
