"""nudge_vias.py in.kicad_pcb drc.json out.kicad_pcb [margin]
For each via/track clearance error in the DRC json, move the via (and the ends of its own
net's tracks that sit on its centre) away from the other net's nearby tracks until the
required clearance (+margin) holds against every other-net segment within 1 mm."""
import json, re, sys, math
src, dj, out = sys.argv[1:4]
margin = float(sys.argv[4]) if len(sys.argv) > 4 else 0.003
t = open(src, encoding="utf-8").read()
d = json.load(open(dj, encoding="utf-8"))
SEG = re.compile(r"\n\t\(segment\n.*?\n\t\)", re.S)
segs = []
for m in SEG.finditer(t):
    s = m.group(0)
    a = tuple(map(float, re.search(r"\(start ([-\d.]+) ([-\d.]+)", s).groups()))
    b = tuple(map(float, re.search(r"\(end ([-\d.]+) ([-\d.]+)", s).groups()))
    w = float(re.search(r"\(width ([\d.]+)", s).group(1))
    L = re.search(r'\(layer "([^"]+)"', s).group(1)
    n = re.search(r'\(net "([^"]*)"\)', s).group(1)
    segs.append((a, b, w, L, n))


def dseg(p, a, b):
    ax, ay = a; bx, by = b; px, py = p
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    u = 0 if L2 == 0 else max(0, min(1, ((px - ax) * dx + (py - ay) * dy) / L2))
    cx, cy = ax + u * dx, ay + u * dy
    return math.hypot(px - cx, py - cy), (px - cx, py - cy)


moves = {}
for v in d["violations"]:
    if v["severity"] != "error" or v["type"] != "clearance": continue
    vi = [i for i in v["items"] if i["description"].startswith(("Vía", "Vía", "Via"))]
    if len(vi) != 1: continue
    m = re.search(r"(?:margen|clearance) ([\d.,]+) mm", v["description"])   # Spanish or English DRC text
    if not m: continue
    req = float(m.group(1).replace(",", "."))
    p = (vi[0]["pos"]["x"], vi[0]["pos"]["y"])
    moves.setdefault(p, req)
    moves[p] = max(moves[p], req)
VIA = re.compile(r"\n\t\(via\n.*?\n\t\)", re.S)
for (x, y), req in moves.items():
    def at(s, k="at"):
        q = re.search(rf"\({k} ([-\d.]+) ([-\d.]+)\)", s)
        return q and abs(float(q.group(1)) - x) < 1e-4 and abs(float(q.group(2)) - y) < 1e-4 and q.group(0)
    m = next(m for m in VIA.finditer(t) if at(m.group(0)))
    vs = m.group(0)
    size = float(re.search(r"\(size ([\d.]+)", vs).group(1))
    net = re.search(r'\(net "([^"]*)"\)', vs).group(1)
    px, py = x, y
    for _ in range(200):
        worst = None
        for a, b, w, L, n in segs:
            if n == net: continue
            dist, vec = dseg((px, py), a, b)
            need = req + size / 2 + w / 2 + margin
            if dist < need and (worst is None or need - dist > worst[0]): worst = (need - dist, vec, dist)
        if not worst: break
        gap, (vx, vy), dist = worst
        k = (gap + 0.0005) / dist
        px, py = px + vx * k, py + vy * k
    px, py = round(px, 4), round(py, 4)
    print(f"via {net} {x},{y} -> {px},{py}")
    t = t.replace(vs, vs.replace(at(vs), f"(at {px:.4f} {py:.4f})"))
    def fix(mm):
        s = mm.group(0)
        if f'(net "{net}")' not in s: return s
        for k in ("start", "end"):
            g = at(s, k)
            if g: s = s.replace(g, f"({k} {px:.4f} {py:.4f})")
        return s
    t = SEG.sub(fix, t)
open(out, "w", encoding="utf-8", newline="\n").write(t)
