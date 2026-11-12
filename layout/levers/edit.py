"""edit.py in out CMD... : small text edits of a board, in order. Commands (one per argument):
  place:REF:x:y:rot:F|B        move a footprint; a side change flips it (two-terminal parts only)
  seg:NET:LAYER:w:x,y:x,y...   add a track (a polyline)
  via:NET:x:y                  add a through via 0.45/0.2, filled and capped
  delseg:NET:x,y               delete NET's tracks with an end within 0.02 mm of (x, y)
  delvia:NET:x,y               delete NET's via at (x, y)
  rip:NET                      delete every track and via of NET
Text edit, no pcbnew."""
import re, sys, uuid, math

inp, out, *cmds = sys.argv[1:]
t = open(inp, encoding="utf-8").read()
new = []


def f(v): return f"{v:.4f}".rstrip("0").rstrip(".")


def fp_span(ref):
    i = t.find(f'(property "Reference" "{ref}"')
    assert i >= 0, ref
    s = t.rfind("\n\t(footprint", 0, i); e = t.find("\n\t)", i) + 3
    return s, e


def place(ref, x, y, rot, side):
    global t
    s, e = fp_span(ref)
    fp = t[s:e]
    m = re.search(r"\n\t\t\(at ([-\d.]+) ([-\d.]+)(?: ([-\d.]+))?\)", fp)
    r0 = float(m.group(3) or 0); d = rot - r0
    cur = re.search(r'\n\t\t\(layer "([FB])\.Cu"\)', fp).group(1)
    head = fp[:m.start()] + f"\n\t\t(at {f(x)} {f(y)}" + (f" {f(rot % 360)}" if rot % 360 else "") + ")"
    rest = fp[m.end():]
    if d:
        def rr(mm):
            ang = (float(mm.group(4) or 0) + d) % 360
            return f"{mm.group(1)}(at {mm.group(2)} {mm.group(3)}" + (f" {f(ang)}" if ang else "") + ")"
        rest = re.sub(r"(\n\t\t\t)\(at ([-\d.]+) ([-\d.]+)(?: ([-\d.]+))?\)", rr, rest)
    fp = head + rest
    if cur != side:
        for py in re.findall(r'\(pad "[^"]*" \w+ \w+\s*\(at [-\d.]+ ([-\d.]+)', fp):
            assert abs(float(py)) < 1e-6, f"{ref}: pad off the x axis, flip not supported"
        a, b = ("F.", "B.") if side == "B" else ("B.", "F.")
        fp = re.sub(r'"' + re.escape(a) + r'(Cu|Mask|Paste|SilkS|CrtYd|Fab|Adhes)"', lambda mm: f'"{b}{mm.group(1)}"', fp)
        if side == "B":
            fp = re.sub(r"(\(effects\s*\(font(?:(?!\(effects).)*?\)\s*\))(\s*\))", r"\1\n\t\t\t\t(justify mirror)\2", fp, flags=re.S)
        else:
            fp = re.sub(r"\s*\(justify mirror\)", "", fp)
    t = t[:s] + fp + t[e:]


def delete(pred):
    global t
    n = [0]
    def k(mm):
        if pred(mm.group(1), mm.group(0)):
            n[0] += 1; return ""
        return mm.group(0)
    t = re.sub(r"\n\t\((segment|via|arc)\n.*?\n\t\)", k, t, flags=re.S)
    return n[0]


def netof(b):
    m = re.search(r'\(net "([^"]*)"\)', b); return m.group(1) if m else ""


for c in cmds:
    p = c.split(":")
    if p[0] == "place":
        place(p[1], float(p[2]), float(p[3]), float(p[4]), p[5])
    elif p[0] == "seg":
        net, layer, w = p[1], p[2], float(p[3])
        pts = [tuple(map(float, q.split(","))) for q in p[4:]]
        for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
            new.append(f'\t(segment\n\t\t(start {f(x1)} {f(y1)})\n\t\t(end {f(x2)} {f(y2)})\n\t\t(width {f(w)})\n'
                       f'\t\t(layer "{layer}")\n\t\t(net "{net}")\n\t\t(uuid "{uuid.uuid4()}")\n\t)')
    elif p[0] == "via":
        new.append(f'\t(via\n\t\t(at {f(float(p[2]))} {f(float(p[3]))})\n\t\t(size 0.45)\n\t\t(drill 0.2)\n'
                   f'\t\t(layers "F.Cu" "B.Cu")\n\t\t(capping yes)\n\t\t(filling yes)\n\t\t(net "{p[1]}")\n\t\t(uuid "{uuid.uuid4()}")\n\t)')
    elif p[0] in ("delseg", "delvia"):
        net = p[1]; x, y = map(float, p[2].split(","))
        def pred(kind, b, net=net, x=x, y=y, want=p[0]):
            if netof(b) != net: return False
            if want == "delvia":
                if kind != "via": return False
                q = tuple(map(float, re.search(r"\(at ([-\d.]+) ([-\d.]+)\)", b).groups()))
                return math.dist(q, (x, y)) < 0.02
            if kind == "via": return False
            for tag in ("start", "end"):
                q = tuple(map(float, re.search(r"\(" + tag + r" ([-\d.]+) ([-\d.]+)\)", b).groups()))
                if math.dist(q, (x, y)) < 0.02: return True
            return False
        print(c, "->", delete(pred), "deleted")
    elif p[0] == "rip":
        print(c, "->", delete(lambda kind, b, net=p[1]: netof(b) == net), "deleted")
    else:
        sys.exit(f"unknown command {c}")

if new:
    end = t.rstrip().rfind(")")
    t = t[:end].rstrip("\n") + "\n" + "\n".join(new) + "\n)\n"
open(out, "w", encoding="utf-8", newline="\n").write(t)
