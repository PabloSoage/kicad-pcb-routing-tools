"""swap_polarity.py in out PAIR U BALL_A BALL_B [TERM_REF]
Swap a differential pair's legs at one chip (an FPGA polarity inversion): the nets of pads
BALL_A and BALL_B of footprint U are exchanged, as are the nets of the vias sitting in those pads
and of the tracks wholly inside them, and the termination TERM_REF (a two-pad part straddling the
two balls) is turned 180 degrees so that each of its pads stays on its own net. The schematic and
the pin table change with it (by hand). Text edit; DRC with parity after."""
import sys, re, math

inp, out, pair, U, ba, bb = sys.argv[1:7]
term = sys.argv[7] if len(sys.argv) > 7 else None
t = open(inp, encoding="utf-8").read()
P, N = f"/{pair}_P", f"/{pair}_N"
swap = {P: N, N: P}


def footprint(ref):
    i = t.index(f'(property "Reference" "{ref}"')
    s = t.rfind("\n\t(footprint", 0, i)
    e = t.index("\n\t(footprint", i) if "\n\t(footprint" in t[i:] else t.index("\n\t(", i)
    return s, e


def pad_xy(fp, num):
    at = re.search(r"\n\t\t\(at ([-\d.]+) ([-\d.]+)(?: ([-\d.]+))?\)", fp)
    X, Y, A = float(at.group(1)), float(at.group(2)), math.radians(float(at.group(3) or 0))
    m = re.search(r'\(pad "%s" \S+ \S+\s*\(at ([-\d.]+) ([-\d.]+)' % num, fp)
    px, py = float(m.group(1)), float(m.group(2))
    return X + px * math.cos(A) + py * math.sin(A), Y - px * math.sin(A) + py * math.cos(A)


s, e = footprint(U)
fp = t[s:e]
pts = [pad_xy(fp, ba), pad_xy(fp, bb)]
for num in (ba, bb):
    m = re.search(r'(\(pad "%s" .*?\(net ")([^"]*)("\))' % num, fp, re.S)
    assert m.group(2) in swap, f"pad {num} is on {m.group(2)}, not {pair}"
    fp = fp[:m.start(2)] + swap[m.group(2)] + fp[m.end(2):]
t = t[:s] + fp + t[e:]
print(f"{U}.{ba}/{bb}: nets swapped")


def near(x, y):
    return any(math.dist((x, y), p) < 0.3 for p in pts)


def fix_item(m):
    b = m.group(0)
    net = re.search(r'\(net "([^"]*)"\)', b)
    if not net or net.group(1) not in swap:
        return b
    if b.startswith("\n\t(via"):
        x, y = map(float, re.search(r"\(at ([-\d.]+) ([-\d.]+)\)", b).groups())
        ok = near(x, y)
    else:
        a = tuple(map(float, re.search(r"\(start ([-\d.]+) ([-\d.]+)\)", b).groups()))
        z = tuple(map(float, re.search(r"\(end ([-\d.]+) ([-\d.]+)\)", b).groups()))
        ok = near(*a) and near(*z)
    if not ok:
        return b
    print("  ", b.split("\n")[1].strip()[:40], "->", swap[net.group(1)])
    return b.replace(f'(net "{net.group(1)}")', f'(net "{swap[net.group(1)]}")')


t = re.sub(r"\n\t\((?:via|segment)\n.*?\n\t\)", fix_item, t, flags=re.S)

if term:
    s, e = footprint(term)
    fp = t[s:e]
    at = re.search(r"(\n\t\t\(at ([-\d.]+) ([-\d.]+))(?: ([-\d.]+))?\)", fp)
    rot = (float(at.group(4) or 0) + 180) % 360
    fp = fp[:at.start()] + f"{at.group(1)} {rot:g})" + fp[at.end():]
    # pads, fields and graphics carry the footprint angle in their own (at x y angle)
    fp = re.sub(r"(\n\t\t\t\(at [-\d.]+ [-\d.]+)(?: ([-\d.]+))?\)",
                lambda m: f"{m.group(1)} {(float(m.group(2) or 0) + 180) % 360:g})", fp)
    t = t[:s] + fp + t[e:]
    print(f"{term}: turned to {rot:g} deg")

open(out, "w", encoding="utf-8", newline="").write(t)
