"""ripbad.py in out [--drc report.json]
After pairnego: runs KiCad's DRC (zones refilled) and rips every LVDS pair that has a copper
violation (clearance, hole clearance, short, crossing, keep-out, board edge, dangling copper),
keeping its via-in-pad at the FPGA's balls. pairnego_mp.py / pass2 then route the ripped pairs
again with the others fixed. Prints the pairs ripped and why. Text edit."""
import os, sys, re, json, subprocess, argparse, collections
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
ap = argparse.ArgumentParser()
ap.add_argument("inp"); ap.add_argument("out"); ap.add_argument("--drc")
ap.add_argument("--cli", default=os.environ.get("CLI", "kicad-cli"))
ap.add_argument("--pairs", default="", help="PAIR,PAIR: rip these too (no DRC needed with --only)")
ap.add_argument("--only", action="store_true", help="rip only --pairs, run no DRC")
A = ap.parse_args()
PAIR = re.compile(r"/((?:ADCR_(?:D\d|DCO|OR))|(?:ADCS_(?:D\d|DCO|FCO))|(?:LINK_RX_(?:D\d|CLK))|(?:LINK_TX_(?:D\d|CLK)))_[PN]\]")
BAD = {"clearance", "hole_clearance", "shorting_items", "tracks_crossing", "items_not_allowed", "copper_edge_clearance",
       "track_dangling", "via_dangling", "hole_to_hole", "track_width", "diff_pair_gap_out_of_range"}
rep = A.drc or A.out + ".drc.json"
if not A.drc and not A.only:
    subprocess.run([A.cli, "pcb", "drc", "--refill-zones", "--severity-all", "--format", "json", "-o", rep, A.inp],
                   capture_output=True)
d = json.load(open(rep, encoding="utf-8")) if not A.only else {"violations": []}
why = collections.defaultdict(collections.Counter)
for p in filter(None, A.pairs.split(",")): why[p.strip("/")]["asked"] += 1
for v in d["violations"]:
    if v["type"] not in BAD: continue
    for it in v["items"]:
        m = PAIR.search(it["description"])
        if m: why[m.group(1)][v["type"]] += 1
t = open(A.inp, encoding="utf-8").read()
# the balls' via-in-pad stay: vias at a U1101 pad of the same net
from geo import Board
B = Board(A.inp)
balls = {(it.net, round(it.data[2], 3), round(it.data[3], 3)) for it in B.by_layer["F.Cu"] if it.kind == "pad" and it.data[0] == "U1101"}
nets = {f"/{p}_{s}" for p in why for s in "PN"}


def rip(m):
    b = m.group(0)
    n = re.search(r'\(net "([^"]*)"\)', b)
    if not n or n.group(1) not in nets: return b
    if b.startswith("\n\t(via"):
        at = re.search(r"\(at ([-\d.]+) ([-\d.]+)\)", b)
        if (n.group(1), round(float(at.group(1)), 3), round(float(at.group(2)), 3)) in balls: return b
    return ""


t = re.sub(r"\n\t\((segment|via|arc)\n.*?\n\t\)", rip, t, flags=re.S)
open(A.out, "w", encoding="utf-8", newline="\n").write(t)
for p, c in sorted(why.items()): print(f"ripped /{p}: {dict(c)}")
print("pairs ripped:", len(why))
