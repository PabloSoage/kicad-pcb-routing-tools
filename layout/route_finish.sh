#!/bin/bash
# Finish an autorouted board without breaking what already works (3-5 Oct 2026).
#
# A plain re-run of route.py with rip-up allowed made things worse: it ripped
# good nets (the link pair, an IF line) to route others, and its improvement
# gate still shipped the result because the count went down. What works:
#   * every net that is already COMPLETE is written into the project as
#     protected (kicad_routing_tools.protected_nets), so nothing may rip it;
#   * only the incomplete nets are routed, on a 0.05 mm grid, with a large
#     search budget. The router can then only use free space.
# One pass took 100 -> 89 unrouted, none worse. Run it again for more.
#
# CLR, TW: Default-class clearance and track width (0.15, class width); PW: supply width (0.3).
# RIP="<glob> ..." lets the router rip and re-route those complete nets when
# they block (give PROTECT a regex that leaves them out, or they stay protected).
#
# Usage:  [LAYERS="F.Cu In2.Cu In3.Cu B.Cu" COSTS="1 1.5 3 2"] [GRID=0.025] [ONLY=<regex>] [PROTECT=<regex>] [RIP=<globs>]
#             bash layout/route_finish.sh <work dir>
#   <work dir>/in.kicad_pcb + in.kicad_pro  ->  <work dir>/out.kicad_pcb
# Then:   python layout/drop_dangling.py <work dir>/out.kicad_pcb --max-rounds 40
set -e
export MSYS_NO_PATHCONV=1          # keep "/NET" names from becoming Windows paths
K="$1"
. "$(dirname "$0")/toolpaths.sh"                 # CLI, KRT, PY (and py)
[ -n "$KRT" ] || { echo "KRT not set: run layout/setup_krt.sh <dir>, then set KRT" >&2; exit 1; }

"$CLI" pcb drc --refill-zones --format json -o "$K/before.json" "$K/in.kicad_pcb" > /dev/null
py - "$K" <<'EOF'
import json, os, re, sys
K = sys.argv[1]
d = json.load(open(f"{K}/before.json", encoding="utf-8"))
todo = sorted({re.search(r"\[([^\]]+)\]", v["items"][0]["description"]).group(1)
               for v in d["unconnected_items"]} - {""})
todo = [n for n in todo if not n.startswith("unconnected-")]
if os.environ.get("ONLY"):                 # a regex: route just these of the incomplete nets
    todo = [n for n in todo if re.search(os.environ["ONLY"], n)]
board = open(f"{K}/in.kicad_pcb", encoding="utf-8").read()
nets = set(re.findall(r'\(net "([^"]+)"\)', board))
pro = json.load(open(f"{K}/in.kicad_pro", encoding="utf-8"))
keep = os.environ.get("PROTECT")         # a regex: protect only these, the rest may be ripped
pro.setdefault("kicad_routing_tools", {})["protected_nets"] = {
    n: "complete" for n in sorted(nets - set(todo))
    if not n.startswith("unconnected-") and (not keep or re.search(keep, n))}
json.dump(pro, open(f"{K}/in.kicad_pro", "w", encoding="utf-8", newline="\n"), indent=2)
open(f"{K}/nets.txt", "w").write("\n".join(todo))
print(f"{len(todo)} incomplete nets to route; the rest protected")
EOF

mapfile -t NETS < "$K/nets.txt"
if [ -z "${NETS[*]}" ]; then cp "$K/in.kicad_pcb" "$K/out.kicad_pcb"; exit 0; fi   # nothing to route
cd "$KRT"
PYTHONIOENCODING=utf-8 $PY py_router/route.py "$K/in.kicad_pcb" "$K/out.kicad_pcb" --nets "${NETS[@]}" \
    --grid-step ${GRID:-0.05} --max-iterations 3000000 --fab-tier standard --escalation off \
    --via-size 0.45 --via-drill 0.2 --clearance ${CLR:-0.15} ${TW:+--track-width $TW} --board-edge-clearance 0.3 \
    --layers ${LAYERS:-F.Cu In2.Cu B.Cu} --layer-costs ${COSTS:-1 1.5 2} \
    --power-nets '+*' '-3V3' --power-nets-widths ${PW:-0.3} ${PW:-0.3} ${RIP:+--rip-existing-nets $RIP} > "$K/out.log" 2>&1
grep -E 'Single-ended:|Multi-point:|IMPROVEMENT GATE' "$K/out.log" | tail -3
