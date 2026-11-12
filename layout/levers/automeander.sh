#!/bin/bash
# automeander.sh in out NET NEED_MM [REGEX]: meander NET's longest F/B segments (meander.py, amplitude
# 1.5, 0.8, 0.5 mm) until NEED mm are added. REGEX selects the segments when NET has regex
# characters in its name (Net-(U401-Q0)). It may also lengthen a stub: check with lvds_skew.py.
# AM_LAYERS="F.Cu B.Cu In2.Cu In5.Cu": the layers whose segments may be meandered (default F.Cu B.Cu).
# PRO: the board's .kicad_pro (default: next to IN). LEVERS_OUT: scratch folder (default ./_levers).
H="$(cd "$(dirname "$0")" && (pwd -W 2>/dev/null || pwd))"   # pwd -W: a path Windows python opens (Git Bash)
export MSYS_NO_PATHCONV=1; . "$H/../toolpaths.sh"
in=$1; out=$2; net=$3; need=$4; M="$H/../meander.py"; mkdir -p "${LEVERS_OUT:-$PWD/_levers}"
cur=${LEVERS_OUT:-$PWD/_levers}/am_cur.kicad_pcb; cp "$in" $cur; PRO="${PRO:-${in%.kicad_pcb}.kicad_pro}"; cp "$PRO" ${LEVERS_OUT:-$PWD/_levers}/am_cur.kicad_pro; cp "$PRO" ${LEVERS_OUT:-$PWD/_levers}/am_try.kicad_pro
$PY "$H/../netdump.py" "$in" "${5:-^${net}$}" | awk -v ly=" ${AM_LAYERS:-F.Cu B.Cu} " '$1=="SEG" && $8>1.8 && index(ly, " "$3" ") {print $8,$4,$5,$6,$7,$3}' | sort -rn > ${LEVERS_OUT:-$PWD/_levers}/am_segs.txt
while read len x1 y1 x2 y2 L; do
  awk -v n=$need 'BEGIN{exit !(n>0.05)}' || break
  for amp in 1.5 0.8 0.5; do
    r=$($PY $M $cur ${LEVERS_OUT:-$PWD/_levers}/am_try.kicad_pcb "$net" $x1 $y1 $x2 $y2 $L $need --amp $amp --margin 0.4 2>&1 | grep "^+")
    if [ -n "$r" ]; then got=$(echo "$r" | sed 's/^+\([0-9.]*\).*/\1/'); mv ${LEVERS_OUT:-$PWD/_levers}/am_try.kicad_pcb $cur; need=$(awk -v n=$need -v g=$got 'BEGIN{print n-g}'); echo "  $L $x1,$y1-$x2,$y2 amp $amp +$got (left $need)"; break; fi
  done
done < ${LEVERS_OUT:-$PWD/_levers}/am_segs.txt
cp $cur "$out"; echo "left $need"
