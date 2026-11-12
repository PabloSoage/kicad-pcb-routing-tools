#!/bin/bash
# skewfix.sh in out [LIMIT_PS] : for every LVDS pair whose |P-N| (lvds_skew.py, package included) is over
# LIMIT_PS (default 5), the earlier leg is meandered by the
# difference (automeander.sh on F.Cu, B.Cu and the inner signal layers, a coupled pair on its
# fan-ins only; 6.8 ps/mm in the striplines,
# 5.9 on the outer layers: 6.5 is used, then a second pass trims what is left). Prints before/after.
H="$(cd "$(dirname "$0")" && (pwd -W 2>/dev/null || pwd))"   # pwd -W: a path Windows python opens (Git Bash)
in="$1"; out="$2"; LIM=${3:-5}
. "$H/../toolpaths.sh"
EIGHT=$(head -c 3000 "$in" | grep -c '"In6.Cu"')
[ "$EIGHT" -gt 0 ] && export AM_LAYERS="F.Cu B.Cu In2.Cu In5.Cu" || export AM_LAYERS="F.Cu B.Cu In2.Cu In3.Cu"
cur="${out%.kicad_pcb}.sf.kicad_pcb"; cp "$in" "$cur"
cp "${in%.kicad_pcb}.kicad_pro" "${cur%.kicad_pcb}.kicad_pro" 2>/dev/null; cp "${in%.kicad_pcb}.kicad_dru" "${cur%.kicad_pcb}.kicad_dru" 2>/dev/null
for pass in 1 2; do
  $PY "$H/../lvds_skew.py" "$cur" | awk -v lim=$LIM '$1 ~ /^(ADC|LINK|CLK_FPGA)/ && $4 ~ /^[+-][0-9]+$/ { s=$4+0; c=$(NF-1); sub("%","",c); if (s>lim || s<-lim) print $1, s, c+0 }' > "$cur.todo"
  [ -s "$cur.todo" ] || break
  while read pair s c; do
    net="/${pair}_N"; [ "$s" -lt 0 ] && net="/${pair}_P"
    [ "$pair" = "CLK_FPGA" ] && { net="/CONTROL/FPGA_CLK_N"; [ "$s" -lt 0 ] && net="/CONTROL/FPGA_CLK_P"; }
    mm=$(awk -v s=$s 'BEGIN{v=(s<0?-s:s)/6.5; printf "%.2f", v}')
    echo "pass $pass: $pair P-N $s ps: $net +$mm mm"
    L="$AM_LAYERS"; [ "$c" -ge 50 ] && L="F.Cu B.Cu"     # a coupled pair: only its single-ended fan-ins
    AM_LAYERS="$L" LEVERS_OUT="$(dirname "$cur")/_am" bash "$H/automeander.sh" "$cur" "$cur" "$net" "$mm" 2>&1 | tail -1
  done < "$cur.todo"
done
$PY "$H/../lvds_skew.py" "$cur" | awk '$1 ~ /^(ADC|LINK|CLK)/ && $4 ~ /^[+-][0-9]+$/ {print $1, $4, $(NF-1)}' | column -t
mv "$cur" "$out"
