# Sourced by the routing scripts: where KiCad and KiCadRoutingTools live.
#   CLI           kicad-cli: $KICAD_CLI, else on PATH, else the KiCad 10 install folders
#                 (per user, then per machine), as toolpaths.py
#   KICAD_PYTHON  KiCad's own python, next to kicad-cli (route.py uses it for exact zone fills)
#   KRT           the KiCadRoutingTools checkout (setup_krt.sh makes it); no default
#   PY            the python of its venv, $KRT/.venv
#   py            the host python: the Windows launcher if present, else python (mise)
# Machine-specific values go in the environment, e.g. an untracked mise.local.toml:
#   [env]
#   KRT = "/path/to/KiCadRoutingTools"
for c in "$KICAD_CLI" "$(command -v kicad-cli)" \
         "$LOCALAPPDATA/Programs/KiCad/10.0/bin/kicad-cli.exe" \
         "${PROGRAMFILES:-C:/Program Files}/KiCad/10.0/bin/kicad-cli.exe"; do
    [ -n "$c" ] && [ -f "$c" ] && { CLI="$c"; break; }
done
[ -n "$CLI" ] || { echo "kicad-cli not found: install KiCad 10 or set KICAD_CLI" >&2; exit 1; }
export KICAD_PYTHON=${KICAD_PYTHON:-$(dirname "$CLI")/python.exe}
# KRT is only needed by route_finish.sh, which checks it; without it PY is the host python.
if [ -z "$PY" ]; then
    if [ -n "$KRT" ]; then
        PY="$KRT/.venv/Scripts/python.exe"
        [ -f "$PY" ] || PY="$KRT/.venv/bin/python"
    else
        PY=$(command -v python3 || command -v python)
    fi
fi
command -v py > /dev/null || py() { python "$@"; }
