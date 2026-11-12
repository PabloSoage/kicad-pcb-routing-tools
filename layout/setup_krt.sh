#!/bin/bash
# Install KiCadRoutingTools the way the routing scripts expect it (26 Oct 2026):
#   1. clone github.com/drandyhaas/KiCadRoutingTools into <dir> at the commit the board was
#      routed with (bce10dd7, version 0.22.1);
#   2. apply krt_finalize_layers.patch (no supply tracks on the GND planes, docs/LAYOUT.md);
#   3. a venv at <dir>/.venv with its requirements.txt, from the host python (mise);
#   4. build the Rust grid router from source (cargo, e.g. rustup from winget): the
#      prebuilt download is a newer version than this commit's Cargo.toml.
# Then set KRT=<dir> in the environment (mise.local.toml, see toolpaths.sh).
#
# Usage:  bash layout/setup_krt.sh <dir>
set -e
KRT="$1"
COMMIT=bce10dd7
HERE="$(cd "$(dirname "$0")" && pwd)"
[ -d "$KRT/.git" ] || git clone https://github.com/drandyhaas/KiCadRoutingTools.git "$KRT"
cd "$KRT"
git checkout -q "$COMMIT"
git checkout -q -- py_router/route.py
tr -d '\r' < "$HERE/krt_finalize_layers.patch" | git apply     # the repo checks it out CRLF
python -m venv .venv
PY=.venv/Scripts/python.exe; [ -f "$PY" ] || PY=.venv/bin/python
"$PY" -m pip install -q -r requirements.txt
"$PY" build_router.py --from-source
echo "done: set KRT=$KRT"
