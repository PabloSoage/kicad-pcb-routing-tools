#!/bin/bash
# pns.sh SCRIPT...: run pns_drive on each script, in the script's directory.
# Address randomisation off: the router's results depend on pointer order. SEED=n pads the
# environment by n bytes, which moves the heap, to try other orders reproducibly.
# PNS_DRIVE: the binary (default ~/kicad-pns/build/qa/tools/pns/pns_drive).
# PNS_SBX:   an Apptainer image/sandbox to run it in (built from kicad-build.def); unset = run directly.
BIN=${PNS_DRIVE:-$HOME/kicad-pns/build/qa/tools/pns/pns_drive}
for f in "$@"; do
  echo "== $f"
  ( cd "$(dirname "$f")"
    CFG=$(mktemp -d)              # a fresh KiCad config per run: nothing leaks between runs
    PAD=$(head -c "${SEED:-0}" /dev/zero | tr '\0' x)
    if [ -n "$PNS_SBX" ]; then
      KICAD_CONFIG_HOME=$CFG PNS_PAD="$PAD" apptainer exec --bind /mnt "$PNS_SBX" \
        setarch "$(uname -m)" -R "$BIN" "$(basename "$f")"
    else
      KICAD_CONFIG_HOME=$CFG PNS_PAD="$PAD" setarch "$(uname -m)" -R "$BIN" "$(basename "$f")"
    fi 2>&1 | grep -v -E "Gtk|GLib|schema file|^$"
    rm -rf "$CFG" )
done
