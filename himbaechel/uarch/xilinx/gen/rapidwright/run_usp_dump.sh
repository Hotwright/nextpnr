#!/bin/bash
# Dump an UltraScale+ part from RapidWright, then build its himbaechel chipdb.
#
#   gen/rapidwright/run_usp_dump.sh <part> <fabric> <outdir>
#   e.g. gen/rapidwright/run_usp_dump.sh xck26-sfvc784-2LV-c xck26 /tmp/usp-db
#
# RAPIDWRIGHT_CLASSPATH  a RapidWright classpath (e.g. the classpath.txt that
#                        0-ultrascale-bits/nextpnr-xilinx/build-jar holds)
# RAPIDWRIGHT_PATH       the RapidWright checkout: node delays come from its
#                        timing/ultrascaleplus delay terms
# BBASM                  bbasm binary (default: build-xilinx/bba/bbasm)
#
# Writes <outdir>/chipdb-<fabric>.bba and .bin. The dump itself is ~120 MB and
# takes seconds; the generator ~1 minute and under 1 GB for the K26.
set -euo pipefail
PART="${1:?usage: run_usp_dump.sh <part> <fabric> <outdir>}"
FABRIC="${2:?}"
OUT="${3:?}"
HERE="$(cd -P "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CP="${RAPIDWRIGHT_CLASSPATH:?set RAPIDWRIGHT_CLASSPATH}"
: "${RAPIDWRIGHT_PATH:?set RAPIDWRIGHT_PATH to the RapidWright checkout}"
[ -f "$RAPIDWRIGHT_PATH/timing/ultrascaleplus/intersite_delay_terms.txt" ] ||
    { echo "no timing/ultrascaleplus delay terms under RAPIDWRIGHT_PATH=$RAPIDWRIGHT_PATH" >&2; exit 1; }
export RAPIDWRIGHT_PATH
BBASM="${BBASM:-$HERE/../../../../../build-xilinx/bba/bbasm}"

CLASSES="$(mktemp -d)"
trap 'rm -rf "$CLASSES"' EXIT
javac -cp "$CP" -d "$CLASSES" "$HERE/UspDump.java"
java -Xmx24g -cp "$CLASSES:$CP" dev.fpga.rapidwright.UspDump "$PART" "$FABRIC" "$OUT"
mkdir -p "$OUT/timing"
cp "$RAPIDWRIGHT_PATH"/timing/ultrascaleplus/*_delay_terms.txt "$OUT/timing/"
python3 "$HERE/../xilinx_gen.py" --usp-db "$OUT" --device "$FABRIC" --bba "$OUT/chipdb-$FABRIC.bba"
"$BBASM" --l "$OUT/chipdb-$FABRIC.bba" "$OUT/chipdb-$FABRIC.bin"
ls -la "$OUT/chipdb-$FABRIC.bin"
