#!/bin/zsh
# Build all available V6 modules headless, audit clearances + ROM, then load into the live GUI.
# usage: scripts/integrate.sh [modules]   (default: all modules that exist, in pipeline order)
set -u
FC=/Applications/FreeCAD.app/Contents/Resources/bin/freecadcmd
D=/path/to/DexiGrab-Robot-Hand/Hardware/V6_FreeCAD/freecad
cd $D
MODS=${1:-$(for m in j0 pillars thumb actuation shells electronics fixes tactile proportions splay hardware; do [ -f lib/mod_$m.py ] && printf "%s," $m; done | sed 's/,$//')}
STAMP=$(date +%H%M%S)
W=$D/work/integrate_$STAMP; mkdir -p $W
cp DexiGrab_V6.FCStd $W/base.FCStd
echo "modules: $MODS -> $W"
V6_RUN=1 V6_IN=$W/base.FCStd V6_MODULES=$MODS V6_BREP=$D/build V6_OUT=$W/built.FCStd $FC scripts/build_v6.py > $W/build.log 2>&1
tr '\r\t' '\n\n' < $W/build.log | grep -E "vol |manifest|saved|Exception|Error|Traceback"
V6_IN=$W/built.FCStd V6_AUDIT_OUT=$W/audit.csv $FC scripts/audit_clearance.py > $W/audit.log 2>&1
tr '\r\t' '\n\n' < $W/audit.log | grep -E "^\('INTERF|pairs flagged|Exception|Traceback"
V6_IN=$W/built.FCStd V6_ROM_OUT=$W/rom.csv $FC scripts/rom_sweep.py > $W/rom.log 2>&1
tr '\r\t' '\n\n' < $W/rom.log | grep -E "COLLIDES|done|Exception|Traceback"
echo "workdir $W"
