#!/usr/bin/env bash
# Starts the full robot sim in one go: robot code + Sim GUI, then AdvantageScope connected to it
# and showing this repo's robot model.
#
#   ./sim.sh
#
# Then in the Sim GUI: click Teleoperated (or pick an auto and click Autonomous), click the Sim GUI
# window so it gets the keyboard, and drive with WASD + Q/E.
set -uo pipefail
cd "$(dirname "$0")" || exit 1
REPO="$(pwd)"

# Only one sim can own NetworkTables; close any old one (from this folder or any other). Gradle
# starts the robot as "java -jar build/libs/<project>.jar", so find it by the NetworkTables port.
OLD_SIMS="$(lsof -tiTCP:5810 -sTCP:LISTEN 2> /dev/null)"
if [ -n "$OLD_SIMS" ]; then
  echo "Closing the sim that's already running..."
  kill $OLD_SIMS
  for _ in $(seq 1 20); do
    lsof -tiTCP:5810 -sTCP:LISTEN > /dev/null 2>&1 || break
    sleep 0.5
  done
fi

# The Sim GUI rewrites simgui-ds.json when it closes, which can lose the keyboard bindings, so put
# them back every time
cp tools/sim/simgui-ds.json simgui-ds.json

# Point AdvantageScope at this repo's robot model (it reads one assets folder, set in its prefs) and
# open the Sim layout (tools/ascope/sim_layout.json), whose 3D tab shows the robot where the physics
# has it, climbing over the BUMPs. AdvantageScope rewrites both files when it quits, so quit it first.
ASCOPE="$HOME/wpilib/2026/advantagescope/AdvantageScope (WPILib).app"
[ -d "$ASCOPE" ] || ASCOPE="/Applications/AdvantageScope (WPILib).app"
SUPPORT="$HOME/Library/Application Support/AdvantageScope"
PREFS="$SUPPORT/prefs.json"
LAYOUT_VERSION="$(python3 -I -c 'import json, sys; print(json.load(open(sys.argv[1]))["version"])' \
  tools/ascope/sim_layout.json)"
STATE="$SUPPORT/state-${LAYOUT_VERSION//./_}.json"
NEEDS_PREFS=false
NEEDS_LAYOUT=false
[ -f "$PREFS" ] && ! grep -q "\"userAssetsFolder\": *\"$REPO/ascope_assets\"" "$PREFS" && NEEDS_PREFS=true
[ -d "$SUPPORT" ] && ! grep -qs "FieldSimulation/RobotPose3d" "$STATE" && NEEDS_LAYOUT=true
if $NEEDS_PREFS || $NEEDS_LAYOUT; then
  if pgrep -f "Contents/MacOS/AdvantageScope$" > /dev/null; then
    echo "Restarting AdvantageScope to load this robot's model and the Sim layout..."
    osascript -e "tell application \"$ASCOPE\" to quit" > /dev/null 2>&1
    for _ in $(seq 1 20); do
      pgrep -f "Contents/MacOS/AdvantageScope$" > /dev/null || break
      sleep 0.5
    done
  fi
fi
if $NEEDS_PREFS; then
  python3 -I -c '
import json, sys
path, folder = sys.argv[1], sys.argv[2]
prefs = json.load(open(path))
prefs["userAssetsFolder"] = folder
json.dump(prefs, open(path, "w"), indent=2)
' "$PREFS" "$REPO/ascope_assets"
fi
if $NEEDS_LAYOUT; then
  # Keep the old window layout around in case it was worth something
  [ -f "$STATE" ] && cp "$STATE" "$STATE.before-sim-layout"
  python3 -I -c '
import json, sys
layout, state = sys.argv[1], sys.argv[2]
d = json.load(open(layout))
json.dump({"hubs": d["hubs"], "satellites": d.get("satellites", [])}, open(state, "w"))
' tools/ascope/sim_layout.json "$STATE"
fi

echo "Building and starting the sim (the Sim GUI opens when it's ready)..."
./gradlew simulateJava &
SIM_PID=$!
trap 'kill $SIM_PID 2>/dev/null' EXIT

# Wait for the robot code to start serving NetworkTables
for _ in $(seq 1 240); do
  nc -z 127.0.0.1 5810 2> /dev/null && break
  if ! kill -0 $SIM_PID 2> /dev/null; then
    echo "The sim exited before it started; see the errors above."
    exit 1
  fi
  sleep 0.5
done

# Open AdvantageScope (it remembers the Sim layout) and connect it
open "$ASCOPE"
sleep 4
if ! osascript -e "tell application \"$ASCOPE\" to activate" \
    -e 'delay 0.5' \
    -e 'tell application "System Events" to keystroke "p" using {command down, shift down}' \
    > /dev/null 2>&1; then
  echo "Couldn't connect AdvantageScope automatically: press Cmd+Shift+P in it (Connect to Simulator)."
  echo "(To let this script do it, allow your terminal under System Settings > Privacy & Security >"
  echo " Accessibility.)"
fi

echo
echo "Sim is up. In the Sim GUI: click Teleoperated, click the window, drive with WASD + Q/E."
echo "Space shoot, I intake, J jitter intake, H idle, hold U unjam, Shift/Ctrl slow."
echo "Close the Sim GUI window (or Ctrl+C here) to stop."
wait $SIM_PID
