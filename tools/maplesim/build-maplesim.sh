#!/usr/bin/env bash
# Builds maple-sim from source and drops the jars into libs/.
#
# The published 0.4.0-beta vendordep is missing fixes we rely on (red hub fuel recycling, the match
# timer, the relaxed wheel COF bound) and the newer "0.4.0-beta-obstacles-fix" jar it points to was
# never uploaded, so we build it ourselves. Point MAPLESIM_REPO at our fork to pick up our changes.
#
# Usage: tools/maplesim/build-maplesim.sh [git ref]
#   MAPLESIM_REPO  repo to clone (default: upstream)
#   git ref        commit, tag or branch (default: the commit we currently vendor)
set -euo pipefail

REPO="${MAPLESIM_REPO:-https://github.com/Shenzhen-Robotics-Alliance/maple-sim}"
REF="${1:-1b1e8ea}"
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

if [ -z "${JAVA_HOME:-}" ]; then
  for year in 2026 2027; do
    [ -d "$HOME/wpilib/$year/jdk" ] && export JAVA_HOME="$HOME/wpilib/$year/jdk"
  done
fi

git clone -q "$REPO" "$WORK/maple-sim"
git -C "$WORK/maple-sim" checkout -q "$REF"
SHA="$(git -C "$WORK/maple-sim" rev-parse --short=7 HEAD)"

cd "$WORK/maple-sim/project"
# Upstream's build script calls a Spotless method that its pinned Spotless version doesn't have.
sed -i.bak 's/leadingTabsToSpaces(\([0-9]*\))/indentWithSpaces(\1)/' build.gradle
./gradlew jar sourcesJar -x test -x spotlessApply --console=plain -q

rm -f "$ROOT"/libs/maplesim-java-*.jar
cp build/libs/project.jar "$ROOT/libs/maplesim-java-$SHA.jar"
cp build/libs/project-sources.jar "$ROOT/libs/maplesim-java-$SHA-sources.jar"
echo "Built maple-sim $SHA into libs/. Update the maplesim jar name in build.gradle if it changed."
