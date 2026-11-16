#!/usr/bin/env bash
# Compile every MISSION X sketch with arduino-cli (Git Bash / Linux / macOS).
#   ./build_all.sh                 compile all four sketches (incremental)
#   ./build_all.sh --clean         full rebuild (shows every warning again)
#   ARDUINO_CLI=/path/arduino-cli ./build_all.sh
# Build output goes to firmware/build/<sketch>/ (git-ignored). Prints flash /
# RAM usage and fails on any compiler warning coming from our own code
# (warnings inside the Arduino core or third-party libraries are ignored).
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if command -v cygpath >/dev/null 2>&1; then FW="$(cygpath -m "$HERE")"; else FW="$HERE"; fi
CLI="${ARDUINO_CLI:-/d/Library/3-Programs/arduino-cli/arduino-cli_1.4.1_Windows_64bit/arduino-cli.exe}"
command -v "$CLI" >/dev/null 2>&1 || CLI="arduino-cli"
CLEAN=""
[ "${1:-}" = "--clean" ] && CLEAN="--clean"

SKETCHES=("mobility_mega:arduino:avr:mega" "payload_uno:arduino:avr:uno"
          "arm_uno:arduino:avr:uno" "legacy_fixed:arduino:avr:mega")
FAIL=0
printf "%-14s %-18s %s\n" "sketch" "board" "result"
for entry in "${SKETCHES[@]}"; do
  name="${entry%%:*}"
  fqbn="${entry#*:}"
  log="$HERE/build/$name.log"
  mkdir -p "$HERE/build/$name"
  "$CLI" compile --fqbn "$fqbn" --warnings all $CLEAN \
    --library "$FW/libraries/MissionX_SSP" \
    --build-path "$FW/build/$name" "$FW/$name" >"$log" 2>&1
  rc=$?
  # our code = anything under firmware/ (sketches, build/<name>/sketch copies,
  # MissionX_SSP); the core (Arduino15) and installed libraries live elsewhere
  own_warn=$(grep -E "(warning|error):" "$log" | grep -E 'firmware[/\\]' || true)
  flash=$(grep -o "Sketch uses [0-9]* bytes ([0-9]*%)" "$log" | head -1)
  ram=$(grep -o "Global variables use [0-9]* bytes ([0-9]*%)" "$log" | head -1)
  if [ $rc -ne 0 ]; then
    printf "%-14s %-18s FAILED (see %s)\n" "$name" "$fqbn" "$log"; FAIL=1
  elif [ -n "$own_warn" ]; then
    printf "%-14s %-18s WARNINGS:\n%s\n" "$name" "$fqbn" "$own_warn"; FAIL=1
  else
    printf "%-14s %-18s ok  %s, %s\n" "$name" "$fqbn" "${flash#Sketch uses }" "${ram#Global variables use }"
  fi
done
exit $FAIL
