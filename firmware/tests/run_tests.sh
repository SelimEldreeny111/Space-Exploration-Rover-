#!/usr/bin/env bash
# Build and run the MissionX_SSP host unit tests (Git Bash / Linux).
#   1. gen_vectors.py writes cross-implementation vectors using the Python
#      reference implementation (ros2_ws/src/rover_core) as oracle.
#   2. test_ssp is built twice (CRC table and -DSSP_CRC_BITWISE) and replays
#      them; test_logic checks pure firmware logic shared with the sketches.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FW="$(cd "$HERE/.." && pwd)"
LIB="$FW/libraries/MissionX_SSP/src"
OUT="$FW/build/tests"
CXX="${CXX:-g++}"
PY="${PYTHON:-python}"
CXXFLAGS="-std=c++11 -O2 -Wall -Wextra -Wshadow -Werror"
mkdir -p "$OUT"

echo "== generating vectors with the Python reference implementation"
PYTHONDONTWRITEBYTECODE=1 "$PY" -B "$HERE/gen_vectors.py" "$OUT/vectors.txt"

SRCS=("$LIB/ssp_core.cpp" "$LIB/ssp_protocol.cpp" "$LIB/MissionX_SSP.cpp")

echo "== building test_ssp (CRC table)"
"$CXX" $CXXFLAGS -I"$LIB" "$HERE/test_ssp.cpp" "${SRCS[@]}" -o "$OUT/test_ssp"
echo "== building test_ssp (bitwise CRC)"
"$CXX" $CXXFLAGS -DSSP_CRC_BITWISE -I"$LIB" "$HERE/test_ssp.cpp" "${SRCS[@]}" -o "$OUT/test_ssp_bitwise"

FAILED=0
echo "== running test_ssp"
"$OUT/test_ssp" "$OUT/vectors.txt" || FAILED=1
echo "== running test_ssp (bitwise CRC)"
"$OUT/test_ssp_bitwise" "$OUT/vectors.txt" || FAILED=1

if [ -f "$HERE/test_logic.cpp" ]; then
  echo "== building test_logic (firmware logic shared with the sketches)"
  "$CXX" $CXXFLAGS -I"$LIB" -I"$FW/mobility_mega" -I"$FW/payload_uno" -I"$FW/arm_uno" \
    "$HERE/test_logic.cpp" "$FW/mobility_mega/drive_math.cpp" "$FW/mobility_mega/scan_logic.cpp" \
    "$FW/mobility_mega/legacy_tlm.cpp" "$LIB/ssp_core.cpp" "$LIB/ssp_protocol.cpp" \
    "$FW/payload_uno/bmp180_comp.cpp" "$FW/payload_uno/imu_math.cpp" "$FW/arm_uno/arm_math.cpp" -o "$OUT/test_logic"
  echo "== running test_logic"
  "$OUT/test_logic" || FAILED=1
fi

if [ "$FAILED" -ne 0 ]; then
  echo "HOST TESTS FAILED"
  exit 1
fi
echo "ALL HOST TESTS PASSED"
