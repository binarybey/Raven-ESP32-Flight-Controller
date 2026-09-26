#!/usr/bin/env bash
# Host-side (PC) tests for the Arduino-free modules: NMEA parser, F-Code
# interpreter, DEM terrain reader, and FlightKinematics mode/mission/failsafe
# logic. Terrain checks use test/vtol_bin_tiles if present (not in git).
#
# Needs a host C++17 compiler. Default: zig's bundled clang via
#     python -m pip install ziglang
# or point CXX at any other compiler, e.g.  CXX=g++ ./test/host/run_host_tests.sh
#
# PlatformIO's `pio test` ignores this folder (it only runs test_* folders).
set -e
cd "$(dirname "$0")/../.."
CXX=${CXX:-"python -m ziglang c++"}
OUT="${TMPDIR:-/tmp}/ahrs_host_tests.exe"
$CXX -std=gnu++17 -O1 -Wall -Wextra \
  -I"lib/ReadGNSS" -I"lib/F-Code Interpreter" -I"lib/Flight Kinematics" -I"lib/Terrain" -I"lib/Hardware Interface" \
  test/host/host_tests.cpp \
  lib/ReadGNSS/nmea_parse.cpp \
  "lib/F-Code Interpreter/fcode_interpreter.cpp" \
  "lib/Flight Kinematics/flight_kinematics.cpp" \
  lib/Terrain/terrain.cpp \
  -o "$OUT" 2>&1 | grep -v "nullability" | grep -E "error|warning: [^n]" || true
"$OUT"
