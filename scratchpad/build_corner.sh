#!/usr/bin/env bash
# Rebuild the offline corner-entry harness against the already-built
# mpc_path_tracking core library, reusing the CMake compile/link lines.
set -euo pipefail

REPO=/home/ebrahim/Ebrahim_Master_Thesis_repo
BUILD=$REPO/build/mpc_path_tracking
SRC=${1:-$REPO/scratchpad/corner.cpp}
OUT=${2:-$BUILD/corner}

FLAGS=$(grep -E '^CXX_FLAGS' "$BUILD/CMakeFiles/test_mpc_controller.dir/flags.make" | cut -d= -f2-)
DEFS=$(grep -E '^CXX_DEFINES' "$BUILD/CMakeFiles/test_mpc_controller.dir/flags.make" | cut -d= -f2-)
INCS=$(grep -E '^CXX_INCLUDES' "$BUILD/CMakeFiles/test_mpc_controller.dir/flags.make" | cut -d= -f2-)

OBJ=$BUILD/$(basename "$SRC").o
# shellcheck disable=SC2086
c++ $FLAGS $DEFS $INCS -c "$SRC" -o "$OBJ"

# Reuse the test executable's link line, swapping in our object and dropping gtest.
LINK=$(tr '\n' ' ' < "$BUILD/CMakeFiles/test_mpc_controller.dir/link.txt")
LINK=${LINK/\/usr\/bin\/c++/}
LINK=${LINK/CMakeFiles\/test_mpc_controller.dir\/test\/test_mpc_controller.cpp.o/$OBJ}
LINK=${LINK/-o test_mpc_controller/-o $OUT}
LINK=${LINK//gtest\/libgtest_main.a/}
LINK=${LINK//gtest\/libgtest.a/}

cd "$BUILD"
# shellcheck disable=SC2086
c++ $LINK
echo "built $OUT"
