#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PLATFORM_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

SRC_DIR="$PLATFORM_ROOT/src/third_party_libs/navigation_third_party_libs/osqp-0.6.3"
BUILD_DIR="$PLATFORM_ROOT/build/src/third_party_libs/navigation_third_party_libs/osqp-0.6.3"
INSTALL_DIR="$PLATFORM_ROOT/install/third_party_libs/navigation_third_party_libs/osqp-0.6.3"

rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"
mkdir -p "$INSTALL_DIR"

cmake "$SRC_DIR" \
  -DBUILD_SHARED_LIBS=ON \
  -DCMAKE_CXX_STANDARD=14 \
  -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR" \
  -B "$BUILD_DIR"

cmake --build "$BUILD_DIR" -j"${NUM_CORES:-$(nproc)}"
cmake --install "$BUILD_DIR"

echo "OSQP installed to:"
echo "$INSTALL_DIR"