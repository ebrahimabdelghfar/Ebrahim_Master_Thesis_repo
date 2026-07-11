#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PLATFORM_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

SRC_DIR="$PLATFORM_ROOT/third_party_libs/casadi-3.7.0"
BUILD_DIR="$PLATFORM_ROOT/build/third_party_libs/casadi-3.7.0"
INSTALL_DIR="$PLATFORM_ROOT/install/third_party_libs/casadi-3.7.0"
IPOPT_INSTALL_DIR="$PLATFORM_ROOT/install/third_party_libs/ipopt-stable-3.14"

rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"
mkdir -p "$INSTALL_DIR"

cd "$BUILD_DIR"

cmake "$SRC_DIR" \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_SHARED_LIBS=ON \
  -DWITH_IPOPT=ON \
  -DIPOPT_INCLUDE_DIRS="${IPOPT_INSTALL_DIR}/include/coin-or" \
  -DIPOPT_LIBRARIES="${IPOPT_INSTALL_DIR}/lib/libipopt.so" \
  -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR"

make -j"${NUM_CORES:-$(nproc)}"
make install

echo "CasADi installed to:"
echo "$INSTALL_DIR"