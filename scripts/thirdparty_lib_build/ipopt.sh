#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PLATFORM_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

SRC_DIR="$PLATFORM_ROOT/thirdparty_lib/Ipopt-stable-3.14"
BUILD_DIR="$PLATFORM_ROOT/build/thirdparty_lib/Ipopt-stable-3.14"
INSTALL_DIR="$PLATFORM_ROOT/install/thirdparty_lib/Ipopt-stable-3.14"

rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"
mkdir -p "$INSTALL_DIR"

cd "$BUILD_DIR"

"$SRC_DIR/configure" \
  --srcdir="$SRC_DIR" \
  --prefix="$INSTALL_DIR"

make -j"${NUM_CORES:-$(nproc)}"
make install

echo "IPOPT installed to:"
echo "$INSTALL_DIR"
