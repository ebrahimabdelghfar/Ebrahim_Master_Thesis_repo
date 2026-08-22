#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PLATFORM_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

SRC_DIR="$PLATFORM_ROOT/thirdparty_lib/acados-0.5.5"
BUILD_DIR="$PLATFORM_ROOT/build/thirdparty_lib/acados-0.5.5"
INSTALL_DIR="$PLATFORM_ROOT/install/thirdparty_lib/acados-0.5.5"

rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"
mkdir -p "$INSTALL_DIR"

cd "$BUILD_DIR"

# libacados.so pulls in libqpOASES_e/libhpipm/libblasfeo as transitive deps.
# A consumer's DT_RUNPATH is not used to resolve *its* dependencies' needs, so
# without an RPATH of its own libacados.so only loads when the acados lib dir
# happens to be on LD_LIBRARY_PATH. $ORIGIN makes it self-locating.
cmake "$SRC_DIR" \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_SHARED_LIBS=ON \
    -DACADOS_WITH_QPOASES=ON \
    -DACADOS_WITH_OPENMP=ON \
    -DCMAKE_INSTALL_RPATH='$ORIGIN' \
    -DCMAKE_BUILD_WITH_INSTALL_RPATH=ON \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_DIR}"

make -j"${NUM_CORES:-$(nproc)}"
make install

echo "acados installed to:"
echo "$INSTALL_DIR"