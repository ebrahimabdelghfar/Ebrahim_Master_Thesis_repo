#!/usr/bin/env bash
# Builds the main ROS2 workspace (repo root) and fixes up
# install/setup.bash for the acados solver backend. Run this instead of a
# bare `colcon build` from the repo root whenever solver.backend: "acados"
# (mpc_path_tracking) is in play - see the fixup comment below for why.
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# ── Source ROS2 ──────────────────────────────────────────────────────────
ROS_DISTROS=("humble" "jazzy" "iron" "rolling")
ROS_SOURCED=false
for distro in "${ROS_DISTROS[@]}"; do
  setup="/opt/ros/${distro}/setup.bash"
  if [ -f "$setup" ]; then
    # shellcheck source=/dev/null
    source "$setup"
    echo "[build] Sourced ROS 2 ${distro} from $setup"
    ROS_SOURCED=true
    break
  fi
done

if [ "$ROS_SOURCED" = false ]; then
  echo "[build] ERROR: No ROS 2 installation found under /opt/ros/."
  exit 1
fi

cd "$WS_ROOT"
echo "[build] Workspace root: $WS_ROOT"
echo "[build] Running: colcon build --symlink-install $*"
colcon build --symlink-install "$@"

# ── acados LD_LIBRARY_PATH fixup ────────────────────────────────────────
# libacados.so (thirdparty_lib/, built by scripts/thirdparty_lib_build/
# acados.sh) has no RPATH of its own to its sibling libs (libhpipm.so,
# libqpOASES_e.so, libblasfeo.so.0 - all in the same lib/ dir). This is
# upstream acados' own build, not something introduced here - acados' own
# example launchers (thirdparty_lib/acados-0.5.5/examples/*/env.sh) work
# around it the same way. Without this, any node/test that selects
# mpc_path_tracking's solver.backend: "acados" fails to start with
# "error while loading shared libraries: libqpOASES_e.so: cannot open
# shared object file".
#
# colcon regenerates install/setup.bash from scratch on every build, so a
# one-off manual edit gets wiped by the next `colcon build` - this fixup
# has to run right after every build, hence it lives here instead.
SETUP_BASH="$WS_ROOT/install/setup.bash"
ACADOS_LIB_DIR="$WS_ROOT/install/thirdparty_lib/acados-0.5.5/lib"
LD_LIB_MARKER="# added by scripts/colcon_build.sh: acados LD_LIBRARY_PATH fixup"

if [ -f "$SETUP_BASH" ] && [ -d "$ACADOS_LIB_DIR" ]; then
  if ! grep -qF "$LD_LIB_MARKER" "$SETUP_BASH"; then
    {
      echo ""
      echo "$LD_LIB_MARKER"
      echo "export LD_LIBRARY_PATH=\"$ACADOS_LIB_DIR:\$LD_LIBRARY_PATH\""
    } >> "$SETUP_BASH"
    echo "[build] Appended acados LD_LIBRARY_PATH fixup to $SETUP_BASH"
  fi
fi
