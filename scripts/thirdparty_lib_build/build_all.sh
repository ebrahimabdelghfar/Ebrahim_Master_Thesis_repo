#!/usr/bin/env bash
# Builds all vendored thirdparty_lib/ sources into the isolated
# install/thirdparty_lib/<Name-version>/ prefix, in dependency order:
#   ipopt, osqp (independent) -> casadi (needs ipopt), osqp-eigen (needs osqp)
#   -> acados (independent)
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PLATFORM_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
INSTALL_ROOT="$PLATFORM_ROOT/install/thirdparty_lib"

"$SCRIPT_DIR/ipopt.sh"
"$SCRIPT_DIR/osqp.sh"
"$SCRIPT_DIR/casadi.sh"
"$SCRIPT_DIR/osqp-eigen.sh"
"$SCRIPT_DIR/acados.sh"

echo
echo "===== thirdparty_lib build summary ====="
echo "Ipopt:      $INSTALL_ROOT/Ipopt-stable-3.14"
echo "OSQP:       $INSTALL_ROOT/osqp-0.6.3"
echo "CasADi:     $INSTALL_ROOT/casadi-3.7.0"
echo "osqp-eigen: $INSTALL_ROOT/osqp-eigen-0.8.1"
echo "acados:     $INSTALL_ROOT/acados-0.5.5"
