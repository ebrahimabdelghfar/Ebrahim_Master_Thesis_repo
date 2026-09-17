#!/usr/bin/env bash
# Offline friction-adaptation sweep: seven controller configurations against a
# decaying and a step-dropping surface, all in the real MpcController + acados +
# ReferenceTrajectoryHandler (scratchpad/corner.cpp).
#
# Each arm adds one term to the one above it, so the row where the grip criterion
# starts holding is the term that earned its place. Traces land in --out-dir.
set -euo pipefail

REPO=/home/ebrahim/Ebrahim_Master_Thesis_repo
BIN=$REPO/build/mpc_path_tracking/corner
OUT_DIR=${OUT_DIR:-$REPO/scratchpad/friction_sweep}
mkdir -p "$OUT_DIR"

# The coefficient set On-Track-SysID identifies, and a drivetrain lag matched to
# the plant. A mismatched tau spins the car on its own and would drown out
# everything this sweep is measuring.
IDENT="--ctrl-Bf 5.1014 --ctrl-Cf 1.4474 --ctrl-Df 2.0 --ctrl-Ef -2.8165 \
       --ctrl-Br 5.4111 --ctrl-Cr 1.4651 --ctrl-Dr 2.0 --ctrl-Er -1.7593"
BASE="--csv $REPO/traj_race_cl.csv --N 20 --rate 20 --alat 6.5 --grip-util 0.7 \
      --decel 3.71 --accel 5.71 --speed-max 31.0 \
      --tau-cfg 2.77 --tau-auto 0 --tau-plant 2.77 --mu 1.0 --dur 200 --sysid-interval 30"

# Each arm is the one above it plus one term, so the row where the grip
# criterion starts holding names the term that earned its place.
ARMS=(
  "today|--grip-longitudinal 0 --friction-ellipse 0 --shape-util 0 --sigma-gain 0 --mu-fast-rate 0"
  "longitudinal|--grip-longitudinal 1 --friction-ellipse 0 --shape-util 0 --sigma-gain 0 --mu-fast-rate 0"
  "ellipse|--grip-longitudinal 1 --friction-ellipse 1 --shape-util 0 --sigma-gain 0 --mu-fast-rate 0"
  "shape|--grip-longitudinal 1 --friction-ellipse 1 --shape-util 1 --sigma-gain 0 --mu-fast-rate 0"
  "fastmu|--grip-longitudinal 1 --friction-ellipse 1 --shape-util 1 --sigma-gain 0 --mu-fast-rate 1"
  "sigma|--grip-longitudinal 1 --friction-ellipse 1 --shape-util 1 --sigma-gain 1 --mu-fast-rate 1"
  "fallback|--grip-longitudinal 1 --friction-ellipse 1 --shape-util 1 --sigma-gain 1 --mu-fast-rate 0"
)

# 0.5 %/s is slow enough for a 30 s identification cycle to track and fast enough
# to exhaust the margin within a run. 2 %/s is not observable at that cycle: the
# prediction model ends up four times too grippy and no amount of reference-side
# derating recovers it, which is the measurement that sets
# reidentification_interval_s rather than a property of the arms.
SCHEDULES=(
  "decay|--mu-schedule decay --mu-decay-per-s 0.005 --mu-floor 0.5"
  "decay2pct|--mu-schedule decay --mu-decay-per-s 0.02 --mu-floor 0.5"
  "step|--mu-schedule step --mu-step-t 40 --mu-step-frac 0.6"
)

printf '%-10s %-6s %8s %8s %6s %6s %8s\n' arm sched ay_ratio verdict spins fails "max|e_y|"
for s in "${SCHEDULES[@]}"; do
  sched_name=${s%%|*}; sched_flags=${s#*|}
  for arm in "${ARMS[@]}"; do
    arm_name=${arm%%|*}; arm_flags=${arm#*|}
    out=$OUT_DIR/${sched_name}_${arm_name}.csv
    # shellcheck disable=SC2086
    line=$("$BIN" $BASE $IDENT $sched_flags $arm_flags --out "$out")
    ratio=$(sed -n 's/.*mu_plant\*g) = \([0-9.]*\).*/\1/p' <<<"$line")
    verdict=$(sed -n 's/.*mu_plant\*g) = [0-9.]* at t=[0-9.]* s \[\([A-Z]*\)\].*/\1/p' <<<"$line")
    spins=$(sed -n 's/.*| spins \([0-9]*\) \[.*/\1/p' <<<"$line")
    fails=$(sed -n 's/.*solver failures \([0-9]*\) \[.*/\1/p' <<<"$line")
    ey=$(sed -n 's/.*max|e_y| \([0-9.]*\) m |.*/\1/p' <<<"$line")
    printf '%-10s %-6s %8s %8s %6s %6s %8s\n' \
      "$arm_name" "$sched_name" "$ratio" "$verdict" "$spins" "$fails" "$ey"
  done
done

echo
echo "The fallback arm runs the full stack with nothing published on sysid/friction,"
echo "so its row must match the shape arm's. Compare the summary metrics, not the"
echo "traces: acados/HPIPM is not bitwise reproducible run to run (~1e-7 on u0), so"
echo "two runs of the SAME arm already differ in the trace."
echo
echo "How short an identification cycle the 2 %/s decay needs, full stack:"
for si in 30 20 10 5; do
  # shellcheck disable=SC2086
  line=$("$BIN" $BASE $IDENT --sysid-interval $si \
    --mu-schedule decay --mu-decay-per-s 0.02 --mu-floor 0.5 \
    --grip-longitudinal 1 --friction-ellipse 1 --shape-util 1 --sigma-gain 1 --mu-fast-rate 1)
  printf '  reidentification_interval_s=%-3s %s\n' "$si" \
    "$(sed -n 's/.*mu_plant\*g) = \([0-9.]* at t=[0-9.]* s \[[A-Z]*\]\).*/ay_ratio \1/p' <<<"$line")"
done
