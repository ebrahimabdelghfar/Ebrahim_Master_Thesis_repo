#!/usr/bin/env bash
# Adaptive-preview sweep. YAML mirrors mpc_path_tracking.yaml verbatim; the tire
# set is the one On-Track-SysID actually pushes through mpc/update_params.
# Caller flags go LAST because corner.cpp's arg() is last-match-wins.
set -u
BIN=/home/ebrahim/Ebrahim_Master_Thesis_repo/build/mpc_path_tracking/corner
OUT=${OUT:-/tmp/claude-1000/-home-ebrahim-Ebrahim-Master-Thesis-repo/9587289a-09d3-4842-a4bb-78643c50351c/scratchpad/hsweep}
mkdir -p "$OUT"
YAML=(--csv /home/ebrahim/Ebrahim_Master_Thesis_repo/traj_race_cl.csv
  --N 20 --dt-min 0.02 --dt-max 0.07 --horizon-distance 5.5 --rate 20
  --q-ey 10 --q-epsi 5 --q-vx 25 --q-r 100
  --r-steer 10 --r-accel 1 --rrate-steer 10 --rrate-accel 50
  --steer-rate-max 1.0 --accel 5.71 --decel 3.71 --speed-max 27.77
  --alat 6.5 --grip-util 0.70
  --tau-cfg 2.77 --tau-cfg-decel -0.2 --tau-auto 0
  --ctrl-Bf 5.1014 --ctrl-Cf 1.4474 --ctrl-Df 2.0 --ctrl-Ef -2.8165
  --ctrl-Br 5.4111 --ctrl-Cr 1.4651 --ctrl-Dr 2.0 --ctrl-Er -1.7593)

run() { local label=$1; shift; "$BIN" "${YAML[@]}" "$@" > "$OUT/$label.txt" 2>&1; echo "$label"; }
