#!/usr/bin/env bash
# Shipped working-tree config + the tire set On-Track-SysID currently identifies.
# NOTE: corner.cpp's arg() returns the FIRST match, so "$@" must come first for
# caller overrides to win.
IDENT="--ctrl-Bf 5.1014 --ctrl-Cf 1.4474 --ctrl-Df 2.0 --ctrl-Ef -2.8165 \
       --ctrl-Br 5.4111 --ctrl-Cr 1.4651 --ctrl-Dr 2.0 --ctrl-Er -1.7593"
exec /home/ebrahim/Ebrahim_Master_Thesis_repo/build/mpc_path_tracking/corner \
  "$@" \
  --csv /home/ebrahim/Ebrahim_Master_Thesis_repo/traj_race_cl.csv \
  --N 20 --rate 20 --alat 6.5 --grip-util 0.7 \
  --decel 3.71 --accel 5.71 --speed-max 31.0 \
  --tau-cfg 0.2 --tau-auto 0 --tau-plant 2.77 --mu 1.0 \
  $IDENT
