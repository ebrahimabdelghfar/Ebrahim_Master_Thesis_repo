#!/bin/bash
source ../install/setup.bash
# Note: install is in parent dir because I built from parent... wait
# I built with Cwd=/home/ebrahim/Ebrahim_Master_Thesis_repo/twist_to_ackermann.
# So install should be in twist_to_ackermann/install or similar?
# Let's check where install is.
if [ -f install/setup.bash ]; then
    source install/setup.bash
else
    source ../install/setup.bash
fi

ros2 launch twist_to_ackermann twist_to_ackermann.launch.py &
PID=$!
sleep 5
echo "--- Topic List ---"
ros2 topic list
echo "--- Parameters ---"
ros2 param get /twist_to_ackermann twist_topic
ros2 param get /twist_to_ackermann ackermann_topic
ros2 param get /twist_to_ackermann wheelbase
kill $PID
