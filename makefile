# Usage: make [command]
SHELL:=/bin/bash
WORKSPACE=$(shell pwd)
build_docker:
	docker build -t eth_sys:ros2 -f ./docker/dockerfile .
up_container:
	docker compose -f ./docker/docker_compose.yml up -d
down_container:
	docker compose -f ./docker/docker_compose.yml down
exec_container:
	docker compose -f ./docker/docker_compose.yml exec ros2_eth /bin/bash
launch_f1_simulator:
	source /opt/ros/humble/setup.bash && source ${WORKSPACE}/install/setup.bash && \
	ros2 launch f1tenth_simulator simulator.launch.py 
launch_on_track_sys_id:
	source /opt/ros/humble/setup.bash && source ${WORKSPACE}/install/setup.bash && \
	ros2 launch on_track_sys_id sys_id.launch.py
launch_adaptive_controll:
	source /opt/ros/humble/setup.bash && source ${WORKSPACE}/install/setup.bash && \
	ros2 launch adaptive_controller_manager adaptive_stack.launch.py