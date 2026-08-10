green= \033[0;32m
red= \033[0;31m
yellow= \033[0;33m
nc= \033[0m
reset= \033[0m
# Usage: make [command]
SHELL:=/bin/bash
WORKSPACE=$(shell pwd)
RACELINE_CSV?=$(WORKSPACE)/src/on_track_sys_id/config/raceline.csv
WAYPOINT_TOPIC?=/raceline_waypoints
FRAME_ID?=map
PUBLISH_INITIAL_POSE?=true
ENV_NAME?=identification_env
setup_conda_env:
	@echo -e "${green}Setting up conda environment: $(ENV_NAME)${reset}"
	@conda env create -f environment.yml -n $(ENV_NAME) || conda env update -f environment.yml -n $(ENV_NAME)
activate_conda_env:
	@echo -e "${yellow}conda can't activate parent shell from make. Run:${reset}"
	@echo -e "${green}conda activate $(ENV_NAME)${reset}"
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
launch_raceline_publisher:
	source /opt/ros/humble/setup.bash && source ${WORKSPACE}/install/setup.bash && \
	ros2 launch raceline_publisher raceline_publisher.launch.py \
	raceline_csv:=$(RACELINE_CSV) \
	waypoint_topic:=$(WAYPOINT_TOPIC) \
	frame_id:=$(FRAME_ID) \
	publish_initial_pose:=$(PUBLISH_INITIAL_POSE)
launch_on_track_sys_id:
	source /opt/ros/humble/setup.bash && source ${WORKSPACE}/install/setup.bash && \
	ros2 launch on_track_sys_id sys_id.launch.py
launch_on_track_sys_id_with_graph:
	source /opt/ros/humble/setup.bash && source ${WORKSPACE}/install/setup.bash && \
	ros2 launch on_track_sys_id sys_id.launch.py  \
	enable_tire_force_benchmark:=true \
	benchmark_update_params_enable:=true \
	tire_force_benchmark_plot_output_dir:=/home/ebrahim/Ebrahim_Master_Thesis_repo/graphs/identification/identification_with_friction_warm_start_base_line
launch_adaptive_controll:
	source /opt/ros/humble/setup.bash && source ${WORKSPACE}/install/setup.bash && \
	ros2 launch adaptive_controller_manager adaptive_stack.launch.py
launch_adaptive_control_with_graph:
	source /opt/ros/humble/setup.bash && source ${WORKSPACE}/install/setup.bash && \
	ros2 launch adaptive_controller_manager adaptive_stack.launch.py  \
	enable_controller_benchmark:=true \
	controller_benchmark_plot_output_dir:=/home/ebrahim/Ebrahim_Master_Thesis_repo/graphs/control/control_with_friction_warm_start_base_line
setup_ros2_workspace:
	source /opt/ros/humble/setup.bash && \
	bash ${WORKSPACE}/scripts/colcon_build.sh