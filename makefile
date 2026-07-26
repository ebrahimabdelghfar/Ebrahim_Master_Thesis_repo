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