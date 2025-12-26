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

