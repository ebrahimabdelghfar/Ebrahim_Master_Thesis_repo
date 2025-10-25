# Usage: make [command]
SHELL:=/bin/bash
WORKSPACE=$(shell pwd)
build_docker:
	docker build -t eth_sys:latest -f ./docker/dockerfile .
up_container:
	docker compose -f ./docker/docker_compose.yml up
exec_container:
	docker compose -f ./docker/docker_compose.yml exec ros1_eth /bin/bash

