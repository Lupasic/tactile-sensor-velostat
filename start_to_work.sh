#!/bin/bash

DIRNAME=$(dirname "$0")

DOCKER_FILE="docker-compose.yml"
if command -v nvidia-smi
then
    echo "Nvidia drivers were found"
    DOCKER_FILE="docker-compose_nvidia.yml"
fi

echo "Docker compose file is $DOCKER_FILE"
# $DIRNAME
gnome-terminal --tab --title="docker_up" -e "bash -c \"docker-compose -f $DOCKER_FILE up; exec bash\"" --tab -e "bash -c \"sleep 5; docker exec -it --privileged tactile-sensor-velostat_master_1 bash; exec bash\"" --tab -e "bash -c \"git status; exec bash\" ";
