#!/bin/bash
IMAGE=$1
CONTAINER=$2
WS="${3:-"ws"}"

# The directory from which script has been run
SIM_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
echo "Path to ${WS} volume folder: ${SIM_ROOT}/${WS}"

# TODO: делать xhost -local:docker после удаления контейнера
xhost +local:docker && \
  docker run -d -it --rm \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ~/.Xauthority:/home/ubuntu/.Xauthority \
    -v $XDG_RUNTIME_DIR/$WAYLAND_DISPLAY:/tmp/$WAYLAND_DISPLAY \
    -v /etc/localtime:/etc/localtime:ro \
    -v ${SIM_ROOT}/:/ws \
    -v /dev:/dev \
    --ipc=host \
    --network=host \
    --device-cgroup-rule='c *:* rmw' \
    --name $CONTAINER $IMAGE
