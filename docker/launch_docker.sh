#!/bin/bash
xhost +local:

docker run --name isaac-sim-ros2 --entrypoint bash -it --gpus all -e "ACCEPT_EULA=Y" --rm --network=host --privileged \
    -v ~/docker/isaac-sim-ros2/cache/kit:/isaac-sim/kit/cache/Kit:rw \
    -v ~/docker/isaac-sim-ros2/cache/ov:/root/.cache/ov:rw \
    -v ~/docker/isaac-sim-ros2/cache/pip:/root/.cache/pip:rw \
    -v ~/docker/isaac-sim-ros2/cache/glcache:/root/.cache/nvidia/GLCache:rw \
    -v ~/docker/isaac-sim-ros2/cache/computecache:/root/.nv/ComputeCache:rw \
    -v ~/docker/isaac-sim-ros2/logs:/root/.nvidia-omniverse/logs:rw \
    -v ~/docker/isaac-sim-ros2/data:/root/.local/share/ov/data:rw \
    -v ~/docker/isaac-sim-ros2/documents:/root/Documents:rw \
    --env="DISPLAY" \
    -v $HOME/.Xauthority:/root/.Xauthority:rw \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --workdir="/root/colcon_ws" \
    --volume="$(pwd)/../colcon_ws:/root/colcon_ws" \
    --volume="$(pwd)/../lidar_cfg/hokuyo:/isaac-sim/exts/omni.isaac.sensor/data/lidar_configs/hokuyo" \
    isaac-ros2-image:latest	
