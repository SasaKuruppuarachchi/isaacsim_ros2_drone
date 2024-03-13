# Comprehensive Guide to Dokerised implementation of IsaacSim with ROS 2 for drone development

## Prerequisites
- Ubuntu 20.04/22.04 Operating System
- NVIDIA GPU (RTX 2070 or higher)
- NVIDIA GPU Driver (recommended version 525.85)

## Neucleas Server
[Download](https://docs.omniverse.nvidia.com/install-guide/latest/workstation-install.html) and install Omniverse Launcher and install [Neucleas Server](https://docs.omniverse.nvidia.com/launcher/latest/workstation-launcher.html#collaboration-tab)

## Installing docker
Follow the latest instructions at the [official docker page](https://docs.docker.com/engine/install/ubuntu/)

## Installing Portainer CE
Portainer CE is an open-source GUI for creating and managing docker containers. Install it following [these instructions](https://docs.portainer.io/start/install-ce/server/docker/linux)

## Installing Nvidia Docker
Install the Nvidia Container toolkit with [These Instructions](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

If GPUs fail to load after deploying Isaac sim docker please follow [these instructions]()

2. Clone the repo to your ros2 workspace
   ```bash
   git clone https://github.com/hijimasa/isaac-ros2-control-sample.git
   ```

3. Get git submodules
   ```bash
   cd isaac-ros2-control-sample
   git submodule update --init --recursive
   ```

4. Build a docker image with shell script.
   ```bash
   cd docker
   ./build_docker_image.sh
   ```

5. Launch a docker container
   ```bash
   ./launch_docker.sh
   ```

6. Build ros2 source codes
   ```bash
   colcon build && source install/setup.bash
   ```

7. Launch the package

   7.1. For Mobile Robot
   - To launch simulator
   ```bash
   ros2 run isaac_ros2_scripts launcher
   ```





> [!NOTE]
> For the first time, launching Isaac Sim takes a very long time.
> Isaac Sim must be fully launched to spawn the robot.
