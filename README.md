# Comprehensive Guide to Dokerised implementation of IsaacSim with ROS 2 for drone development

## Prerequisites
- Ubuntu 20.04/22.04 Operating System
- NVIDIA GPU (RTX 2070 or higher)
- NVIDIA GPU Driver (recommended version 525.85)

## Nucleus Server
[Download](https://docs.omniverse.nvidia.com/install-guide/latest/workstation-install.html) and install Omniverse Launcher and install [Nucleus Server](https://docs.omniverse.nvidia.com/launcher/latest/workstation-launcher.html#collaboration-tab)

## Installing docker
Follow the latest instructions at the [official docker page](https://docs.docker.com/engine/install/ubuntu/)

## Installing Portainer CE
Portainer CE is an open-source GUI for creating and managing docker containers. Install it following [these instructions](https://docs.portainer.io/start/install-ce/server/docker/linux)

## Installing Nvidia Docker
Install the Nvidia Container toolkit with [These Instructions](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

If GPUs fail to load after deploying Isaac sim docker please follow [these instructions](). TODO

## General Installation

1. Clone the repo to your ros2 workspace
   ```bash
   git clone https://github.com/SasaKuruppuarachchi/isaacsim_ros2_drone.git
   ```

2. Get git submodules
   ```bash
   cd isaacsim_ros2_drone
   git submodule update --init --recursive
   ```

3. Build a docker image with shell script.
   ```bash
   cd docker
   ./build_docker_image.sh
   ```

4. Launch a docker container
   ```bash
   ./launch_docker.sh
   ```
5. Launch Isaac Sim

   At this point, you are expected to have NVIDIA Isaac Sim fully installed and working. To make sure you have everything setup correctly, open a new terminal window (Ctrl+Alt+T), and test the following commands:

   Check that the simulator app can be launched

   - Run the simulator with the --help argument to see all available options
   ```bash
   ISAACSIM --help
   ```
   - Run the simulator. A new window should open
   ```bash
   ISAACSIM
   ```
   Check that you can launch the simulator from a python script (standalone mode)

   - Run the bundled python interpreter and see if it prints on the terminal "Hello World."
   ```bash
   ISAACSIM_PYTHON -c "print('Hello World.')"
   ```

   - Run the python interpreter and check if we can run a script that starts the simulator and adds cubes to the world
   ```bash
   ISAACSIM_PYTHON ${ISAACSIM_PATH}/standalone_examples/api/omni.isaac.core/add_cubes.py
   ```

   If you were unable to run the commands above successfuly, then something is incorrectly configured. Please do not proceed with this installation until you have everything setup correctly.


   > [!NOTE]
   > Secondary screens could freeze due to a bug in Nvidia driver. Disconnect and reconnect the HDMI
   > If Isaac sim failed to launch please post the log in issues section

6. Attach to docker in new terminal
   ```bash
   docker exec -it isaac-sim-ros2 bash
   ```

<!-- 7. Build ros2 source codes
   ```bash
   colcon build && source install/setup.bash
   ```

8. Launch the package

   7.1. For Mobile Robot
   - To launch simulator
   ```bash
   ros2 run isaac_ros2_scripts launcher
   ``` -->





> [!NOTE]
> For the first time, launching Isaac Sim takes a very long time.
> Isaac Sim must be fully launched to spawn the robot.
