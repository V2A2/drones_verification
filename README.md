# wm_drones

Multi-drone simulation with online reachability-based collision checking: every drone runs a reachable-set verifier
(ProbStar or face lifting) and exchanges its reachable sets with the other drones over a shared radio topic.

* Operating system: Ubuntu 24.04
* ROS 2 Jazzy, PX4 v1.17.0, Gazebo Harmonic
* Python 3.12 (conda environment `sim`)

There are two ways to run it:

* [Option 1: Local machine](#option-1-local-machine): install everything on the host.
* [Option 2: Docker](#option-2-docker): everything except QGroundControl comes in one image.

## Option 1: Local machine

### Set up the environment

#### 1) PX4 dependencies

Install the PX4 toolchain. On Ubuntu 24.04 this script also installs Gazebo Harmonic:

```bash
bash PX4-Autopilot/Tools/setup/ubuntu.sh
```

Build the PX4 SITL target. Besides the `px4` binary, this generates `build/px4_sitl_default/rootfs/gz_env.sh`,
which `gazebo.mod` sources to find the PX4 models, worlds and plugins:

```bash
cd PX4-Autopilot
make px4_sitl
```

#### 2) ROS 2 dependencies

Install ROS 2 Jazzy Desktop by following the [official instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html),
together with colcon and rosdep:

```bash
sudo apt install ros-jazzy-desktop python3-colcon-common-extensions python3-rosdep
```

Install the package dependencies and build the workspace. Build in a terminal **without** the conda environment
activated, so that the ROS 2 packages use the system Python:

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
sudo rosdep init   # only once per machine
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

#### 3) conda environment

Install the system libraries needed to build the `glpk` wheel:

```bash
sudo apt install libglpk-dev libgmp3-dev
```

Create the environment. It contains simd's dependencies and StarV, installed from GitHub at commit `deb3953`:

```bash
conda env create -f sim_environment.yml
```

StarV uses Gurobi. Set up a Gurobi license as described in the
[StarV README](https://github.com/V2A2/StarV#gurobi-installation-on-ubuntu).

#### 4) QGroundControl

QGroundControl is required. Install it by following the
[official instructions](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html#ubuntu)
(download the AppImage and make it executable).

#### 5) Micro-XRCE-DDS-Agent

The agent bridges PX4 and ROS 2. Build it from source; `micro-xrce-dds-agent.mod` calls it as `MicroXRCEAgent`:

```bash
git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

### Run the simulation

Start QGroundControl, then open a terminal in the project root directory, run the following commands:
```bash
conda activate sim
./run_sim.sh
```

This starts the simulation defined in `demo.mod`: four drones, each with its state estimate, its reachability
verifier and a random walk mission. `run_sim.sh` has several options, given as environment variables:

| Variable       | Default | Meaning                                                                       |
|----------------|---------|-------------------------------------------------------------------------------|
| `SIM_VERIFIER` | `ps`    | reachability checker: `ps` (ProbStar) or `fl` (face lifting)                  |
| `SIM_VIZ`      | `0`     | `1` starts the reachable set viewer of `SIM_VERIFIER` (needs `DISPLAY`)       |

For example, face lifting with the viewer:

```bash
SIM_VERIFIER=fl SIM_VIZ=1 ./run_sim.sh
```

## Option 2: Docker

### Set up the environment

The image contains everything from Option 1 except QGroundControl: PX4 SITL (built), Gazebo Harmonic (server only),
ROS 2 Jazzy with the built `ros2_ws`, Micro-XRCE-DDS-Agent, the `sim` conda environment and tmux. The sources of this
folder are copied into the image, local changes included, so rebuild the image after changing them.

Build it from the root directory:

```bash
docker build -t wm_drones/sim .
```

On the host, install only QGroundControl (Option 1, step 4). For the reachable set viewer, the host needs an X server
(any Ubuntu desktop session).

### Run the simulation

Start QGroundControl on the host, then start a container. By default it runs `run_sim.sh` (see
[Run the simulation](#run-the-simulation) of Option 1). `Ctrl+C` or `docker stop wm_sim` stops the missions, then the
drones:

```bash
docker run -it --rm --name wm_sim --network host --ipc host wm_drones/sim
```

The [options](#run-the-simulation) of `run_sim.sh` are given with `-e`, for example `-e SIM_VERIFIER=fl -e SIM_VIZ=1`.

#### Reachable set viewer

The viewer is a window, so the container needs the host's X server. Allow local root clients once per login, then pass
`DISPLAY` and the X11 socket. Closing the viewer does not stop the simulation:

```bash
xhost +local:root
docker run -it --rm --name wm_sim --network host --ipc host \
  -e SIM_VERIFIER=fl -e SIM_VIZ=1 -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix wm_drones/sim
```

#### Gazebo GUI on the host

The Gazebo and PX4 modules use `GZ_PARTITION=px4_sim` and `GZ_IP=127.0.0.1` (see `gazebo.mod` and `px4_gz.mod`).
A Gazebo GUI on the host must use the same values to see the world:

```bash
GZ_PARTITION=px4_sim GZ_IP=127.0.0.1 gz sim -g
```
