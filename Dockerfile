# syntax=docker/dockerfile:1
#
# Self-contained image of the wm_drones simulation: PX4 v1.17 SITL, Gazebo Harmonic (server), ROS 2 Jazzy with the
# built ros2_ws, Micro-XRCE-DDS-Agent, the `sim` conda env (StarV, simd) and tmux.
#
# The local sources are copied in (not cloned): several of them carry local changes or are not in any git repo.
#
#   docker build -t wm_drones/sim .                            # from the repository root
#   docker run -it --rm --name wm_sim --network host --ipc host wm_drones/sim        # runs run_sim.sh
#   docker run -it --rm --name wm_sim --network host --ipc host wm_drones/sim bash   # shell
#   docker run -it --rm --name wm_sim --network host --ipc host -e SIM_VIZ=1 \
#       -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix wm_drones/sim                   # + reachable set viewer (xhost +local:root)
#
# Stages: base (system deps, conda env) -> px4-build (make px4_sitl, needs PX4's .git) and ros-manifests (package.xml
# files, for rosdep caching) -> final (PX4 runtime files, built ros2_ws, simd, mission scripts and modules).
#
# PX4 and ros2_ws are built with the system Python (as in the README); the `sim` conda env is the runtime environment.

ARG ROS_DISTRO=jazzy

# ---------------------------------------------------------------------------------------------------------------------
FROM ros:${ROS_DISTRO}-ros-base AS base

ENV DEBIAN_FRONTEND=noninteractive \
    RUNS_IN_DOCKER=true \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    TZ=UTC

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git tmux wget ca-certificates \
        python3-colcon-common-extensions python3-rosdep \
        libglpk-dev libgmp3-dev \
    && rm -rf /var/lib/apt/lists/*

# PX4 toolchain + Gazebo Harmonic (gz-harmonic), as in the README. Only the setup files are copied so that this layer
# stays cached when the PX4 sources change. --no-nuttx: SITL only, no hardware toolchain.
# numpy stays at the Debian version the ROS base image ships (1.26.4, as on the host): requirements.txt does not cap
# matplotlib/pandas/contourpy, so pip would take their newest releases, which need numpy 2.x, and fail replacing the
# Debian numpy ("Cannot uninstall numpy ... installed by debian"). With the constraint pip picks compatible releases.
COPY PX4-Autopilot/Tools/setup/ubuntu.sh PX4-Autopilot/Tools/setup/requirements.txt /tmp/px4_setup/
RUN echo "numpy==$(python3 -c 'import numpy; print(numpy.__version__)')" > /tmp/px4_setup/constraints.txt \
    && PIP_CONSTRAINT=/tmp/px4_setup/constraints.txt bash /tmp/px4_setup/ubuntu.sh --no-nuttx \
    && rm -rf /tmp/px4_setup /var/lib/apt/lists/*

# Micro-XRCE-DDS-Agent (PX4 <-> ROS 2 bridge), same version as on the host
RUN git clone --depth 1 -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/agent \
    && cmake -S /tmp/agent -B /tmp/agent/build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /tmp/agent/build -j"$(nproc)" \
    && cmake --install /tmp/agent/build \
    && ldconfig /usr/local/lib \
    && rm -rf /tmp/agent

# conda env "sim" from sim_environment.yml (StarV + simd dependencies). The env file is copied on its own: this is the
# slowest layer to rebuild.
RUN wget -q https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh \
        -O /tmp/miniforge.sh \
    && bash /tmp/miniforge.sh -b -p /opt/conda \
    && rm /tmp/miniforge.sh
COPY sim_environment.yml /tmp/sim_environment.yml
RUN /opt/conda/bin/conda env create -f /tmp/sim_environment.yml \
    && /opt/conda/bin/conda clean -afy \
    && rm /tmp/sim_environment.yml

RUN git config --global --add safe.directory '*'

# ---------------------------------------------------------------------------------------------------------------------
FROM base AS px4-build

# Including .git: the PX4 build runs git describe for its version and checks the submodules
COPY PX4-Autopilot /wm_drones/PX4-Autopilot
WORKDIR /wm_drones/PX4-Autopilot
# Built without ROS sourced, so px4 links against the system Gazebo Harmonic libraries. Also generates
# build/px4_sitl_default/rootfs/gz_env.sh with the /wm_drones paths, which gazebo.mod sources.
RUN make px4_sitl \
    && rm -rf .git \
    && find build/px4_sitl_default \( -name '*.o' -o -name '*.a' -o -name '*.obj' \) -delete

# ---------------------------------------------------------------------------------------------------------------------
FROM base AS ros-manifests

# Only the package manifests, so that the rosdep layer is rebuilt only when the dependencies change
COPY ros2_ws/src /src
RUN mkdir /manifests && cd /src \
    && find . \( -name package.xml -o -name COLCON_IGNORE -o -name AMENT_IGNORE \) -exec cp --parents {} /manifests \;

# ---------------------------------------------------------------------------------------------------------------------
FROM base

WORKDIR /wm_drones

COPY --from=px4-build /wm_drones/PX4-Autopilot /wm_drones/PX4-Autopilot

# Dependencies of every package in ros2_ws/src
COPY --from=ros-manifests /manifests /wm_drones/ros2_ws/src
RUN cd ros2_ws \
    && apt-get update \
    && rosdep update --rosdistro "${ROS_DISTRO}" \
    && rosdep install -y -r --from-paths src --ignore-src --rosdistro "${ROS_DISTRO}" \
    && rm -rf /var/lib/apt/lists/*

# Whole workspace, built with the system Python (conda is not on PATH yet), as the README requires
COPY ros2_ws/src /wm_drones/ros2_ws/src
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && cd ros2_ws \
    && colcon build --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log

COPY simd /wm_drones/simd
COPY mission_scripts /wm_drones/mission_scripts
COPY *.mod sim_environment.yml README.md /wm_drones/
# simd modules of the ROS 2 processes (pixhawk bridge, aerostack2, verifiers), referenced as ros2_ws/<name> by drone.mod
COPY ros2_ws/*.mod /wm_drones/ros2_ws/
# Default command: drones.mod, then drones_random_walk.mod after a warm-up
COPY --chmod=755 run_sim.sh /wm_drones/run_sim.sh

# The sim conda env is the environment of every process in the container: shells, `docker exec`, simd, and the nodes
# started with `#!/usr/bin/env python3` (the ProbStar verifier imports StarV from it)
ENV PATH=/opt/conda/envs/sim/bin:/opt/conda/bin:${PATH} \
    CONDA_DEFAULT_ENV=sim \
    CONDA_PREFIX=/opt/conda/envs/sim

# Fast DDS over UDP only. With --network host --ipc host, Fast DDS would use its shared-memory transport between the
# container and ROS 2 tools on the host; its /dev/shm files belong to the container's root, so host tools list the
# topics (UDP discovery) but receive no data. Override with `docker run -e FASTDDS_BUILTIN_TRANSPORTS=DEFAULT`.
ENV FASTDDS_BUILTIN_TRANSPORTS=UDPv4

# ROS 2 + workspace for the container's command and for `docker exec -it <container> bash` shells
COPY --chmod=755 <<'EOF' /entrypoint.sh
#!/bin/bash
source /opt/ros/${ROS_DISTRO}/setup.bash
source /wm_drones/ros2_ws/install/setup.bash
exec "$@"
EOF
RUN printf 'source /opt/ros/%s/setup.bash\nsource /wm_drones/ros2_ws/install/setup.bash\n' "${ROS_DISTRO}" \
        >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
# `docker run ... wm_drones/sim bash` for a shell instead
CMD ["/wm_drones/run_sim.sh"]
