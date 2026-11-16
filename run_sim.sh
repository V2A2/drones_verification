#!/bin/bash
#
# Runs the whole simulation (demo.mod: the drones and their random walk missions), optionally with the reachable set
# viewer. Default command of the Docker image; also runs on the host (terminal set up as in
# "Option 1: Local machine > Run the simulation" in the README).
#
# Ctrl+C / SIGTERM (docker stop) closes the viewer, then stops the simulation.
#
# Environment (optional):
#   SIM_VERIFIER  reachability checker, ps (ProbStar) or fl (face lifting) (default: ps)
#   SIM_VIZ       1 starts the reachable set viewer of SIM_VERIFIER (reach_viz), needs DISPLAY (default: 0). Closing
#                 its window does not stop the simulation.
#
# The processes run in tmux windows of the simd-<pid> sessions: `tmux ls`, then `tmux attach -t simd-<pid>`.

set -u

cd "$(dirname "$(readlink -f "$0")")"

verifier=${SIM_VERIFIER:-ps}
viz=${SIM_VIZ:-0}

sim_pid=
viz_pid=

# Closes the viewer, stops the simulation (simd shuts its processes down on SIGTERM), and exits with $1.
# The viewer gets SIGINT: rqt closes its window on it and saves the plugin settings (the added topics); on SIGTERM it
# would die without saving.
stop() {
    trap '' INT TERM
    if [ -n "$viz_pid" ] && kill -0 "$viz_pid" 2>/dev/null; then
        kill -INT "$viz_pid" 2>/dev/null
        wait "$viz_pid"
    fi
    if kill -0 "$sim_pid" 2>/dev/null; then
        kill -TERM "$sim_pid" 2>/dev/null
        wait "$sim_pid"
    fi
    exit "$1"
}
trap 'stop 130' INT
trap 'stop 143' TERM

# Starts the reachable set viewer of the verifier in the background. It runs outside the conda env (as in the README):
# its rqt/Qt dependencies are those of the system Python, and `#!/usr/bin/env python3` would pick conda's Python.
start_viz() {
    local pkg=probstar_reach
    [ "$verifier" = fl ] && pkg=fl_reach
    if [ -z "${DISPLAY:-}" ]; then
        echo "[run_sim] SIM_VIZ=1 but DISPLAY is not set, not starting the viewer" >&2
        return
    fi
    local path
    path=$(printf %s "$PATH" | tr : '\n' | grep -Ev 'conda|miniforge|mambaforge' | paste -sd:)
    # The workspace's setup.sh also sources /opt/ros/jazzy, so the calling shell needs no ROS setup (as the modules).
    # The executable is run directly, not with `ros2 run`, so that viz_pid is the rqt process itself (see stop).
    # rqt keeps its settings, the added topics included, in ~/.config/ros.org/rqt_gui.ini.
    env -u CONDA_PREFIX -u CONDA_DEFAULT_ENV -u CONDA_SHLVL PATH="$path" \
        bash -c '. ros2_ws/install/setup.sh && exec "$(ros2 pkg prefix "$1")/lib/$1/reach_viz"' _ "$pkg" &
    viz_pid=$!
    echo "[run_sim] reachable set viewer started ($pkg)"
}

# Background jobs of this non-interactive shell ignore SIGINT, so Ctrl+C reaches only this script and the shutdown
# order above is kept
python3 simd/simd.py sim demo.mod -c verifier="$verifier" &
sim_pid=$!
echo "[run_sim] simulation started (verifier $verifier)"

if [ "$viz" = 1 ]; then
    start_viz
fi

wait "$sim_pid"
code=$?
echo "[run_sim] simd exited with code $code" >&2
stop "$code"
