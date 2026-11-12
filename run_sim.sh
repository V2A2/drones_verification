#!/bin/bash
#
# Runs the whole simulation: starts the drones (drones.mod), waits for them to come up, then starts the random walk
# missions (drones_random_walk.mod). Default command of the Docker image; also runs on the host (terminal set up as in
# "Option 1: Local machine > Run the simulation" in the README).
#
# Ctrl+C / SIGTERM (docker stop) stops the missions first, then the drones. If either simd exits, the other is stopped.
#
# Environment (optional):
#   SIM_WARMUP    seconds between starting the drones and the missions (default: 120)
#   SIM_VERIFIER  reachability checker, ps (ProbStar) or fl (face lifting) (default: ps)
#   SIM_SEED      random walk seed (default: 0)
#   SIM_VIZ       1 starts the reachable set viewer of SIM_VERIFIER (reach_viz), needs DISPLAY (default: 0). Closing
#                 its window does not stop the simulation.
#
# The processes run in tmux windows of the simd-<pid> sessions: `tmux ls`, then `tmux attach -t simd-<pid>`.

set -u

cd "$(dirname "$(readlink -f "$0")")"

warmup=${SIM_WARMUP:-120}
verifier=${SIM_VERIFIER:-ps}
seed=${SIM_SEED:-0}
viz=${SIM_VIZ:-0}

sim_pid=
walk_pid=
viz_pid=

# Closes the viewer, stops the missions, then the drones (simd shuts its processes down on SIGTERM), and exits with $1.
# The viewer gets SIGINT: rqt closes its window on it and saves the plugin settings (the added topics); on SIGTERM it
# would die without saving.
stop() {
    trap '' INT TERM
    if [ -n "$viz_pid" ] && kill -0 "$viz_pid" 2>/dev/null; then
        kill -INT "$viz_pid" 2>/dev/null
        wait "$viz_pid"
    fi
    for pid in $walk_pid $sim_pid; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -TERM "$pid" 2>/dev/null
            wait "$pid"
        fi
    done
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
python3 simd/simd.py sim drones.mod -c verifier="$verifier" &
sim_pid=$!

if [ "$viz" = 1 ]; then
    start_viz
fi

echo "[run_sim] drones started, starting the random walk missions in ${warmup}s"
# Slept in the background so that the traps run during the warm-up. Returns early if the drones' simd exits.
sleep "$warmup" &
sleep_pid=$!
wait -n "$sim_pid" "$sleep_pid"
kill "$sleep_pid" 2>/dev/null
if ! kill -0 "$sim_pid" 2>/dev/null; then
    wait "$sim_pid"
    code=$?
    echo "[run_sim] drones.mod exited with code $code during the warm-up" >&2
    exit "$code"
fi

python3 simd/simd.py sim drones_random_walk.mod -c seed="$seed" &
walk_pid=$!
echo "[run_sim] random walk missions started (seed $seed)"

wait -n "$sim_pid" "$walk_pid"
code=$?
echo "[run_sim] a simd instance exited with code $code, stopping the simulation" >&2
stop "$code"
