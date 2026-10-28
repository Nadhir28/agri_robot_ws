#!/bin/bash
# Start "gz sim <args>" so that Gazebo never outlives its launch file.
#
# Why: `gz sim` is a Ruby wrapper that starts separate "gz sim server" and
# "gz sim gui" processes. When ros2 launch was stopped, the server could keep
# running in the background, and the next launch then had two servers
# fighting over the same topics ("Gazebo GUI is not responding", bridge crashes).
#
# What this script does:
#  1. Stops leftover Gazebo processes in the SAME Gazebo partition
#     (GZ_PARTITION). They would clash with the new simulation anyway.
#     Simulations in other partitions (e.g. run_tests.sh) are not touched.
#  2. Starts gz sim in its own process group.
#  3. Stops that whole group when this script ends: on Ctrl+C / SIGTERM, on
#     normal exit, and - through a watchdog - even if this script is killed
#     with SIGKILL.

# Signals that were ignored when bash started cannot be trapped (this happens
# when the launch runs in the background of a script). Restart once with
# default signal handling so Ctrl+C / SIGTERM always reach the traps below.
if [ -z "${GZ_WRAPPER_SIGNALS_RESET:-}" ]; then
    export GZ_WRAPPER_SIGNALS_RESET=1
    exec env --default-signal=INT,TERM,HUP bash "$0" "$@"
fi

MY_PART="${GZ_PARTITION:-}"

# ---- 1. leftovers in the same partition
STALE=()
for pid in $(pgrep -u "$(id -u)" -f '^gz sim'); do
    part=$(tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null | sed -n 's/^GZ_PARTITION=//p')
    [ "$part" = "$MY_PART" ] && STALE+=("$pid")
done
if [ ${#STALE[@]} -gt 0 ]; then
    echo "[gz_sim_wrapper] stopping ${#STALE[@]} leftover Gazebo process(es) from an earlier run: ${STALE[*]}"
    kill -INT "${STALE[@]}" 2>/dev/null
    for _ in 1 2 3; do
        alive=0; for p in "${STALE[@]}"; do kill -0 "$p" 2>/dev/null && alive=1; done
        [ $alive -eq 0 ] && break; sleep 1
    done
    kill -9 "${STALE[@]}" 2>/dev/null
fi

# ---- 2. gz sim in its own process group
setsid gz sim "$@" &
GZ=$!

stop_group() {   # $1 = process group id
    kill -INT -- -"$1" 2>/dev/null || return 0
    for _ in 1 2 3; do
        kill -0 -- -"$1" 2>/dev/null || return 0
        sleep 1
    done
    kill -9 -- -"$1" 2>/dev/null
    return 0
}

# ---- 3a. watchdog in its own session: survives a SIGKILL of this script
#          (or of the whole launch process group) and then stops Gazebo.
WRAPPER=$$
setsid bash -c "
    while kill -0 $WRAPPER 2>/dev/null && kill -0 -- -$GZ 2>/dev/null; do sleep 1; done
    $(declare -f stop_group)
    stop_group $GZ
" > /dev/null 2>&1 < /dev/null &

# ---- 3b. normal shutdown paths
trap 'stop_group $GZ; exit 0' INT TERM HUP
trap 'stop_group $GZ' EXIT

wait "$GZ"
