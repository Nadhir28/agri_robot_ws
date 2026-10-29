#!/bin/bash
# Run the agri_ugv automated test suites, each in its own fresh headless
# simulation, and print a summary.
#
#   ros2 run agri_ugv_description run_tests.sh            # all suites
#   ros2 run agri_ugv_description run_tests.sh motion     # one suite
#   ros2 run agri_ugv_description run_tests.sh sensor orchard
#
# Suites:  motion  -> motion_test.py  in empty_flat     (Phase 1, robot driving)
#          orchard -> orchard_test.py in olive_orchard  (Phase 2, orchard driving)
#          sensor  -> sensor_test.py  in olive_orchard  (Phase 3 + Tasks 1-2, sensors)
#
# The tests use their own ROS domain and Gazebo partition, so a simulation you
# have open at the same time is not disturbed (and does not disturb the tests).
# Full logs: ~/.ros/agri_ugv_tests/<date-time>/
# Best results with no other Gazebo window open (the tests run faster).

set -u
SUITES=("$@")
[ ${#SUITES[@]} -eq 0 ] && SUITES=(motion orchard sensor rtk)

export ROS_DOMAIN_ID=${AGRI_TEST_DOMAIN:-87}
export GZ_PARTITION=agri_ugv_tests_$$
LOGDIR=~/.ros/agri_ugv_tests/$(date +%Y-%m-%d_%H-%M-%S)
mkdir -p "$LOGDIR"

declare -A WORLD=([motion]=empty_flat [orchard]=olive_orchard [sensor]=olive_orchard [rtk]=olive_orchard)
declare -A TEST=([motion]=motion_test.py [orchard]=orchard_test.py [sensor]=sensor_test.py [rtk]=rtk_test.py)

stop_sim() {
    kill -INT -- -"$1" 2>/dev/null
    for _ in 1 2 3 4 5 6 7 8; do kill -0 -- -"$1" 2>/dev/null || return; sleep 1; done
    kill -9 -- -"$1" 2>/dev/null
}

SUMMARY=()
ALL_OK=0
for s in "${SUITES[@]}"; do
    if [ -z "${WORLD[$s]:-}" ]; then
        echo "unknown suite '$s' (use: motion, orchard, sensor, rtk)"; exit 2
    fi
    echo
    echo "================ $s: ${TEST[$s]} in ${WORLD[$s]} ================"
    echo "starting simulation (headless)..."
    setsid ros2 launch agri_ugv_description sim.launch.py world:="${WORLD[$s]}" headless:=true \
        > "$LOGDIR/$s.sim.log" 2>&1 < /dev/null &
    PG=$!
    trap 'stop_sim $PG; exit 130' INT TERM

    READY=0
    # patient: on a loaded PC (e.g. a Gazebo window open) ros2 CLI itself starts slowly
    for _ in $(seq 1 18); do
        if timeout 10 ros2 topic echo --once /ground_truth/odom > /dev/null 2>&1; then READY=1; break; fi
        sleep 1
    done
    if [ $READY -eq 0 ]; then
        echo "simulation did not start, see $LOGDIR/$s.sim.log"
        SUMMARY+=("$s|NO SIM|-"); ALL_OK=1; stop_sim $PG; continue
    fi

    START=$(date +%s)
    ros2 run agri_ugv_description "${TEST[$s]}" 2>&1 \
        | sed -E 's/^\[INFO\] \[[0-9.]*\] \[[a-z_]*\]: //' | grep -v '^\[ros2run\]' | tee "$LOGDIR/$s.log"
    RC=${PIPESTATUS[0]}
    DUR=$(( $(date +%s) - START ))
    stop_sim $PG
    trap - INT TERM

    RESULT=$(grep -oE '[0-9]+/[0-9]+ checks passed' "$LOGDIR/$s.log" | tail -1)
    [ $RC -eq 0 ] && STATUS=PASS || { STATUS=FAIL; ALL_OK=1; }
    SUMMARY+=("$s|$STATUS ${RESULT:-no result}|${DUR}s")
done

echo
echo "======================= SUMMARY ======================="
printf "%-9s %-32s %s\n" suite result time
for line in "${SUMMARY[@]}"; do
    IFS='|' read -r a b c <<< "$line"; printf "%-9s %-32s %s\n" "$a" "$b" "$c"
done
echo "logs: $LOGDIR"
[ $ALL_OK -eq 0 ] && echo "ALL SUITES PASSED" || echo "SOME SUITES FAILED (see logs above)"
exit $ALL_OK
