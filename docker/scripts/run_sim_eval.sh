#!/usr/bin/env bash
# Headless MISSION X simulation evaluation (docker compose service `eval`).
#
#   rover_sim + EKF + slam_toolbox + Nav2 + frontier explorer (autostart)
#   -> rosbag of /ground_truth/odom, /odometry/filtered, /tf (+ /tf_static,
#      /wheel/odom, /sim/collisions, /explorer/status)
#   -> final map saved with nav2_map_server map_saver_cli
#   -> summary.txt (explorer status, collisions, final poses)
#
# Environment:
#   EVAL_DURATION  max run time in seconds of wall time (default 900)
#   EVAL_OUTPUT    output root (default /eval_output)
#   EVAL_WORLD     rover_core.sim world YAML (default: rover_sim worlds/lab.yaml)
#   EVAL_RTF       simulator real-time factor (default 1.0)
#   EVAL_SEED      simulator noise seed (default 0)
set -eo pipefail

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source /ws/install/setup.bash

DURATION="${EVAL_DURATION:-900}"
OUT_ROOT="${EVAL_OUTPUT:-/eval_output}"
RTF="${EVAL_RTF:-1.0}"
SEED="${EVAL_SEED:-0}"
WORLD="${EVAL_WORLD:-$(ros2 pkg prefix rover_sim)/share/rover_sim/worlds/lab.yaml}"
RUN="${OUT_ROOT}/run_$(date +%Y%m%d_%H%M%S)"
mkdir -p "${RUN}"
echo "[eval] output=${RUN} duration=${DURATION}s rtf=${RTF} seed=${SEED} world=${WORLD}"

LAUNCH_PID=""
BAG_PID=""

stop_pid() {   # SIGINT (clean shutdown, bag finalisation), then SIGTERM
  local pid="$1" name="$2"
  [ -n "${pid}" ] || return 0
  kill -0 "${pid}" 2>/dev/null || return 0
  kill -INT "${pid}" 2>/dev/null || true
  for _ in $(seq 1 40); do
    kill -0 "${pid}" 2>/dev/null || return 0
    sleep 0.5
  done
  echo "[eval] ${name} still running, sending SIGTERM"
  kill -TERM "${pid}" 2>/dev/null || true
}

cleanup() {
  stop_pid "${BAG_PID}" "ros2 bag"
  stop_pid "${LAUNCH_PID}" "ros2 launch"
  wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM

field() {      # field <topic> <type> <field> : print one field of the next message
  timeout 5 ros2 topic echo --once --field "$3" "$1" "$2" 2>/dev/null | head -n 1 || true
}

# ------------------------------------------------------------------ launch
ros2 launch rover_bringup sim.launch.py slam:=true nav:=true explore:=true \
    explore_autostart:=true rviz:=false world:="${WORLD}" \
    real_time_factor:="${RTF}" seed:="${SEED}" > "${RUN}/launch.log" 2>&1 &
LAUNCH_PID=$!

echo "[eval] waiting for /map from slam_toolbox ..."
for _ in $(seq 1 60); do
  if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    echo "[eval] launch exited early - see ${RUN}/launch.log"
    exit 1
  fi
  if [ -n "$(field /map nav_msgs/msg/OccupancyGrid info.width)" ]; then
    break
  fi
  sleep 2
done

ros2 bag record -o "${RUN}/bag" \
    /ground_truth/odom /odometry/filtered /wheel/odom /tf /tf_static \
    /sim/collisions /explorer/status > "${RUN}/bag.log" 2>&1 &
BAG_PID=$!

# ------------------------------------------------------------------ monitor
START=${SECONDS}
STATUS="?"
COLL="?"
while (( SECONDS - START < DURATION )); do
  sleep 10
  if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    echo "[eval] launch exited early - see ${RUN}/launch.log"
    break
  fi
  STATUS="$(field /explorer/status std_msgs/msg/String data)"
  COLL="$(field /sim/collisions std_msgs/msg/UInt32 data)"
  echo "[eval] t=$((SECONDS - START))s explorer=${STATUS:-?} collisions=${COLL:-?}"
  if [[ "${STATUS}" == *COMPLETE* ]]; then
    break
  fi
done
ELAPSED=$((SECONDS - START))

# ------------------------------------------------------------------ results
echo "[eval] saving map"
ros2 run nav2_map_server map_saver_cli -f "${RUN}/map" --ros-args -p use_sim_time:=true \
    > "${RUN}/map_saver.log" 2>&1 || echo "[eval] map_saver_cli failed (see map_saver.log)"

{
  echo "MISSION X simulation evaluation"
  echo "date:            $(date -Iseconds)"
  echo "world:           ${WORLD}"
  echo "seed / rtf:      ${SEED} / ${RTF}"
  echo "wall time [s]:   ${ELAPSED} (limit ${DURATION})"
  echo "explorer status: ${STATUS:-?}"
  echo "collisions:      ${COLL:-?}"
  echo "ground truth (world frame):"
  timeout 5 ros2 topic echo --once --field pose.pose.position /ground_truth/odom nav_msgs/msg/Odometry 2>/dev/null | head -n 3 || true
  echo "EKF /odometry/filtered (odom frame, origin = start pose):"
  timeout 5 ros2 topic echo --once --field pose.pose.position /odometry/filtered nav_msgs/msg/Odometry 2>/dev/null | head -n 3 || true
} | tee "${RUN}/summary.txt"

stop_pid "${BAG_PID}" "ros2 bag"
BAG_PID=""
ros2 bag info "${RUN}/bag" > "${RUN}/bag_info.txt" 2>&1 || true
echo "[eval] done: $(ls "${RUN}" | tr '\n' ' ')"
