# MISSION X rover — Docker (ROS 2 Jazzy)

The image (`FROM ros:jazzy-ros-base`) contains Nav2, slam_toolbox,
robot_localization, xacro, robot_state_publisher, joint_state_publisher,
teleop_twist_keyboard and the colcon-built `ros2_ws/src` workspace. It is
meant for the **simulator** (`rover_sim`), SLAM / navigation tests and the
headless evaluation. The real rover runs natively on the Raspberry Pi 4
(see `docs/ROS2.md`).

Build context is the repository root (`docker/Dockerfile.dockerignore` keeps it
small). All commands from the repository root:

```bash
docker compose -f docker/docker-compose.yml build

# simulator + EKF + slam_toolbox (host network, ROS_DOMAIN_ID=42)
docker compose -f docker/docker-compose.yml up sim-slam
# simulator + EKF + AMCL (maps/lab.yaml) + Nav2
docker compose -f docker/docker-compose.yml up sim-nav

# headless exploration run -> docker/eval_output/run_<timestamp>/
EVAL_DURATION=900 docker compose -f docker/docker-compose.yml run --rm eval
```

`eval` runs `scripts/run_sim_eval.sh`: sim + EKF + SLAM + Nav2 + frontier
explorer (autostart), records a rosbag (`/ground_truth/odom`,
`/odometry/filtered`, `/tf`, `/tf_static`, `/wheel/odom`, `/sim/collisions`,
`/explorer/status`), saves the final map with `nav2_map_server map_saver_cli`
and writes `summary.txt`. Variables: `EVAL_DURATION` (s), `EVAL_RTF`
(real-time factor), `EVAL_SEED`, `EVAL_WORLD`.

Visualisation: the image has no GUI. On a Linux host with ROS 2 Jazzy, run
`ROS_DOMAIN_ID=42 rviz2 -d ros2_ws/src/rover_bringup/rviz/rover_nav.rviz`
while `sim-slam` / `sim-nav` are up (host networking + `ipc: host`), and drive
with `ROS_DOMAIN_ID=42 ros2 run teleop_twist_keyboard teleop_twist_keyboard`.
Host networking is a Linux feature; on Docker Desktop use `eval` or a shell in
the container (`docker compose -f docker/docker-compose.yml run --rm sim-slam bash`).
