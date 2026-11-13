#!/usr/bin/env bash
# Mocap teleop (rebel_demo on the mocap PC, ROS domain 12 over Wi-Fi) -> physical ReBeL through the igus ROS driver.
# Start the driver first (other terminal):  ros2 launch real_robot/dual_arm_driver.launch.py arms:=left
#   real_robot/run_mocap_follower.sh --arm left [--dry-run] [--mocap-domain 12]
# Then type `e` + Enter to enable, `d` to disable, `h` to home (rebel_demo ~/home), `q` to quit (the arm glides to a stop
# first). After each recorded episode (saved or discarded) it homes by itself (--no-home-after-episode: off).
# Do not run the Quest teleop on the same arm at the same time (this node refuses to start if it sees it).
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/../ros_env.sh"  # robot side: private local ROS graph (domain 42, localhost)
source /opt/ros/humble/setup.bash
exec /usr/bin/python3 "$HERE/mocap_follower.py" "$@"
