#!/usr/bin/env bash
# One-command launcher for the rebel_mocap_teleop commands. Run it from any folder, in any terminal:
#   ~/dual_arm_control/src/rebel_mocap_teleop/scripts/run.sh <command> [extra launch args]
#
#   real       teleop for the REAL robot, both arms (the "best known command" of the README): start pose from the
#              robot's /joint_states; stale_timeout 0.5 s for the jittery mocap network
#   isaac      teleop for Isaac Sim, both arms (Isaac must be playing, started with the same ROS_DOMAIN_ID)
#   real-ext   like real, but the robot PC publishes the arm TF on this domain (its own robot_state_publisher / MoveIt):
#              no second model and no fake_state here (start pose from the robot's /joint_states, as with real).
#              robot_ypr 0: the robot PC's world IS the left arm base (our model has the bases turned -90 deg)
#   ui         the GUI: start/stop driver + teleop, status lights, engage/pause buttons, settings profiles (SPACE pauses)
#   keyboard   SPACE = pause/resume both arms, l / r = left / right arm only, q = quit (own terminal)
#   calibrate  save the open-hand pose of both grippers (hold both hands open, launch must be running)
#              keyboard and calibrate use the arms the running launch has: both, or one with rig:=left / right
#   lag        how far the real arm lags behind teleop's commands (tools/lag_probe.py: passive, publishes nothing;
#              run it in its own terminal while you move the hand), e.g. run.sh lag --arm right
#   driver     start the OptiTrack (mocap4r2) driver
#   activate   activate the OptiTrack driver (after "Configured!" in the driver terminal)
#   build      colcon build of rebel_demo from the workspace root
#   env        only print what would be sourced (checks the setup)
#
# Extra arguments are appended to the launch line and override the defaults, e.g.
#   run.sh real scale_trans:=0.5 max_joint_speed_deg:=15
# Settings by environment variable: ROS_DOMAIN_ID (default 11), WS (default ~/dual_arm_control), MOCAP_WS.
set -eo pipefail

WS="${WS:-$HOME/dual_arm_control}"
MOCAP_WS="${MOCAP_WS:-$HOME/mocap_ws}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-11}"

usage() { sed -n '2,/^[^#]/{/^#/p;}' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; }

cmd="${1:-}"
[ -n "$cmd" ] || { usage; exit 1; }
shift || true
case "$cmd" in -h|--help|help) usage; exit 0 ;; esac

# conda's python breaks colcon (No module named 'catkin_pkg'); ros2 itself uses /usr/bin/python3, so drop conda from PATH
PATH="$(echo "$PATH" | tr ':' '\n' | grep -v 'miniconda\|anaconda\|/conda' | paste -sd:)"
export PATH

# the shell setup scripts are not safe with 'set -u', so source them with it off
source_ws() { [ -f "$1" ] && { set +u; source "$1"; return 0; } || { echo "run.sh: missing $1" >&2; return 1; }; }
source_ws /opt/ros/humble/setup.bash
if [ -f "$MOCAP_WS/install/setup.bash" ]; then source_ws "$MOCAP_WS/install/setup.bash"
else echo "run.sh: warning - $MOCAP_WS not found: mocap_tf will fail (mocap4r2_msgs)" >&2; fi
if [ "$cmd" != build ]; then
  [ -f "$WS/install/setup.bash" ] || { echo "run.sh: $WS/install/setup.bash missing - run: run.sh build" >&2; exit 1; }
  source_ws "$WS/install/setup.bash"
fi

echo "[run.sh] $cmd | ROS_DOMAIN_ID=$ROS_DOMAIN_ID | workspace $WS" >&2

# nodes of the running launch, comma-separated: rig:=both -> /<name>_left,/<name>_right; rig:=left / right -> /<name>
running_nodes() { ros2 node list --no-daemon --spin-time 2 2>/dev/null | grep -xE "/$1(_left|_right)?" | sort | paste -sd, || true; }

case "$cmd" in
  real)
    exec ros2 launch rebel_demo teleop.launch.py target:=isaac rig:=both rviz:=false scale_trans:=1.0 cutoff_hz:=6.0 \
      left_hand_frame:=2 left_gripper_frame:=4 \
      right_hand_frame:=1 right_gripper_frame:=3 gripper_mode:=orientation \
      orientation:=semi max_joint_speed_deg:=45 workspace_max:="0.7 0.7 1.4" \
      robot_ypr:="1.5708 0 0" home_deg:="0 0 60 0 50 0" stale_timeout:=0.5 \
      use_sim_time:=false fake_state:=true robot_joint_states_topic:=/joint_states require_joint_states:=true \
      max_lead_deg:=20 rot_axis_sign:="-1 1 1" limit_follow:=true "$@"
    ;;
  real-ext)
    exec ros2 launch rebel_demo teleop.launch.py target:=isaac rig:=both rviz:=false scale_trans:=1.0 cutoff_hz:=6.0 \
      left_hand_frame:=2 left_gripper_frame:=4 right_hand_frame:=1 right_gripper_frame:=3 gripper_mode:=orientation \
      orientation:=semi max_joint_speed_deg:=45 workspace_max:="0.7 0.7 1.4" \
      robot_ypr:="1.5708 0 0" home_deg:="0 0 60 0 50 0" stale_timeout:=0.5 \
      use_sim_time:=false robot_tf:=external robot_joint_states_topic:=/joint_states require_joint_states:=true \
      max_lead_deg:=20 rot_axis_sign:="-1 1 1" limit_follow:=true "$@"
    ;;
  isaac)
    # no use_sim_time / fake_state: Isaac's bridge provides /clock and the joint states
    exec ros2 launch rebel_demo teleop.launch.py target:=isaac rig:=both rviz:=false scale_trans:=1.0 cutoff_hz:=6.0 \
      left_hand_frame:=2 left_gripper_frame:=4 \
      right_hand_frame:=1 right_gripper_frame:=3 gripper_mode:=orientation \
      orientation:=true max_joint_speed_deg:=45 workspace_max:="0.7 0.7 1.4" \
      robot_ypr:="1.5708 0 0" home_deg:="0 0 60 0 50 0" "$@"
    ;;
  ui)
    exec ros2 run rebel_demo rebel_ui "$@"
    ;;
  keyboard)
    nodes="$(running_nodes rebel_teleop)"
    nodes="${nodes:-/rebel_teleop_left,/rebel_teleop_right}"      # launch not up yet: assume both arms
    echo "[run.sh] keyboard controls $nodes" >&2
    exec ros2 run rebel_demo teleop_keyboard --ros-args -p teleop:="$nodes" "$@"
    ;;
  calibrate)
    nodes="$(running_nodes rebel_gripper)"
    [ -n "$nodes" ] || { echo "run.sh: no gripper node on ROS_DOMAIN_ID=$ROS_DOMAIN_ID - start the launch first (same domain)" >&2; exit 1; }
    for n in ${nodes//,/ }; do ros2 service call "$n/calibrate" std_srvs/srv/Trigger; done
    ;;
  lag)
    exec python3 "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../tools/lag_probe.py" "$@"
    ;;
  driver)
    exec ros2 launch mocap4r2_optitrack_driver optitrack2.launch.py "$@"
    ;;
  activate)
    exec ros2 lifecycle set /mocap4r2_optitrack_driver_node activate
    ;;
  build)
    cd "$WS"
    exec colcon build --packages-select rebel_demo "$@"
    ;;
  env)
    echo "ROS_DISTRO=${ROS_DISTRO:-?}  ROS_DOMAIN_ID=$ROS_DOMAIN_ID  RMW=${RMW_IMPLEMENTATION:-default}"
    printf 'rebel_demo     : '; ros2 pkg prefix rebel_demo || true
    printf 'mocap4r2_msgs  : '; ros2 pkg prefix mocap4r2_msgs || true
    ;;
  *)
    echo "run.sh: unknown command '$cmd'" >&2; usage; exit 1
    ;;
esac
