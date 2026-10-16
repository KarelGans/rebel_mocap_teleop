# rebel_mocap_teleop — hand teleoperation of igus ReBeL arms from OptiTrack mocap

Your hands (OptiTrack rigid bodies) drive one or two igus ReBeL 6-DoF arms with HIWIN XEG-32 grippers, in real time:
the hand pose is filtered, turned into joint angles by IK at 60 Hz, and published as **absolute joint positions** for
Isaac Sim or for your real robot's driver. A second rigid body per hand opens and closes the gripper.
The ROS 2 package is called `rebel_demo`; this repo folder is `rebel_mocap_teleop`.

```
Motive (Windows) --NatNet--> mocap4r2 OptiTrack driver --/rigid_bodies--> mocap_tf --TF (one frame per streaming ID)-->
   teleop   (hand: filter -> IK -> speed limit) -> arm joints 1-6        } joint_merger: ONE JointState per cycle
   gripper  (fingertip vs hand) ---------------> XEG-32 open / closed    }   --> /dual_arm/isaac_joint_commands
                                                                               --> Isaac Sim, or your robot driver
   /dual_arm/isaac_joint_states (Isaac, your driver, or fake_state) --> robot_state_publisher (TF of the rig)
```

| I want to... | Go to |
|---|---|
| install everything on a new PC | [1. Install](#1-install) |
| start teleop with a few commands | [Quick start](#quick-start) |
| drive the real robot / Isaac Sim | [2.2](#22-real-robot), [2.3](#23-isaac-sim) |
| look up an argument, a topic or a limit | [Reference](#reference) |
| fix an error message | [Troubleshooting](#troubleshooting) |

## Quick start
After the one-time [install](#1-install), `scripts/run.sh` does the sourcing (ROS, mocap workspace, this workspace,
`ROS_DOMAIN_ID=11`; conda is taken off `PATH`) and runs the command, from any folder. One terminal per line, in order:

| Terminal | Command (prefix `~/dual_arm_control/src/rebel_mocap_teleop/scripts/`) | Notes |
|---|---|---|
| 1 | `run.sh driver` | OptiTrack driver; wait for `Configured!`, then in any terminal `run.sh activate` |
| 2 | `run.sh real` (real robot) or `run.sh isaac` (Isaac Sim) | wait for `at HOME` from both arms; keep it open |
| 3 | `run.sh keyboard` | click this terminal to focus it; **SPACE** engages / pauses / resumes both arms, `l` / `r` one arm, `q` quits |
| once | `run.sh calibrate` | hold both hands open while it runs; repeat after every restart of the launch |

`run.sh help` lists everything (`build`, `env` check, ...). Extra arguments override the defaults:
`run.sh real scale_trans:=0.5 max_joint_speed_deg:=15`; another domain: `ROS_DOMAIN_ID=12 run.sh real`. Optional alias for
`~/.bashrc`: `alias rebel='~/dual_arm_control/src/rebel_mocap_teleop/scripts/run.sh'` (then `rebel real`, `rebel keyboard`).
The robot side (Isaac Sim, or your driver that reads `/dual_arm/isaac_joint_commands`) must be running too.
First run on a new robot or setup: `scale_trans:=0.5 max_joint_speed_deg:=15` and the e-stop in reach.

---

## 1. Install
### 1.1 Requirements
- Ubuntu 22.04 with ROS 2 Humble (what this was developed and tested on), **one** ROS distribution only. The code is plain
  rclpy and should also run on Ubuntu 24.04 + Jazzy (not re-tested). Never mix distributions in terminals or workspaces.
- `colcon`, `rosdep`, `xacro`, `numpy`; `robot_state_publisher`; `rviz2` (only for RViz); the robot model package
  `igus_rebel_description` (below); Isaac Sim (only for simulation).
- The mocap driver needs Linux (its NatNet library is Linux-only). RViz can run on another machine with this package built
  (same network, same `ROS_DOMAIN_ID`).

### 1.2 Mocap driver and Motive
The driver lives in a separate workspace (it provides `mocap4r2_msgs`, which `mocap_tf` needs even in Isaac mode):
```bash
source /opt/ros/$(ls /opt/ros)/setup.bash
sudo apt install -y python3-rosdep python3-vcstool
sudo rosdep init 2>/dev/null; rosdep update
mkdir -p ~/mocap_ws/src && cd ~/mocap_ws/src
git clone -b rolling https://github.com/MOCAP4ROS2-Project/mocap4ros2_optitrack.git
vcs import < mocap4ros2_optitrack/dependency_repos.repos
cd ~/mocap_ws && rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```
Use the `MOCAP4ROS2-Project` repository, not the `OptiTrack/mocap4ros2_optitrack` fork (its dependency file points to a
repository that does not exist). Then set the addresses in the driver config (only `MOTIVE_IP` needs your value):
```bash
MOTIVE_IP=192.168.1.10                       # the Motive PC (Motive: Edit -> Settings -> Streaming -> Local Interface)
LOCAL_IP=$(hostname -I | awk '{print $1}')   # this Linux PC
CFG=~/mocap_ws/src/mocap4ros2_optitrack/mocap4r2_optitrack_driver/config/mocap4r2_optitrack_driver_params.yaml
sed -i -E "s/^( *connection_type:).*/\1 \"Multicast\"/; s/^( *server_address:).*/\1 \"$MOTIVE_IP\"/; \
  s/^( *local_address:).*/\1 \"$LOCAL_IP\"/; s/^( *multicast_address:).*/\1 \"239.255.42.99\"/" "$CFG"
cd ~/mocap_ws && colcon build --symlink-install
```
`connection_type` must match Motive's Transmission Type (Multicast here; use Unicast in both if needed).

**Motive (Windows):** Edit -> Settings -> Streaming: *Broadcast Frame Data* on, *Local Interface* = the Motive PC's IP on the
shared network, *Transmission Type* = the same as the YAML, *Rigid Bodies* on. Windows firewall: network profile Private,
allow Motive. All this repo needs from Motive is each rigid body's **Streaming ID** (Properties -> Streaming ID); the body's
name does not matter. Defaults: palm (moves the arm) = ID 1, fingertip (gripper) = ID 2; with two hands, left 1/2, right 3/4.

### 1.3 This package, the robot model, and the build
```bash
mkdir -p ~/dual_arm_control/src && cd ~/dual_arm_control/src
git clone git@github.com:KarelGans/rebel_mocap_teleop.git          # -> src/rebel_mocap_teleop/
cp -r ~/igus_mesh_lod_bundle/src/igus_rebel_description .           # vendor CAD, not in this repo
```
`igus_rebel_description` (rebel2 arm and XEG-32 macros, `meshes/`, `meshes_low/`) must sit directly in `src/`: colcon does not
find packages nested inside another package. Build **from the workspace root**, never from `src/`:
```bash
cd ~/dual_arm_control
source /opt/ros/$(ls /opt/ros)/setup.bash
rosdep install --from-paths src --ignore-src -r -y \
  --skip-keys "topic_based_ros2_control gz_ros2_control ros_gz_bridge"   # igus_rebel_description extras, not needed
sudo apt install -y ros-$(ls /opt/ros)-joint-state-publisher-gui
colcon build --symlink-install            # or: scripts/run.sh build
source install/setup.bash
```
```
~/dual_arm_control/                 build/ install/ log/ are created by colcon next to src
  src/
    igus_rebel_description/         from the igus mesh LOD bundle (not in this repo)
    rebel_mocap_teleop/             this repo (ROS package: rebel_demo)
```
- Build **both** packages. A copied `igus_rebel_description` that was never built gives `package 'igus_rebel_description' not found`.
- With `--symlink-install`, edits to existing `.py` files apply on the next launch. Rebuild after adding or renaming files or
  changing `setup.py`, `package.xml`, the URDF, meshes or launch files. Without `--symlink-install`, rebuild after every edit.
- If your prompt shows `(base)` (conda), run `conda deactivate` before building (`run.sh` handles this itself).

### 1.4 Shell environment
Every terminal (driver, launch, `ros2 service call`, keyboard) and the shell that starts Isaac Sim need the same
`ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION`, and both workspaces sourced, **mocap first**. `run.sh` does this for you; by hand:
```bash
export ROS_DOMAIN_ID=11                      # pick a number nobody else on your network uses
source /opt/ros/$(ls /opt/ros)/setup.bash
source ~/mocap_ws/install/setup.bash
source ~/dual_arm_control/install/setup.bash
```
After changing any of these, restart every ROS program (driver, teleop, Isaac Sim) and run `ros2 daemon stop`: running
programs keep their old values. Rather than putting this in `~/.bashrc` (other projects on the PC may use another domain),
use `run.sh` or a small script you source on demand.

---

## 2. Run
`rig` defaults to `both` (two arms, two hands). Nodes: `/rebel_teleop_left`, `/rebel_teleop_right`, `/rebel_gripper_left`,
`/rebel_gripper_right`; with `rig:=left` or `right` (one arm, the other holds its pose) they are `/rebel_teleop` and `/rebel_gripper`.
The frame arguments always come per side and take the Motive streaming ID (`left_hand_frame:=2`; the full TF name
`rigid_body_2` also works): the left arm uses `left_hand_frame` / `left_gripper_frame`, the right arm `right_*`.

### 2.1 Driver
```bash
ros2 launch mocap4r2_optitrack_driver optitrack2.launch.py              # wait for "Configured!"   (run.sh driver)
ros2 lifecycle set /mocap4r2_optitrack_driver_node activate              #                          (run.sh activate)
ros2 topic hz /rigid_bodies                                              # ~120 Hz
```

### 2.2 Real robot
Teleop publishes the commands; your driver (or a small relay node) subscribes to `/dual_arm/isaac_joint_commands` and sends
them to the arm ([interface](#11-real-robot-interface)). Without Isaac there is no clock and no joint-state stream, so two
flags are needed: `use_sim_time:=false` and `fake_state:=true` (see [Start pose and joint feedback](#6-start-pose-and-joint-feedback)).
**Best known command so far** (both arms; this is `run.sh real`):
```bash
ros2 launch rebel_demo teleop.launch.py target:=isaac rig:=both rviz:=false scale_trans:=1.0 cutoff_hz:=6.0 \
  left_hand_frame:=2 left_gripper_frame:=4 right_hand_frame:=1 right_gripper_frame:=3 gripper_mode:=orientation \
  orientation:=semi max_joint_speed_deg:=45 workspace_max:="0.7 0.7 1.4" \
  robot_ypr:="1.5708 0 0" home_deg:="0 0 60 0 50 0" \
  use_sim_time:=false fake_state:=true
```
- `target:=isaac` only selects the output mode (commands go out on a topic); it does not mean Isaac Sim is running.
- `home_deg` is the start pose of both arms. The robot must really be at all zeros (straight up) when you start: without
  joint feedback teleop assumes zeros and ramps from there. If it already is at the pose you want, drop `home_deg` and add `skip_home:=true`.
- `robot_ypr` turns the robot frame to match how you stand ([2.5](#25-align-the-robot-with-how-you-stand)).
- Keep the whole command on one logical line (`\` at the line ends) and quote values that contain spaces.
- Single arm: `rig:=left` (uses `left_hand_frame` / `left_gripper_frame`) or `rig:=right` (uses `right_*`); drop the other side's arguments.

### 2.3 Isaac Sim
In Isaac's Script Editor (stage loaded, timeline **stopped**) run the rig's three scripts, then press **Play**. Isaac must be
started from a shell with the same `ROS_DOMAIN_ID` as your terminals (without one it is on domain 0).
```python
scripts = ["dual_arm_rig_ros2_bridge.py", "sim_flap_weld.py", "sim_scene_control.py"]
path_base = "/home/<you>/dual_arm_control/src/igus_rebel_description/urdf/igus_stand_dual_arm/"   # absolute path, no "~"
for script in scripts:
    with open(path_base + script) as f:
        exec(f.read())
```
The bridge subscribes to `/dual_arm/isaac_joint_commands` and publishes `/dual_arm/isaac_joint_states` and `/clock`. Then
(this is `run.sh isaac`):
```bash
ros2 launch rebel_demo teleop.launch.py target:=isaac rig:=both rviz:=false scale_trans:=1.0 cutoff_hz:=6.0 \
  left_hand_frame:=2 left_gripper_frame:=4 right_hand_frame:=1 right_gripper_frame:=3 gripper_mode:=orientation \
  orientation:=semi max_joint_speed_deg:=45 workspace_max:="0.7 0.7 1.4" \
  robot_ypr:="1.5708 0 0" home_deg:="0 0 60 0 50 0"
```
Do **not** add `use_sim_time:=false` or `fake_state:=true` here, and restart the launch whenever Isaac is stopped or reset
(the sim clock restarts at 0). Isaac details: [section 10](#10-isaac-sim-notes).

### 2.4 Engage, pause, home
At start each arm moves from its measured joints to `home_deg` at the joint speed limit; **engage is refused until `at HOME`
is logged**. Engaging never moves the arm: the arm's pose and your hand's pose at that moment become the reference, and
from then on `tool = reference + scale_trans x (hand movement)`.

Use the keyboard (`run.sh keyboard`, its own terminal, keep it focused): **SPACE** pauses all arms if any runs, otherwise
resumes all; `l` / `r` toggle only the left / right arm; `q` quits. Pausing releases the clutch: the robot holds, you
reposition your hand, and on resume the robot's last pose and your hand's pose at that moment are the new reference (hand
motion during the pause is ignored). The gripper is not affected. `/rebel_teleop*/engaged` (latched) shows the state.

The keyboard has no key for going **back to `home_deg`** (this also disengages); use the service (`/rebel_teleop_left`,
`/rebel_teleop_right`, or `/rebel_teleop` for one arm). The other services (`engage`, `toggle`) are listed in
[section 8](#8-nodes-topics-and-services) for scripts and for use without a keyboard.
```bash
ros2 service call /rebel_teleop_left/home std_srvs/srv/Trigger
```

### 2.5 Align the robot with how you stand
Moving your hand forward must move the tool forward, up must move it up.
- **Turned 90 deg** (you have to face sideways to go "forward"): rotate the robot frame about the vertical axis,
  `robot_ypr:="1.5708 0 0"` (yaw pitch roll, radians); if it goes the wrong way use `-1.5708`, if backwards `3.1416`.
- **Up moves it sideways** (mocap frame is Y-up): set Motive's streaming Up Axis to Z-up, or use `robot_ypr:="0 0 1.5708"` (or `-1.5708`).
- Restart the launch after changing it (the transform is published at start). Only the rotation matters; `robot_xyz` has no effect in this mapping.

### 2.6 Without hardware
RViz only (no mocap, no Isaac): `ros2 launch rebel_demo teleop.launch.py target:=rviz mocap_bridge:=false`, and a fake hand
to move it ([section 12](#12-testing-without-hardware)). Over SSH add `rviz:=false` and run RViz elsewhere:
`rviz2 -d $(ros2 pkg prefix rebel_demo)/share/rebel_demo/rviz/rebel_demo.rviz`.

---

# Reference

## 3. Launch arguments
`ros2 launch rebel_demo teleop.launch.py <arg>:=<value> ...`

| argument | default | meaning |
|---|---|---|
| **mode / setup** | | |
| target | rviz | `rviz`: no Isaac, `joint_merger` publishes the rig's joint states (RViz shows the arm). `isaac`: publish the commands on `command_topic` and expect the joint states from outside (Isaac, your driver, or `fake_state`); also the setting for a real robot |
| rig | both | `both`: both arms, two hands. `left` / `right`: only that arm of the dual-arm rig, the other holds its pose; it uses that side's `left_*` / `right_*` frame arguments |
| use_sim_time | (auto) | `true` with `target:=isaac` (Isaac's `/clock`), else `false`. Real robot: `false` |
| fake_state | false | `target:=isaac` only: publish `/dual_arm/isaac_joint_states` from the commanded joints (others 0). For a robot or setup with no state publisher; never together with Isaac |
| command_topic | `/dual_arm/isaac_joint_commands` | output topic; must match Isaac's ROS2 Subscribe Joint State node |
| joint_names | (auto) | `rig:=left/right` only: 6 comma-separated joint names (base to wrist) if the receiver uses other names; default `arm_<side>_joint1..6` |
| rig_xacro | dual_arm_rig_v2.urdf.xacro | rig model in `igus_rebel_description/urdf` (`dual_arm_rig.urdf.xacro` for the v1 rig) |
| rviz / mesh_lod / lock_gripper | true / low / false | start RViz here (needs a display) / `low` decimated or `full` CAD meshes / fix the XEG joints in the model |
| **mocap** | | |
| left_hand_frame, left_gripper_frame | 1, 2 | Motive streaming IDs of the left palm (moves the arm) / fingertip (gripper); used by `rig:=left` and `both` |
| right_hand_frame, right_gripper_frame | 3, 4 | the same for the right hand; used by `rig:=right` and `both` |
| mocap_bridge | true | start `mocap_tf` (`/rigid_bodies` -> TF); `false` = no mocap needed |
| mocap_frame | map | name of the mocap world frame |
| robot_xyz, robot_ypr | `0 0 0`, `0 0 0` | pose of the robot frame in the mocap frame (m; yaw pitch roll, rad). Only the rotation matters |
| **mapping** | | |
| scale_trans | 1.0 | hand-to-robot translation ratio (0.5 = finer). `scale` is the old name and still works as an alias; `scale_trans` wins if both are given |
| scale_rot | 1.0 | hand-to-robot rotation ratio: wrist turn with `orientation:=true`, joint-6 roll with `semi`; unused with `false` |
| orientation | false | `false` position only; `true` follow the full hand rotation; `semi` position only plus joint 6 follows your hand's roll ([section 5](#5-orientation-modes)) |
| roll_sign | 1.0 | `orientation:=semi`: `-1` flips the joint-6 direction |
| cutoff_hz | 4.0 | low-pass cutoff on the hand pose; lower = smoother but more lag |
| max_joint_speed_deg | 45 | per-joint speed limit (the real ReBeL maximum) |
| workspace_min, workspace_max | `-0.7 -0.7 -0.45`, `0.7 0.7 0.95` | clamp of the tool target in the arm base frame (m), widened to include the pose at engage |
| limit_warn_mm | 5 | warn `joint limit reached` when the IK is this far (mm) off the target with a joint at a limit |
| home_deg | `0 0 0 0 0 0` | start pose of every arm, 6 angles in degrees (e.g. `"0 0 60 0 50 0"`); checked against the joint limits at start |
| skip_home | false | no start-up move; start from the measured joints (zeros if none arrive); `engage` works at once |
| **gripper** | | |
| gripper | true | start the gripper nodes (`false` if Isaac reports errors for the mimic joint `..._right_carriage_joint`) |
| gripper_mode | distance | `distance` (palm-fingertip distance) or `orientation` (their relative rotation) |
| gripper_threshold, gripper_hysteresis | 0.08, 0.02 | distance mode: switching distance and dead band (m) |
| gripper_open_angle, gripper_close_angle | 30, 100 | orientation mode: open below / closed above this angle (deg) |
| gripper_invert | false | `true`: far closes, near opens |

Node-only parameters (set with `--ros-args -p`): teleop `rate_hz` 60, `stale_timeout` 0.2 s (no new pose -> hold),
`reanchor_after` 2 s, `max_hand_speed` 2 m/s, `max_tool_speed` 0.5 m/s, `home_wait` 1 s, `start_engaged` false,
`feedback_from_joint_states` false (seed the IK from measured joints instead of the last command); gripper `debounce` 0.1 s,
`max_speed` 0.05 m/s, `start_open` true, `closed_pos` / `open_pos`; keyboard `teleop`, `debounce` 0.3 s. Full list in each node's docstring.

## 4. How teleop works (every cycle, 60 Hz)
1. Read the hand pose from TF, in the arm base frame (hand body with streaming ID 1 = TF frame `rigid_body_1`).
2. Tracking check: no update or a frozen pose for 0.2 s -> hold. A single-sample jump faster than a hand can move
   (`max_hand_speed`) is a marker glitch and is skipped (accepted if it lasts 0.1 s). Gaps under 2 s keep the anchor and the
   tool catches up at most at `max_tool_speed`; after a longer loss the clutch re-anchors when tracking returns (no jumps).
3. Low-pass filter (position: first order; orientation: slerp), so marker jitter stays out of the joints.
4. Clutch mapping: target = pose at engage + `scale_trans` x (hand now - hand at engage) (with `orientation:=true` also rotated
   by `scale_rot` x the hand's rotation since engage); clamped to the workspace box, which is widened to include the pose at
   engage so engaging never moves the arm.
5. IK: damped least squares with the analytic Jacobian (~3 ms), warm-started from the last command. A joint that would cross
   a limit is pinned at it and the rest re-solved, so an unreachable target gives the closest reachable pose. Position only
   unless `orientation:=true`; the free wrist is pulled gently towards `home_deg`. **Elbow up:** joint 3 is kept >= +15 deg (a
   deliberate bound in `kinematics.py`): in front of the arm every elbow-up solution has joint 3 > 0, and the stretched arm
   (joint 3 = 0) would let it flip to elbow-down. A target needing a straighter arm is not reached; the log says
   `joint limit reached: ... elbow-up bound 15 deg` (any joint at a limit is named the same way).
6. Per-joint speed limit (`max_joint_speed_deg`).
7. Publish `sensor_msgs/JointState` (`arm_<side>_joint1..6`, radians); `joint_merger` merges all arms and grippers into one
   message on `command_topic`, or on `/dual_arm/isaac_joint_states` when `target:=rviz`.

## 5. Orientation modes
| `orientation` | What the IK is asked for | Joint 6 / wrist |
|---|---|---|
| `false` (stable default) | tool **position** only (3 equations, 6 joints); orientation is ignored (not locked: it drifts with joints 1-3) | wrist only pulled gently back to `home_deg`; joint 6 never moves (its axis passes through the tool point, so it cannot change the position) |
| `semi` | same as `false` for joints 1-5 | joint 6 = its angle at engage + `roll_sign` x `scale_rot` x your hand's twist about the tool axis (measured since engage, continuous past 180 deg, limited to +-179 deg and the speed limit). Tilting the hand adds ~0 |
| `true` | position **and** rotation (rotation weighted 0.3) | the wrist follows the hand; near the straight-up pose (joint 5 about 0) joints 4 and 6 share an axis and can swing to +-179 deg for a small tilt. Start with a bent wrist (e.g. `home_deg:="0 0 60 0 50 0"`) |

`semi` measures the twist about the tool axis **at engage**, expressed in the robot base frame; with the tool vertical this
is a rotation about the vertical axis, so swinging the hand sideways also turns joint 6. Wrong direction: `roll_sign:=-1`.

## 6. Start pose and joint feedback
Teleop needs two things from outside: the arm's start pose and (for the rig TF) a joint-state stream on
`/dual_arm/isaac_joint_states`. Isaac's bridge provides both. A real robot usually does not.
- **Isaac (or a real driver publishing the states):** the arm moves from its measured joints to `home_deg` at the speed limit.
- **`fake_state:=true`:** a second `joint_merger` (`rig_state_fake`) publishes the commanded joints as the states, others 0, in
  PC time. It exists to connect the TF tree (the rig has movable lift and slider joints that `robot_state_publisher` needs
  values for). The values are your commands, not measurements. Use `use_sim_time:=false` with it, and never together with Isaac
  (two publishers on different clocks give `TF_OLD_DATA`).
- **No joint feedback at all:** the arm is assumed to be at all zeros and ramps to `home_deg` from there.
- **`skip_home:=true`:** no start move at all; use it when the robot already is where you want it.

## 7. Gripper (HIWIN XEG-32)
Two rigid bodies per hand: the **palm** (moves the arm) and a **fingertip** (controls the gripper). The gripper node switches
between two states, with a dead band against marker jitter and a 0.1 s `debounce` so a one-frame glitch cannot open or
close it; the jaws then move to the closed / open carriage position at `max_speed`.
- **Distance mode** (default): finger extended = far = **open**, curled = near = **closed**. Closes below
  `threshold - hysteresis/2` (7 cm), opens above `threshold + hysteresis/2` (9 cm), otherwise keeps its state; typical palm to
  fingertip distances are ~10-13 cm extended and ~4-6 cm curled. Calibrate by printing the distance and setting the
  threshold to the midpoint and the hysteresis to about a third of the difference:
  ```bash
  # tf2_echo needs the TF frame names rigid_body_<ID> (here the bodies with streaming IDs 1 and 2)
  stdbuf -oL ros2 run tf2_ros tf2_echo rigid_body_1 rigid_body_2 2>/dev/null | awk '/Translation/ {gsub(/[\[\],]/,""); printf "%.1f cm\n", 100*sqrt($3*$3+$4*$4+$5*$5)}'
  ```
  Marker tips: put the fingertip markers on the back of the finger (nail side), facing the cameras; markers on the pad are
  hidden when you curl, and when tracking is lost the gripper keeps its last state. Use different marker patterns for palm and fingertip.
- **Orientation mode** (`gripper_mode:=orientation`): positions are ignored; the gripper follows the total rotation angle (0-180
  deg) of the fingertip body relative to the palm body, which does not change when you turn the whole hand. Open below
  `gripper_open_angle` (30), closed above `gripper_close_angle` (100). Motive's rigid-body axes are arbitrary, so hold your hand
  open and **calibrate once** (that pose counts as 0 deg; not saved, repeat after each launch):
  ```bash
  ros2 service call /rebel_gripper_left/calibrate  std_srvs/srv/Trigger     # rig:=left/right: /rebel_gripper/calibrate
  ros2 service call /rebel_gripper_right/calibrate std_srvs/srv/Trigger     # or: scripts/run.sh calibrate
  ros2 topic echo /rebel_gripper_left/angle_deg                              # tune the two angles with this
  ```
  The angle includes any rotation between the two bodies, so keep both rigidly fixed to your hand and only flip the finger body.
- Output: `~/state` (`Int32`, 0 closed / 1 open), `~/opening_mm` (`Float64`, jaw gap beyond closed; use this for a real XEG
  driver), `~/angle_deg`; the jaw joint positions (m) go to `joint_merger` with the arm joints.

## 8. Nodes, topics and services
| Node (executable) | Does | Publishes / serves |
|---|---|---|
| `mocap_tf` | mocap4r2 `/rigid_bodies` -> TF `<frame_id> -> rigid_body_<ID>` | TF |
| `teleop` (`/rebel_teleop[_side]`) | hand TF -> filter -> clutch/scale -> IK -> speed limit | `~/joint_commands`, `~/target` (`PoseStamped`, the IK target), `~/engaged` (latched `Bool`); services `~/engage` (`SetBool`), `~/home`, `~/toggle` (`Trigger`) |
| `gripper` (`/rebel_gripper[_side]`) | fingertip vs palm -> XEG-32 open / closed | `~/joint_commands`, `~/state`, `~/opening_mm`, `~/angle_deg`; service `~/calibrate` |
| `joint_merger` | all arm and gripper commands -> ONE `JointState` per cycle (Isaac keeps only the latest message, so separate messages would take turns) | `command_topic`, or the joint-state topic when `target:=rviz` |
| `rig_state_fake` (a second `joint_merger`) | `fake_state:=true` only | `/dual_arm/isaac_joint_states` |
| `teleop_keyboard` | SPACE / `l` / `r` -> `~/engage` of the teleop nodes | |

```
launch/teleop.launch.py        starts all of the above plus robot_state_publisher, the static TF map -> world and (RViz)
rebel_demo/kinematics.py       rebel2 joint table (= igus_rebel2_arm_macro.xacro), FK, damped-least-squares IK, limits, twist / rotation scaling
rebel_demo/*_node.py           the nodes above            scripts/run.sh   one-command launcher
rviz/                          RViz config                test/test_teleop.py   pure-python tests (python3 -m pytest test/)
tools/fake_hand.py, fake_isaac.py   fake mocap / fake Isaac (section 12)       tools/make_low_poly_meshes.py   mesh decimation, one-off
```

## 9. Robot model and joint limits
The launch expands `igus_rebel_description/urdf/dual_arm_rig_v2.urdf.xacro`, the same model the Isaac rig uses: a lift, two
sliders and two rebel2 arms (`arm_left_joint1..6`, `arm_right_joint1..6`), each with an XEG-32
(`arm_<side>_xeg32_left/right_carriage_joint`, the right jaw mimics the left). The IK targets `tool0` (link 6 + 52.7 mm)
relative to `arm_<side>_base_link`. Limits follow the igus spec sheet and are the same in the rig model and `kinematics.py`:

| joint | 1 | 2 | 3 | 4 | 5 | 6 |
|---|---|---|---|---|---|---|
| range (deg) | +-179 | -80..140 | -80..140 (IK keeps >= 15) | +-179 | +-95 | +-179 |

Speed 45 deg/s. The 15 deg elbow bound on joint 3 is the one deliberate change in `kinematics.py`.

## 10. Isaac Sim notes
- `rig:=left` sets: joints `arm_left_joint1..6`, IK base frame `arm_left_base_link`, gripper joints
  `arm_left_xeg32_left/right_carriage_joint`, topics `/dual_arm/isaac_joint_commands` and `/dual_arm/isaac_joint_states`.
- Joint names must match Isaac's exactly: `ros2 topic echo /dual_arm/isaac_joint_states --once --field name`. If they differ
  (single arm only): `joint_names:=arm_left_join_1,...,arm_left_join_6`. RViz still uses the rig model's names, so with other names it shows the arm at 0.
- Do not run the rig's own `robot_state_publisher` (e.g. `dual_arm_rig_isaac.launch.py`) at the same time: two robot descriptions give conflicting TF.
- Clock: the bridge stamps joint states with **simulation time** and publishes `/clock`, so `target:=isaac` turns on
  `use_sim_time` for every node. Without it the arm TF (sim time) and the hand TF (PC time) never share a timestamp, teleop
  cannot place the hand and keeps sending HOME. Isaac must be **playing**. Restart teleop after stopping or resetting Isaac.
- Slow movement in Isaac: speed limits count in simulation seconds, so an Isaac running below real time moves slowly.
  Otherwise check `max_joint_speed_deg` and `scale_trans` (defaults 45 / 1.0).

## 11. Real-robot interface
Teleop does not talk to hardware itself. One `sensor_msgs/JointState` at 60 Hz on `/dual_arm/isaac_joint_commands`
(rename with `command_topic:=/your/topic`), only `name` and `position` filled in, **absolute** positions (not deltas):
`arm_left_joint1..6`, `arm_right_joint1..6` in radians and `arm_<side>_xeg32_left/right_carriage_joint` in metres. Your driver
(or a relay node) must split this per arm / gripper, keep the names and zero positions consistent with the rig model, and
forward it to the controller. For a real XEG driver use the gripper's `~/opening_mm` instead of the carriage joints.
```bash
ros2 topic hz   /dual_arm/isaac_joint_commands                            # ~60 Hz
ros2 topic info /dual_arm/isaac_joint_commands -v                         # exactly one publisher: joint_merger
ros2 topic echo /rebel_teleop_left/joint_commands --once --field position  # one arm, 6 values
```
Test the receiving side by hand (stop teleop first, otherwise two publishers fight):
```bash
ros2 topic pub -r 10 /dual_arm/isaac_joint_commands sensor_msgs/msg/JointState \
  "{name: [arm_left_joint1, arm_left_joint2, arm_left_joint3], position: [0.0, 0.2, 0.4]}"
```
Safety: there is **no collision checking**. Use `scale_trans:=0.5 max_joint_speed_deg:=15` first, keep the e-stop in reach, and check
the receiver's limits and zero positions against [section 9](#9-robot-model-and-joint-limits).

## 12. Testing without hardware
`tools/fake_hand.py` publishes fake palm and fingertip bodies at 120 Hz (like `mocap_tf`), engages the clutch once the arm is
at HOME and moves each palm in its arm's own axes (hold, 10 cm down, 15 cm forward with the gripper closing, back, up).
`tools/fake_isaac.py` stands in for Isaac: `/clock`, the rig's 21 joints from 0 following the commands with a short drive lag.
Run these from `~/dual_arm_control/src/rebel_mocap_teleop/tools/`; **one command per terminal**, no mocap driver needed
(`mocap_bridge:=false`) unless noted:
```bash
# a) RViz only, both arms (rig:=both is the default)
ros2 launch rebel_demo teleop.launch.py target:=rviz mocap_bridge:=false              # terminal 1
python3 fake_hand.py --loop                                                            # terminal 2
#    one arm: add rig:=left (or right) to the launch and --arm left (or right) to fake_hand.py

# c) the full Isaac interface with the fake Isaac (for real Isaac, leave fake_isaac.py out)
python3 fake_isaac.py                                                                  # terminal 1
ros2 launch rebel_demo teleop.launch.py target:=isaac mocap_bridge:=false             # terminal 2
python3 fake_hand.py --sim-time --loop                                                 # terminal 3 (--sim-time is required with Isaac's clock)

# d) real-robot-like setup, no Isaac and no mocap
ros2 launch rebel_demo teleop.launch.py target:=isaac mocap_bridge:=false use_sim_time:=false fake_state:=true   # terminal 1
python3 fake_hand.py --loop                                                 # terminal 2

# e) through the mocap converter too: fake_hand publishes /rigid_bodies like the driver (mocap4r2_msgs sourced, mocap_bridge on)
ros2 launch rebel_demo teleop.launch.py target:=rviz                                  # terminal 1
python3 fake_hand.py --pattern cross --output rigid_bodies --loop           # terminal 2
```
Options of `fake_hand.py`: `--arm both` (default) `|left|right`, `--pattern reach|cross` (`cross`: up, back, down, back, left, back, right, back,
each `--amp` 0.10 m, gripper open), `--output tf|rigid_bodies`, `--loop`, `--down` / `--forward` (m), `--seg` (s per move),
`--sim-time`, `--service`, `--palm` / `--finger` (streaming IDs; single arm, default 1/2 for left and 3/4 for right), `--rotate-finger` (close by rotating the
fingertip body, for `gripper_mode:=orientation`), and `--unstable` (2 mm jitter, drop-outs, frozen frames, marker glitches; or
`--noise`, `--jitter`, `--dropouts`, `--freezes`, `--outliers`, `--seed`).

**Find the joint limit:** `fake_hand.py --descend 0.05` lowers the hand 5 cm per step (and `--descend-forward` x 5 cm forward, default
0.3), holds 2 s, repeats up to `--max-down` (0.70 m), then returns; use `--descend-forward 0` for straight down. Measured with the
original bent start pose (`home_deg:="0 13.4 87.9 0 78.7 0"`, tool 35 cm in front of the base and 30 cm up), straight down it follows exactly to 60 cm
below the start (joint 2 at 140 deg and joint 3 at 15 deg are reached together); the further forward, the less deep (x = 0.20 m: 65 cm,
0.35 m: 60 cm, 0.55 m: 40 cm). `workspace_min` z = -0.45 m is below what the arm can reach, so a joint limit, not the box, stops it.
Teleop then warns at most once per second; both arms warn independently:
```
joint limit reached: arm_left_joint3 at lower elbow-up bound 15 deg - target 5 mm out of reach, moving to the closest reachable pose
joint limits: target back in range          (when the hand comes back)
target clamped by workspace_min z = -0.45 m (hand is 5.0 cm beyond) - raise the limit if the arm can go there
```
The first appears when the IK is more than `limit_warn_mm` off the target while a joint sits at a limit (the arm slides along the
limit to the closest pose and follows again as soon as the hand returns); the last means the workspace box, not a joint, stopped the target.

## 13. macOS (RoboStack / conda, e.g. env `ros2_jazzy`)
- The shell is zsh: `source install/setup.zsh`. Install into the env: `conda install -c robostack-jazzy -c conda-forge ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui`.
- `colcon build --symlink-install` fails with `option --editable not recognized` when setuptools >= 80: build without it
  (`colcon build`) or `pip install "setuptools<80"` first. After a failed symlink build a plain build fails with
  `option --uninstall not recognized`: `rm -rf build/rebel_demo install/rebel_demo && colcon build`.
- Typical use: RViz on the Mac (same network, same `ROS_DOMAIN_ID`), driver and teleop on the Linux PC.

## Troubleshooting
| symptom | fix |
|---|---|
| **Mocap** | |
| driver log `... not connected :(` | Motive streaming off or wrong Local Interface; firewall; IPs in the YAML. Relaunch the driver after fixing (it connects only at launch) |
| `ping` to the Motive PC: Destination Host Unreachable | the two PCs are not on the same router / network |
| `mocap_tf` dies: `No module named 'mocap4r2_msgs'`, or `message type 'mocap4r2_msgs/msg/RigidBodies' is invalid` | `source ~/mocap_ws/install/setup.bash` before `~/dual_arm_control/install/setup.bash`, then relaunch (needed also for `target:=isaac`) |
| robot does not move after engage | `ros2 run tf2_ros tf2_echo arm_left_base_link rigid_body_1` must change as you move; check the frame arguments and `ros2 topic hz /rigid_bodies` |
| robot moves sideways / the wrong way | [2.5](#25-align-the-robot-with-how-you-stand) (`robot_ypr`) |
| teleop log `hand pose stale`, `frozen`, `holding` | mocap data late or lost: markers occluded or too few, Motive streaming rate, network. `stale_timeout` only tolerates short gaps |
| **Engage / start** | |
| engage refused: `moving to HOME - engage again when 'at HOME' is logged` | the start move has not finished. If `start pose ...` never appears, the control loop is not ticking (`use_sim_time` true without `/clock`, or Isaac not playing); if it appears but `at HOME` does not, check the joint states (NaN, wrong names) |
| commands stay 0 although engaged, log `holding: no TF arm_left_base_link <- rigid_body_N` | nobody publishes `/dual_arm/isaac_joint_states`, so the rig TF is split. Add `fake_state:=true`, or publish the real states there |
| `engage service not available` (keyboard) | the teleop node is not running, died at start (look for `ValueError`, e.g. a wrong `home_deg`), or the terminal is on another domain / RMW |
| **Clock / domain** | |
| `TF_OLD_DATA ignoring data from the past ... at time 4.4` (all frames, all nodes) | two clocks or two publishers: Isaac's sim time mixed with PC time. With Isaac do not pass `use_sim_time:=false` / `fake_state:=true`; restart the launch after stopping or resetting Isaac; `ros2 topic info /dual_arm/isaac_joint_states -v` must show one publisher |
| teleop and Isaac do not see each other | different `ROS_DOMAIN_ID` (Isaac with none = 0): export the same number in every terminal and in the shell that starts Isaac Sim |
| `ros2 service call` hangs on `waiting for service`, or `sequence size exceeds remaining buffer` | this terminal is on a different ROS setup than teleop (distro, `RMW_IMPLEMENTATION` or `ROS_DOMAIN_ID`). Compare: `tr '\0' '\n' < /proc/$(pgrep -n -f rebel_demo/teleop)/environ \| grep -E '^(ROS_\|RMW_)'`, set the same values, then `ros2 daemon stop` |
| RViz loads someone else's robot | another project uses the same `ROS_DOMAIN_ID`: pick another number everywhere |
| RViz `No transform from ... to world` for the arm links | nobody publishes the joint states: Isaac not playing / bridge scripts not run (`ros2 topic hz /dual_arm/isaac_joint_states`), or in `target:=rviz` the `joint_merger` died |
| RViz `could not connect to display` | started over SSH: use `rviz:=false`, run RViz on a machine with a screen |
| **Build / install** | |
| `colcon build` fails: `No module named 'catkin_pkg'`, log shows `miniconda3/bin/python3` | `conda deactivate`, then rebuild (or `--cmake-args -DPython3_EXECUTABLE=/usr/bin/python3`) |
| `colcon build` made `build/ install/ log/` inside `src/` | wrong folder: `rm -rf ~/dual_arm_control/src/{build,install,log}`, then `cd ~/dual_arm_control && rm -rf build install log && colcon build --symlink-install` |
| `package 'igus_rebel_description' not found` | copy it into `src/` if missing, build it (`colcon build --packages-select igus_rebel_description`) and re-source `install/setup.bash` |
| `No module named 'xacro'` at launch | `sudo apt install ros-$(ls /opt/ros)-xacro` |
| `No package metadata was found for rebel_demo` | broken install from a failed build: `rm -rf build/rebel_demo install/rebel_demo`, rebuild |
| `run.sh`: Permission denied | `chmod +x scripts/run.sh` once |
| **Limits** | |
| constant `joint limit reached` warnings | the IK is pinned at a joint limit. Common: joint 3 at the 15 deg elbow bound when starting from zeros, joints 4 / 6 at +-179 with `orientation:=true` (singularity), the workspace box (`target clamped by ...`). See [5](#5-orientation-modes) and [12](#12-testing-without-hardware) |

## Known issues
- No collision checking; no soft margin before the joint limits (joints 4 and 6 can still be driven to +-179 deg with `orientation:=true`).
- IK targets `tool0`, not the point between the XEG jaws; with `orientation:=true` the fingertips swing.
- The wrist side (joint 5 sign) is not constrained; only the elbow is (joint 3 >= 15 deg).
- `fake_state` mirrors the commands, it does not measure the arm. The XEG open / closed carriage values are the model joint limits (16.5 mm travel per jaw): check against the datasheet.

## Licence and credits
- Robot model: `igus_rebel_description` from the igus mesh LOD bundle, not included (dual-arm rig, rebel2 arm, XEG-32, decimated meshes).
- Mocap driver (not included): [MOCAP4ROS2-Project/mocap4ros2_optitrack](https://github.com/MOCAP4ROS2-Project/mocap4ros2_optitrack).
- Kinematics, IK, `mocap_tf` and `teleop` nodes: written for this project (numpy + rclpy); derived from
  [duckQuu/rebel_mocap_teleop](https://github.com/duckQuu/rebel_mocap_teleop). `package.xml` declares Apache-2.0; there is no `LICENSE` file yet.
