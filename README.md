# rebel_demo — igus ReBeL + HIWIN XEG-32 hand teleoperation from OptiTrack mocap

Your hand (an OptiTrack rigid body) drives an igus ReBeL 6-DoF (rebel2) arm of the Isaac **dual-arm rig** in real
time, and a second rigid body opens / closes its HIWIN XEG-32 gripper. One arm (`rig:=left` or `right`) or both
arms with two hands (`rig:=both`). The hand pose is filtered, converted to joint angles with IK at 60 Hz, and sent
to Isaac Sim as one `sensor_msgs/JointState` (`/dual_arm/isaac_joint_commands`); RViz shows the same rig model.

```
Motive (Windows) --NatNet--> mocap4r2 OptiTrack driver --/rigid_bodies-->
  mocap_tf --TF: one frame per Motive streaming ID-->
    teleop  (hand ID 1: filter -> IK -> speed limit)      -> arm_<side>_joint1..6
    gripper (|fingertip ID 2 - hand ID 1| < threshold)    -> XEG-32 open / close
    joint_merger (all arms + grippers in ONE message)
  --> /dual_arm/isaac_joint_commands -> Isaac Sim (rig bridge scripts)
  /dual_arm/isaac_joint_states (from Isaac) -> robot_state_publisher -> RViz
```

## Contents
```
rebel_demo/
  rebel_demo/kinematics.py     rebel2 joint table (= igus_rebel2_arm_macro.xacro), FK, IK (damped least squares)
  rebel_demo/mocap_tf_node.py  node "mocap_tf": mocap4r2 /rigid_bodies -> TF (one TF frame per Motive streaming ID)
  rebel_demo/teleop_node.py    node "teleop":   hand TF -> low-pass -> clutch/scale -> IK -> speed limit -> JointState
  rebel_demo/gripper_node.py   node "gripper":  distance between two rigid bodies -> XEG-32 open (1) / closed (0)
  rebel_demo/joint_merger_node.py  node "joint_merger": all arm / gripper commands -> ONE JointState per cycle
  rebel_demo/keyboard_node.py  node "teleop_keyboard": SPACE pauses / resumes the arm(s)
  launch/teleop.launch.py      starts mocap_tf, teleop + gripper per arm, joint_merger, robot_state_publisher, static TF, (RViz)
  rviz/                        RViz config
  tools/fake_hand.py           fake mocap: palm + fingertip rigid bodies for one or both hands (section 6)
  tools/fake_isaac.py          fake Isaac: sim clock + rig joint states that follow the commands (section 6)
  tools/make_low_poly_meshes.py  mesh decimation used to make igus_rebel_description/meshes_low/
  test/test_teleop.py          pure-python tests: python3 -m pytest test/
```

### Robot model
The launch file expands `igus_rebel_description/urdf/dual_arm_rig_v2.urdf.xacro` (`rig_xacro:=dual_arm_rig.urdf.xacro`
for the v1 rig): the same model the Isaac rig uses, with the lift, two sliders and two rebel2 arms
(`arm_left_joint1..6`, `arm_right_joint1..6`), each with an XEG-32 (`arm_<side>_xeg32_left/right_carriage_joint`, the
right jaw mimics the left). The IK works in `tool0` (link6 + 52.7 mm) of the chosen arm, relative to `arm_<side>_base_link`.
Joint limits follow the igus spec sheet (joint2 -80..140 deg, joint3 -80..140 deg, joint5 +-95 deg, others +-179 deg,
45 deg/s) and are identical in the rig model and `kinematics.py`, except one deliberate change: **joint3 >= +15 deg**
in `kinematics.py` (elbow-up bound, see "How teleop works"). `mesh_lod:=low` (default) loads the decimated
`meshes_low/`, `mesh_lod:=full` the full CAD meshes.

## Requirements
- Linux PC that runs the mocap driver (the NatNet library is Linux-only), with **one** ROS 2 distribution:
  Ubuntu 22.04 + Humble, or Ubuntu 24.04 + Jazzy. The code is plain rclpy and works on both.
  The commands below detect it with `$(ls /opt/ros)` (works when exactly one distribution is installed).
  Never mix two distributions (in terminals, workspaces or Isaac Sim).
- Python: numpy.
- ROS packages: `xacro`, `robot_state_publisher`, `rviz2`, and `igus_rebel_description` (section 3).
- RViz can run on the same Linux PC (needs a screen) or on another machine (e.g. a Mac with RoboStack)
  that has this package built, same network, same `ROS_DOMAIN_ID`.

## 1. Mocap driver (separate workspace)
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
Do not use the `OptiTrack/mocap4ros2_optitrack` fork: its dependency file points to a repository that does not exist.

Set the network addresses in the driver config. Only the first line needs your value: the Motive PC IP
(Motive: Edit -> Settings -> Streaming -> Local Interface). This PC's IP is detected automatically.
```bash
MOTIVE_IP=192.168.1.10                       # <- the Motive PC IP
LOCAL_IP=$(hostname -I | awk '{print $1}')   # this Linux PC (check with: echo $LOCAL_IP)
CFG=~/mocap_ws/src/mocap4ros2_optitrack/mocap4r2_optitrack_driver/config/mocap4r2_optitrack_driver_params.yaml
sed -i -E "s/^( *connection_type:).*/\1 \"Multicast\"/; s/^( *server_address:).*/\1 \"$MOTIVE_IP\"/; \
  s/^( *local_address:).*/\1 \"$LOCAL_IP\"/; s/^( *multicast_address:).*/\1 \"239.255.42.99\"/" "$CFG"
grep -E "connection_type|server_address|local_address|multicast_address" "$CFG"
cd ~/mocap_ws && colcon build --symlink-install
```
`connection_type` must match Motive's Transmission Type (Multicast here; use Unicast in both if needed).

## 2. Motive (Windows)
Edit -> Settings -> Streaming: **Broadcast Frame Data** on, **Local Interface** = the Motive PC IP on the shared
network, **Transmission Type** = same as the YAML, **Rigid Bodies** on. Windows firewall: network profile Private,
allow Motive.

Rigid bodies: all you need from Motive is each body's **Streaming ID** (Properties -> Streaming ID). You pass that number to
the launch arguments; the name you give the body in Motive does not matter. Defaults used by the launch file:

| rigid body | Streaming ID | launch argument |
|---|---|---|
| palm (moves the arm) | 1 | `hand_frame:=1` |
| fingertip (opens / closes the gripper) | 2 | `gripper_frame:=2` |

Other IDs: just pass the number, e.g. `hand_frame:=5 gripper_frame:=6`. (Internally the ID becomes the TF frame
`rigid_body_<ID>`, which you only need to know for debugging with `tf2_echo`; passing the full name also works.) The `mocap_tf`
log prints `new rigid body: map -> rigid_body_<ID>` for every rigid body it receives.

## 3. Build this package
Get the code into the workspace `src` folder (the clone creates `src/rebel_mocap_teleop/`; the ROS package inside it is
called `rebel_demo`):
```bash
mkdir -p ~/dual_arm_control/src
cd ~/dual_arm_control/src
git clone git@github.com:KarelGans/rebel_mocap_teleop.git
```
The robot model needs the `igus_rebel_description` package (rebel2 arm + XEG-32 macros, `meshes/` and
`meshes_low/`). It is **not in this repository** (vendor CAD). Put the igus mesh LOD bundle folder in your home
directory as `~/igus_mesh_lod_bundle`, then copy the package next to `rebel_mocap_teleop` (colcon does not find packages
nested inside another package, so it must sit directly in `src/`):
```bash
cp -r ~/igus_mesh_lod_bundle/src/igus_rebel_description ~/dual_arm_control/src/
```
Build **from the workspace root** `~/dual_arm_control` — never from `src/` or `src/rebel_mocap_teleop/`:
```bash
cd ~/dual_arm_control
source /opt/ros/$(ls /opt/ros)/setup.bash
rosdep install --from-paths src --ignore-src -r -y \
  --skip-keys "topic_based_ros2_control gz_ros2_control ros_gz_bridge"   # igus_rebel_description extras, not needed here
sudo apt install -y ros-$(ls /opt/ros)-joint-state-publisher-gui
colcon build --symlink-install
source install/setup.bash
```
Build **both** packages (`igus_rebel_description` and `rebel_demo`). Copying `igus_rebel_description` into `src/`
is not enough: `package 'igus_rebel_description' not found` at launch means it was never built. To build just it:
`colcon build --packages-select igus_rebel_description`.

If colcon fails with `No module named 'catkin_pkg'` and the log shows `.../miniconda3/bin/python3`, a conda
environment is first on `PATH`. Run `conda deactivate` and build again (or add
`--cmake-args -DPython3_EXECUTABLE=/usr/bin/python3`).

Check the layout. `build`, `install`, `log` must sit next to `src`, not inside it:
```
~/dual_arm_control/
  build/  install/  log/          <- created by colcon build (run in ~/dual_arm_control)
  src/
    igus_rebel_description/       <- from the igus mesh LOD bundle (not in this repo)
    rebel_mocap_teleop/           <- this repo (ROS package name: rebel_demo)
      launch/  rebel_demo/  resource/  rviz/  test/  tools/
      package.xml  README.md  setup.cfg  setup.py
```
Built in the wrong folder by mistake? Delete the stray output and rebuild from the root:
```bash
rm -rf ~/dual_arm_control/src/build ~/dual_arm_control/src/install ~/dual_arm_control/src/log
cd ~/dual_arm_control && rm -rf build install log && colcon build --symlink-install
```
When to rebuild: with `--symlink-install`, edits to existing `.py` files apply on the next launch. Rebuild after
adding or renaming files, or changing `setup.py`, `package.xml`, the URDF, meshes or launch files.
Without `--symlink-install`, rebuild after every edit.

### macOS (RoboStack / conda, e.g. env `ros2_jazzy`)
- The shell is zsh: `source install/setup.zsh`, not `setup.bash`.
- `colcon build --symlink-install` fails with `option --editable not recognized` when the env has
  setuptools >= 80. Either build without it (`colcon build`), or `pip install "setuptools<80"` first.
- After a failed symlink build, a plain build fails with `option --uninstall not recognized`.
  Delete the leftovers and rebuild: `rm -rf build/rebel_demo install/rebel_demo && colcon build`
- Install into the env: `conda install -c robostack-jazzy -c conda-forge ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui`
- Workspace on this Mac: `~/Downloads/rebel_teleop_ws` (use it instead of `~/dual_arm_control`).

Add the environment to `~/.bashrc` once (every terminal must have the **same** values — and the terminal that
starts Isaac Sim too). `ROS_DOMAIN_ID=42`: pick a number nobody else on your network uses.
```bash
sudo apt install -y ros-$(ls /opt/ros)-rmw-cyclonedds-cpp
cat >> ~/.bashrc <<'BASHRC'
source /opt/ros/$(ls /opt/ros)/setup.bash
source ~/mocap_ws/install/setup.bash
source ~/dual_arm_control/install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
BASHRC
source ~/.bashrc && ros2 daemon stop
```
After changing any of these later, restart every ROS program (driver, teleop, Isaac Sim) and run
`ros2 daemon stop`: running programs keep their old values.

## 4. Run
**Every terminal** (driver, launch, `ros2 service call`, keyboard) must have both workspaces sourced,
mocap first. Without the `~/.bashrc` lines from section 3, run in each new terminal:
```bash
source /opt/ros/$(ls /opt/ros)/setup.bash
source ~/mocap_ws/install/setup.bash     # mocap4r2_msgs: needed by mocap_tf even with target:=isaac
source ~/dual_arm_control/install/setup.bash     # rebel_demo + igus_rebel_description
```
Skipping the mocap line makes the `mocap_tf` node die with `No module named 'mocap4r2_msgs'` (the rest of the launch
keeps running, but no hand TF is published). Only `mocap_bridge:=false` (section 6) runs without it.

One arm (`rig:=left` default, or `rig:=right`); for two hands see "Both arms" below.
```bash
# terminal 1: driver (wait for "Configured!")
ros2 launch mocap4r2_optitrack_driver optitrack2.launch.py
# terminal 2: activate, check data (~120 Hz)
ros2 lifecycle set /mocap4r2_optitrack_driver_node activate
ros2 topic hz /rigid_bodies
# terminal 3, RViz only, no Isaac (palm = streaming ID 1, fingertip = ID 2)
#   (add rviz:=false over SSH, then run RViz elsewhere)
ros2 launch rebel_demo teleop.launch.py target:=rviz rig:=left hand_frame:=1 scale:=0.5 \
  gripper_frame:=2 gripper_threshold:=0.08 gripper_hysteresis:=0.02 mesh_lod:=low
# terminal 3, Isaac Sim instead (Isaac playing, rig bridge scripts run, see section 5)
ros2 launch rebel_demo teleop.launch.py target:=isaac rig:=left hand_frame:=1 rviz:=false \
  scale:=1.0 cutoff_hz:=6.0
# terminal 2: wait for "at HOME" in the launch log, then engage the clutch and move your hand slowly
ros2 service call /rebel_teleop/engage std_srvs/srv/SetBool "{data: true}"
```
At start the arm moves from its current joints to HOME (all joints 0: arm straight up; `HOME` in `kinematics.py`) at
the joint speed limit; engage is refused until `at HOME` is logged. Engaging never moves the arm.
`{data: false}` releases the clutch: the robot holds, reposition your hand, engage again.
Choose the start pose with `home_deg:="0 0 100 0 50 0"` (degrees, joints 1-6, both arms). The arm ramps from its measured
joints to it at `max_joint_speed_deg`. **No joint feedback** (real robot with `fake_state:=true`)? Then the arm is *assumed* to be
all zeros (straight up) and ramps from there, so the real robot must really be at zeros when you start.
Robot already at HOME (e.g. a real arm powered up straight)? Add `skip_home:=true`: no start-up move (it ignores `home_deg` for the move), engage works at once.
Back to HOME at any time (disengages): `ros2 service call /rebel_teleop/home std_srvs/srv/Trigger`.

Pause / resume with the spacebar (own terminal, keep it focused; Isaac keeps running, the gripper is not affected):
```bash
ros2 run rebel_demo teleop_keyboard      # SPACE = pause <-> resume, q = quit
# rig:=both:  ros2 run rebel_demo teleop_keyboard --ros-args -p teleop:=/rebel_teleop_left,/rebel_teleop_right
# without a keyboard:  ros2 service call /rebel_teleop/toggle std_srvs/srv/Trigger
```
On resume the robot's last pose and your hand's pose at that moment are the new reference, so you continue from
where it stopped; hand motion during the pause is ignored (use it to reposition your hand). With two arms, SPACE
pauses all of them if any runs, otherwise resumes all. The latched topic `/rebel_teleop/engaged` shows the state.

`orientation:=semi`: the arm is solved exactly as with `orientation:=false` (position only, joints 1-5 identical), and joint 6 is set to its angle at engage + `roll_sign` x `scale_rot` x the twist of your hand about the tool axis (measured since engage; tilting the hand adds ~0; continuous past 180 deg; limited to +-179 deg and `max_joint_speed_deg`). Turn your hand like a doorknob and the gripper rolls. Wrong direction: `roll_sign:=-1`.

Hand up must move the robot up. If it moves sideways the mocap frame is Y-up: set Motive streaming
Up Axis to Z-up, or relaunch with `robot_ypr:="0 0 1.5708"` (or `-1.5708`).

RViz only, no mocap and no Isaac (the arm moves to HOME and stays, gripper open; the rig's other joints are faked at 0):
```bash
ros2 launch rebel_demo teleop.launch.py target:=rviz mocap_bridge:=false
```
To watch the arm move without mocap, add the fake hand stream (section 6).
Model viewer with joint sliders (instead of the launch above, not together with it; one command per terminal):
```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro $(ros2 pkg prefix igus_rebel_description)/share/igus_rebel_description/urdf/dual_arm_rig_v2.urdf.xacro include_ros2_control:=false mesh_lod:=low)"
ros2 run joint_state_publisher_gui joint_state_publisher_gui
rviz2 -d $(ros2 pkg prefix rebel_demo)/share/rebel_demo/rviz/rebel_demo.rviz
```
Check the commands: `ros2 topic echo /dual_arm/isaac_joint_commands` (Isaac mode) or
`ros2 topic echo /dual_arm/isaac_joint_states` (RViz mode, published by `joint_merger`).
RViz on another machine: `rviz2 -d $(ros2 pkg prefix rebel_demo)/share/rebel_demo/rviz/rebel_demo.rviz`

## How teleop works (every cycle, 60 Hz)
1. Read the hand pose from TF (hand body, e.g. ID 1 = TF frame `rigid_body_1`, in the frame `arm_left_base_link`).
2. Tracking check: no update or a frozen pose for 0.2 s -> hold. A single-sample jump faster than a hand can move (`max_hand_speed`, 2 m/s) is a marker glitch and is skipped (accepted if it persists 0.1 s). Gaps under 2 s (`reanchor_after`) keep the anchor and the tool catches up at most at `max_tool_speed` (0.5 m/s); after a longer loss the clutch re-anchors when tracking returns (no jumps). TF is received in its own thread, so the IK never delays it.
3. Low-pass filter on the hand pose (position: first order; orientation: slerp). Filtering before IK keeps marker jitter out of the joints.
4. Clutch mapping: target = robot pose at engage + scale_trans x (hand now - hand at engage) (and, with `orientation:=true`, rotated by `scale_rot` x the hand's rotation since engage); clamped to a workspace box (in the arm base frame, x/y +-0.7 m, z -0.45..0.95 m) that is widened to include the pose at engage, so engaging never moves the arm.
5. IK (damped least squares with the analytic Jacobian, ~3 ms; joints that would cross a limit are pinned at it and the rest re-solved, so an unreachable target gives the closest reachable pose; warm-started from the last command; position only unless `orientation:=true`, the free wrist is pulled gently towards HOME). **Elbow up:** joint3 is kept >= +15 deg (lower limit in `kinematics.py`). In front of the arm every elbow-up solution has joint3 > 0, and the arm could only reach elbow-down by passing through the stretched arm (joint3 = 0); the bound stops it 15 deg before that. A target that would need a straighter arm is not reached: the arm holds at the bound and logs `joint limit reached: ... elbow-up bound 15 deg` (any joint at a limit is named the same way). At start-up the arm moves from its measured pose (e.g. Isaac's straight-up joint3 = 0) to HOME smoothly before the bound applies.
6. Per-joint speed limit (default 45 deg/s, the real ReBeL maximum).
7. Publish `sensor_msgs/JointState` (`arm_<side>_joint1..6`, radians); `joint_merger` combines all arms and grippers into one message on `command_topic` (default `/dual_arm/isaac_joint_commands`), or on `/dual_arm/isaac_joint_states` when `target:=rviz`.

## Launch arguments
| argument | default | meaning |
|---|---|---|
| target | rviz | `rviz`: no Isaac, `joint_merger` publishes the rig's joint states; `isaac`: Isaac publishes them and gets the commands |
| hand_frame | 1 | Motive **streaming ID** of the hand's rigid body (`rig:=left` / `right`) |
| scale_trans | 1.0 | hand-to-robot **translation** ratio (0.5 = finer). `scale` is the old name and still works as an alias; `scale_trans` wins if both are given |
| scale_rot | 1.0 | hand-to-robot **rotation** ratio: with `orientation:=true` the wrist turns by `scale_rot` x your hand rotation since engage; with `orientation:=semi` joint 6 turns by `scale_rot` x your hand roll; unused with `orientation:=false` |
| orientation | false | `false` position only (wrist free; joint 6 never moves) · `true` follow the full hand rotation (swings joints 4/6 near the straight-up singularity) · `semi` position only **plus joint 6 follows your hand's roll** about the tool axis (see below) |
| home_deg | `0 0 0 0 0 0` | start pose of every arm, 6 joint angles in degrees (e.g. `"0 0 100 0 50 0"`): the arm moves there at start, and the IK's posture preference pulls towards it. Must be inside the joint limits (checked at startup) |
| roll_sign | 1.0 | `orientation:=semi`: `-1` flips the joint 6 direction (the roll gain is `scale_rot`) |
| cutoff_hz | 4.0 | filter cutoff; lower = smoother but more lag |
| max_joint_speed_deg | 45 | per-joint speed limit after IK |
| skip_home | false | `true`: no start-up move to `home_deg`; teleop starts from the measured joints (all zeros if none arrive) and `engage` works at once. For a robot that already is at the pose you want |
| workspace_min / workspace_max | -0.7 -0.7 -0.45 / 0.7 0.7 0.95 | IK target clamp x y z in the arm base frame [m]; widened to include the pose at engage; a warning names the bound that clamps |
| limit_warn_mm | 5 | warn `joint limit reached` when the IK is this far [mm] off the target and a joint is at a limit |
| robot_xyz / robot_ypr | 0 0 0 | robot base pose in the mocap `map` frame (m, rad) |
| mocap_bridge | true | start the `mocap_tf` converter (`false` = no mocap needed) |
| mocap_frame | map | name of the mocap world frame that `robot_xyz` / `robot_ypr` are given in |
| rviz | true | start RViz here (needs a display) |
| mesh_lod | low | `low` (decimated meshes) or `full` (CAD) |
| lock_gripper | false | true = XEG joints fixed in the model (no gripper joints in RViz or commands) |
| gripper | true | start the gripper node |
| gripper_frame | 2 | streaming ID of the gripper-control rigid body (`rig:=left` / `right`) |
| gripper_ref_frame | (hand_frame) | streaming ID; the gripper distance is measured to this rigid body |
| gripper_mode | distance | `distance` (palm <-> fingertip distance) or `orientation` (their relative rotation) |
| gripper_open_angle / gripper_close_angle | 30 / 100 | orientation mode: open below / closed above this angle [deg] |
| gripper_threshold | 0.08 | switching distance [m] palm <-> fingertip |
| gripper_hysteresis | 0.02 | dead band [m] around the threshold |
| gripper_invert | false | true = far closes, near opens |
| rig | left | `left` / `right` = that arm of the dual-arm rig, the other holds its pose; `both` = both arms, two hands |
| left_hand_frame / left_gripper_frame | 1 / 2 | `rig:=both`: streaming IDs of the left palm / fingertip |
| right_hand_frame / right_gripper_frame | 3 / 4 | `rig:=both`: streaming IDs of the right palm / fingertip |
| rig_xacro | dual_arm_rig_v2.urdf.xacro | rig model in `igus_rebel_description/urdf` |
| use_sim_time | (auto) | `true` with `target:=isaac` (Isaac's `/clock`), else `false` |
| joint_names | (auto) | 6 comma-separated arm joint names (base to wrist) if Isaac uses other names, e.g. `arm_left_join_1,...,arm_left_join_6`; default = `arm_<side>_joint1..6` |
| command_topic | (auto) | `/dual_arm/isaac_joint_commands`; must match Isaac's ROS2 Subscribe Joint State node exactly |

## Gripper (HIWIN XEG-32)
Two rigid bodies on the same hand: the **palm** (streaming ID 1, moves the arm) and a **fingertip** (ID 2,
Motive Streaming ID 2, controls the gripper). Finger extended = far = **open**, finger curled = near = **closed**.
Moving or rotating the whole hand does not change their distance, so arm motion never opens or closes the gripper.
The node measures the distance between the two rigid bodies' pivot points and switches between two states:
```
distance < threshold - hysteresis/2   -> CLOSE (0)
distance > threshold + hysteresis/2   -> OPEN  (1)
in between, or tracking lost          -> keep the current state
```
The dead band stops marker jitter at the threshold from making the gripper flicker, and a new state must hold for
0.1 s (`debounce`) before the gripper switches, so a one-frame marker glitch cannot open or close it. The jaws then move to the
closed / open carriage position at `max_speed`. Defaults: threshold 0.08 m, hysteresis 0.02 m (closes below 7 cm,
opens above 9 cm); typical palm -> index fingertip distances are ~10-13 cm extended and ~4-6 cm curled.

Calibrate: print the distance live, hold the finger extended, then curled, and set `gripper_threshold` to the
midpoint and `gripper_hysteresis` to about a third of the difference:
```bash
# tf2_echo needs the TF frame names rigid_body_<ID> (here the bodies with streaming IDs 1 and 2)
stdbuf -oL ros2 run tf2_ros tf2_echo rigid_body_1 rigid_body_2 2>/dev/null | awk '/Translation/ {gsub(/[\[\],]/,""); printf "%.1f cm\n", 100*sqrt($3*$3+$4*$4+$5*$5)}'
ros2 topic echo /rebel_gripper/state     # (rig:=both: /rebel_gripper_left/state) 1 = open, 0 = closed (the launch log also prints OPEN / CLOSE + distance)
```
Marker tips: put the fingertip markers on the back of the finger (nail side), facing the cameras: markers on the
pad are hidden when you curl, and when tracking is lost the gripper keeps its last state (it may then never close).
Use clearly different marker patterns for palm and fingertip.
### Orientation mode (`gripper_mode:=orientation`)
Instead of the distance, the gripper follows how much the fingertip body is **rotated relative to the palm body**.
Positions are ignored. The relative rotation (fingertip body seen from the palm body) does not change when you turn the
whole hand, so it works facing up, down, left or right. The node takes the total rotation angle (0-180 deg):
```
angle < gripper_open_angle   (30)   -> OPEN   (same orientation as the calibrated open pose)
angle > gripper_close_angle  (100)  -> CLOSE  (finger body flipped, ~180 deg)
in between, or tracking lost        -> keep the current state
```
Motive rigid-body axes are arbitrary (set when each body was created), so hold your hand relaxed / open and call
the calibration once; that pose then counts as 0 deg:
```bash
ros2 launch rebel_demo teleop.launch.py rig:=left target:=isaac hand_frame:=1 \
  gripper_frame:=2 gripper_mode:=orientation rviz:=false
ros2 service call /rebel_gripper/calibrate std_srvs/srv/Trigger
ros2 topic echo /rebel_gripper/angle_deg     # tune gripper_open_angle / gripper_close_angle with this
# rig:=both: one call per hand (or `scripts/run.sh calibrate`), node names /rebel_gripper_left and /rebel_gripper_right:
#   ros2 service call /rebel_gripper_left/calibrate std_srvs/srv/Trigger
#   ros2 service call /rebel_gripper_right/calibrate std_srvs/srv/Trigger
```
The angle includes any rotation (twist or tilt about any axis) between the two bodies, so keep the two bodies
rigidly fixed to your hand and only flip the finger body to close. The 0.1 s debounce applies here too.

Topics: `/rebel_gripper/state` (0/1), `/rebel_gripper/opening_mm`. The jaw positions go to `joint_merger` together
with the arm joints. Node parameters: `joint_prefix` (set by the launch: `arm_left_xeg32_` / `arm_right_xeg32_`),
`closed_pos` / `open_pos` (carriage values [m], default = model joint limits).

## 5. Isaac Sim (dual-arm rig)
In Isaac's Script Editor (stage loaded, timeline **stopped**), run the rig's three scripts, then press Play. Adjust
`path_base` to where the scripts are:
```python
scripts = ["dual_arm_rig_ros2_bridge.py", "sim_flap_weld.py", "sim_scene_control.py"]
path_base = "/home/<you>/dual_arm_control/src/igus_rebel_description/urdf/igus_stand_dual_arm/"   # absolute path, no "~"
for script in scripts:
    with open(path_base + script) as f:
        exec(f.read())
```
The bridge script subscribes to `/dual_arm/isaac_joint_commands` and publishes `/dual_arm/isaac_joint_states` and
`/clock`. Then teleop **one** arm (`rig:=left` or `rig:=right`); the other arm holds its pose:
```bash
ros2 launch rebel_demo teleop.launch.py target:=isaac rig:=left hand_frame:=1 rviz:=false \
  scale:=1.0 cutoff_hz:=6.0
```
`rig:=left` sets: joints `arm_left_joint1..6`, IK base frame `arm_left_base_link`, gripper joints
`arm_left_xeg32_left/right_carriage_joint`, topics `/dual_arm/isaac_joint_commands` and `/dual_arm/isaac_joint_states`.
- `rig_xacro:=dual_arm_rig.urdf.xacro` if Isaac runs the v1 rig (default `dual_arm_rig_v2.urdf.xacro`).
- Do not run the rig's own `robot_state_publisher` (e.g. `dual_arm_rig_isaac.launch.py`) at the same time: two
  robot descriptions publish conflicting TF.
- `robot_xyz` / `robot_ypr` place the rig's `world` in the mocap frame; only the rotation matters in relative mode.
- Joint names must match Isaac's exactly. Check with
  `ros2 topic echo /dual_arm/isaac_joint_states --once --field name`; if they differ from `arm_left_joint1..6`, pass
  them: `joint_names:=arm_left_join_1,arm_left_join_2,arm_left_join_3,arm_left_join_4,arm_left_join_5,arm_left_join_6`.
  RViz still uses the rig model's names, so with other names it shows the arm at 0.
- Clock: the bridge stamps joint states with **simulation time** and publishes `/clock`, so `target:=isaac` turns on
  `use_sim_time` for every node (override with `use_sim_time:=false`). Without it the arm TF (sim time) and the
  hand TF (PC time) never share a timestamp, teleop cannot place the hand, and it keeps sending HOME (all zeros).
  Isaac must be **playing** (the clock only runs then). Restart teleop after stopping / resetting Isaac.
- Gripper: the rig's XEG joints are unlocked, so `gripper:=true` (default) drives that arm's jaws. If Isaac reports
  errors for `..._right_carriage_joint` (mimic joint), use `gripper:=false`.
- Every arm and gripper publishes on its own topic (`/rebel_teleop/joint_commands`, ...) and `joint_merger` sends all
  of them to Isaac as **one** message per cycle: Isaac's Subscribe Joint State keeps only the latest message, so
  separate arm and gripper messages would otherwise take turns.

### Both arms (two hands)
Four rigid bodies: a palm and a fingertip per hand. Defaults (change them to your Motive Streaming IDs):

| | palm (moves the arm) | fingertip (gripper) |
|---|---|---|
| left arm | `left_hand_frame:=1` | `left_gripper_frame:=2` |
| right arm | `right_hand_frame:=3` | `right_gripper_frame:=4` |

```bash
ros2 launch rebel_demo teleop.launch.py target:=isaac rig:=both rviz:=false scale:=1.0 cutoff_hz:=6.0 \
  left_hand_frame:=1 left_gripper_frame:=2 \
  right_hand_frame:=3 right_gripper_frame:=4
# wait for "at HOME" from both arms, then engage each arm separately (or both):
ros2 service call /rebel_teleop_left/engage std_srvs/srv/SetBool "{data: true}"
ros2 service call /rebel_teleop_right/engage std_srvs/srv/SetBool "{data: true}"
# or pause / resume both with the spacebar:
ros2 run rebel_demo teleop_keyboard --ros-args -p teleop:=/rebel_teleop_left,/rebel_teleop_right
```
- Two teleop nodes (`/rebel_teleop_left`, `/rebel_teleop_right`) and two gripper nodes (`/rebel_gripper_left`,
  `/rebel_gripper_right`; state on `/rebel_gripper_<side>/state`); each gripper measures its fingertip to its own palm.
- All 12 arm joints and 4 jaw joints go to Isaac in one message per cycle (`joint_merger`).
- `scale_trans`, `scale_rot`, `mode`, `orientation`, `cutoff_hz`, `max_joint_speed_deg`, `gripper_threshold` / `gripper_hysteresis` apply
  to both arms. `hand_frame`, `gripper_frame`, `gripper_ref_frame` and `joint_names` are for one arm only.

#### Example: Isaac Sim, both arms, joint 6 follows the hand roll (`orientation:=semi`)
Isaac and every ROS terminal must share one `ROS_DOMAIN_ID` (here 11) and the same `RMW_IMPLEMENTATION`; start Isaac
from a terminal where it is exported, then run the three bridge scripts and press **Play**.
```bash
export ROS_DOMAIN_ID=11
source ~/mocap_ws/install/setup.bash
source ~/dual_arm_control/install/setup.bash
ros2 launch rebel_demo teleop.launch.py target:=isaac rig:=both rviz:=false scale:=1.0 cutoff_hz:=8.0 \
  left_hand_frame:=2 left_gripper_frame:=4 \
  right_hand_frame:=1 right_gripper_frame:=3 gripper_mode:=orientation \
  orientation:=semi max_joint_speed_deg:=45 workspace_max:="0.7 0.7 1.4" \
  home_deg:="0 0 100 0 50 0"
```
- Do **not** add `use_sim_time:=false` or `fake_state:=true` with Isaac: the bridge publishes `/clock` and the joint
  states, and a second publisher with another clock gives `TF_OLD_DATA` warnings.
- Restart the launch every time Isaac is stopped or reset (the sim clock restarts at 0).
- `orientation:=semi`: position-only arm (joints 1-5 as `orientation:=false`), joint 6 follows your hand's roll;
  `roll_sign:=-1` flips it, `scale_rot:=0.5` halves it. For first tests on a robot add `scale:=0.5 max_joint_speed_deg:=15`.
- Slow movement: lower `max_joint_speed_deg` / `scale` are the cause; the defaults are 45 / 1.0.

## 6. Test without mocap / without Isaac (fake data)
`tools/fake_hand.py` is a fake teleop data stream: it publishes the palm and fingertip rigid bodies at 120 Hz (like
`mocap_tf` for real mocap), engages the clutch once the arm is at HOME, and moves each palm in its arm's own axes:
hold, 10 cm down, 15 cm forward (fingertip curls, gripper closes), back, up, gripper opens again.
`tools/fake_isaac.py` stands in for Isaac: sim clock on `/clock`, the rig's 21 joints starting at 0, following
`/dual_arm/isaac_joint_commands` with a short drive lag, published on `/dual_arm/isaac_joint_states`.
One command per terminal (no mocap driver needed: `mocap_bridge:=false`):
```bash
# a) RViz only, one arm
ros2 launch rebel_demo teleop.launch.py target:=rviz mocap_bridge:=false
python3 ~/dual_arm_control/src/rebel_mocap_teleop/tools/fake_hand.py

# b) RViz only, both arms (right hand half a cycle behind), repeating
ros2 launch rebel_demo teleop.launch.py target:=rviz rig:=both mocap_bridge:=false
python3 ~/dual_arm_control/src/rebel_mocap_teleop/tools/fake_hand.py --arm both --loop

# c) the full Isaac interface with the fake Isaac (topics, sim clock, joint names exactly as with Isaac)
python3 ~/dual_arm_control/src/rebel_mocap_teleop/tools/fake_isaac.py
ros2 launch rebel_demo teleop.launch.py target:=isaac rig:=both mocap_bridge:=false
python3 ~/dual_arm_control/src/rebel_mocap_teleop/tools/fake_hand.py --arm both --sim-time --loop

# d) real Isaac, fake hands: as c) without fake_isaac.py (Isaac playing, bridge scripts run)

# e) up / down / left / right instead of down / forward (both arms, 10 cm each way; --amp for more)
python3 ~/dual_arm_control/src/rebel_mocap_teleop/tools/fake_hand.py --arm both --pattern cross --loop     # add --sim-time with target:=isaac

# f) through the mocap converter too: publish /rigid_bodies like the OptiTrack driver (Linux, mocap4r2_msgs sourced)
ros2 launch rebel_demo teleop.launch.py target:=rviz rig:=both                       # mocap_bridge on (default)
python3 ~/dual_arm_control/src/rebel_mocap_teleop/tools/fake_hand.py --arm both --pattern cross --output rigid_bodies --loop
```
`--pattern cross`: hold, up, back, down, back, left, back, right, back, each `--amp` (0.10 m) in the arm's own axes
(left = the arm base's +y), gripper open. Measured on both arms (RViz and fake Isaac): +-10.0 cm and +-20.0 cm exactly
on the commanded axis, under 0.5 cm on the others, back at HOME within 1 mm, no warnings.
**Find the joint limit: progressively lower hand.** `--descend 0.05` goes 5 cm down and `--descend-forward` (default
0.3) x 5 cm forward, holds 2 s, and repeats, deeper each time (prints `depth NN cm, forward NN cm`) up to `--max-down`
(0.70 m), then returns. A hand that reaches lower also reaches out, so the default path leans forward; use
`--descend-forward 0` for straight down.
```bash
ros2 launch rebel_demo teleop.launch.py target:=rviz mocap_bridge:=false
python3 ~/dual_arm_control/src/rebel_mocap_teleop/tools/fake_hand.py --descend 0.05
```
Reach of the arm (elbow-up bound on, tool relative to HOME, which is 35 cm in front of the base and 30 cm up): straight
down it follows exactly to 60 cm below HOME (joint2 140 deg and joint3 15 deg are reached together); the further forward
it is, the less deep it goes (x = 0.20 m: 65 cm, 0.35 m: 60 cm, 0.45 m: 53 cm, 0.55 m: 40 cm below HOME). With the
default lean (0.3) the tool follows to about 45 cm down / 14 cm forward and then reaches its limit. The default
`workspace_min` z = -0.45 m is below everything the arm can reach, so a joint limit (not the box) stops it. Teleop warns,
at most once per second:
```
joint limit reached: arm_left_joint3 at lower elbow-up bound 15 deg - target 5 mm out of reach, moving to the closest reachable pose
joint limit reached: arm_left_joint2 at upper limit 140 deg, arm_left_joint3 at lower elbow-up bound 15 deg - target 119 mm out of reach, moving to the closest reachable pose
joint limits: target back in range          (when the hand comes back)
target clamped by workspace_min z = -0.45 m (hand is 5.0 cm beyond) - raise the limit if the arm can go there
```
The first two appear when the IK is more than `limit_warn_mm` (5) off the target while a joint sits at a limit; the
arm then goes to the closest pose it can reach (the IK pins the joint at its limit and re-solves the others, so the arm
slides along the limit instead of jumping to a corner posture) and follows again as soon as the hand returns. The last
one means the workspace box, not a joint, stopped the target. Both arms warn independently (`rig:=both`, node names
in the log).

Options of `fake_hand.py`: `--arm left|right|both`, `--pattern reach|cross` (+ `--amp`), `--output tf|rigid_bodies`, `--loop`, `--down` / `--forward` [m], `--seg` [s per move],
`--descend STEP` / `--descend-forward RATIO` / `--max-down` [m],
`--sim-time` (required with `target:=isaac`: teleop runs on Isaac's clock there), `--unstable` (2 mm jitter,
drop-outs, frozen frames, marker glitches; or `--jitter`, `--dropouts`, `--freezes`, `--outliers`), `--rotate-finger`
(with `gripper_mode:=orientation`). On this Mac the workspace is `~/Downloads/rebel_teleop_ws` instead of `~/dual_arm_control`.

## 7. Real robot (publish the commands for another node to consume)
Teleop does not talk to hardware itself. It publishes **absolute joint positions** (radians, not deltas) as one
`sensor_msgs/JointState` at 60 Hz on `/dual_arm/isaac_joint_commands` (change it with `command_topic:=/your/topic`).
Your robot driver (or a small relay node) subscribes to that topic and forwards the values to the arm's controller.
- Names: `arm_left_joint1..6`, `arm_right_joint1..6` (rad) and `arm_<side>_xeg32_left_carriage_joint` /
  `..._right_carriage_joint` (jaw opening, m). Only `name` and `position` are filled in.
- Without Isaac, pass `use_sim_time:=false`. Teleop also needs a joint-state feedback stream on
  `/dual_arm/isaac_joint_states` (start pose, and the rig TF); if the robot does not publish one, add `fake_state:=true`
  (states = the commanded joints, the rest 0). With a real state publisher leave `fake_state` off, two publishers clash.

```bash
export ROS_DOMAIN_ID=11
source ~/mocap_ws/install/setup.bash
source ~/dual_arm_control/install/setup.bash
# robot powered up at all zeros (straight up); no joint states from it, so fake_state; it ramps to home_deg first
ros2 launch rebel_demo teleop.launch.py target:=isaac rig:=both rviz:=false scale:=0.5 cutoff_hz:=6.0 \
  left_hand_frame:=2 left_gripper_frame:=4 \
  right_hand_frame:=1 right_gripper_frame:=3 gripper_mode:=orientation \
  orientation:=semi max_joint_speed_deg:=15 workspace_max:="0.7 0.7 1.4" \
  use_sim_time:=false fake_state:=true home_deg:="0 0 100 0 50 0"
# robot already AT the pose you want (no start move): drop home_deg and add skip_home:=true instead
```
**Shortcut:** `scripts/run.sh` does the sourcing (ROS, mocap workspace, this workspace; domain 11) and runs these commands for you,
from any folder: `real` (the command below), `isaac`, `keyboard`, `calibrate`, `driver`, `activate`, `build`, `env`. Extra arguments
override the defaults (`run.sh real scale:=0.5 max_joint_speed_deg:=15`), and `ROS_DOMAIN_ID=12 run.sh real` changes the domain.
```bash
~/dual_arm_control/src/rebel_mocap_teleop/scripts/run.sh real
```

**Best known command so far (real robot, both arms)** - tuned on the real setup; start from this one:
```bash
export ROS_DOMAIN_ID=11
source ~/mocap_ws/install/setup.bash
source ~/dual_arm_control/install/setup.bash
ros2 launch rebel_demo teleop.launch.py target:=isaac rig:=both rviz:=false scale:=1.0 cutoff_hz:=6.0 \
  left_hand_frame:=2 left_gripper_frame:=4 \
  right_hand_frame:=1 right_gripper_frame:=3 gripper_mode:=orientation \
  orientation:=semi max_joint_speed_deg:=45 workspace_max:="0.7 0.7 1.4" \
  robot_ypr:="1.5708 0 0" home_deg:="0 0 60 0 50 0" \
  use_sim_time:=false fake_state:=true
```
Same line without the backslashes: `ros2 launch rebel_demo teleop.launch.py target:=isaac rig:=both rviz:=false scale:=1.0 cutoff_hz:=6.0 left_hand_frame:=2 left_gripper_frame:=4 right_hand_frame:=1 right_gripper_frame:=3 gripper_mode:=orientation orientation:=semi max_joint_speed_deg:=45 workspace_max:="0.7 0.7 1.4" robot_ypr:="1.5708 0 0" home_deg:="0 0 60 0 50 0" use_sim_time:=false fake_state:=true`
- `robot_ypr:="1.5708 0 0"` turns the robot frame 90 deg about the vertical axis so "forward" matches how you stand
  (flip the sign if it goes the wrong way); `home_deg` is the start pose of both arms (joints 1-6, degrees).
- Always put the quotes around values with spaces, and keep the whole command on one logical line (`\` at the line ends).
- First run on a new robot or setup: lower `scale:=0.5` and `max_joint_speed_deg:=15`, keep the e-stop in reach.

Check what is published (the merged topic must show **one** publisher, `joint_merger`):
```bash
ros2 topic hz   /dual_arm/isaac_joint_commands
ros2 topic echo /dual_arm/isaac_joint_commands --once
ros2 topic info /dual_arm/isaac_joint_commands -v
ros2 topic echo /rebel_teleop_left/joint_commands --once --field position     # one arm, 6 values
```
Publish by hand to test the receiving side (stop teleop first, otherwise two publishers fight):
```bash
# move left joint 1 to 0.3 rad (about 17 deg)
ros2 topic pub -r 10 /dual_arm/isaac_joint_commands sensor_msgs/msg/JointState \
  "{name: [arm_left_joint1], position: [0.3]}"
# several joints at once (radians)
ros2 topic pub -r 10 /dual_arm/isaac_joint_commands sensor_msgs/msg/JointState \
  "{name: [arm_left_joint1, arm_left_joint2, arm_left_joint3], position: [0.0, 0.2, 0.4]}"
```
Safety: there is no collision checking. Use `scale:=0.5 max_joint_speed_deg:=15` first, keep the e-stop in reach, and
check the receiver's joint limits and zero positions match the rig model (joint2 -80..140 deg, joint3 -80..140 deg,
joint5 +-95 deg, others +-179 deg; teleop also keeps joint3 >= 15 deg).

## Quick start with `scripts/run.sh` (real robot or Isaac Sim)
`scripts/run.sh` sources ROS, the mocap workspace and this workspace (domain 11 by default), removes conda from `PATH`, and runs
the command. Run it by its path from any folder (`chmod +x scripts/run.sh` once if it says "Permission denied"). Terminals, in order:

| Terminal | Command | Notes |
|---|---|---|
| 1 | `~/dual_arm_control/src/rebel_mocap_teleop/scripts/run.sh driver` | OptiTrack driver; wait for `Configured!`, then (any terminal) `... run.sh activate` |
| 2 | `... run.sh real` (or `... run.sh isaac`) | wait for `at HOME` from both arms; keep it open. Isaac: start it from a shell with the same `ROS_DOMAIN_ID` and press Play first |
| 3 | `... run.sh keyboard` | click this terminal to focus it; SPACE engages / pauses / resumes both arms, `l` / `r` toggles one arm, `q` quits |
| once | `... run.sh calibrate` | hold both hands open while it runs; repeat after every restart of the launch |

The robot's own driver (the node that reads `/dual_arm/isaac_joint_commands`) must be running too. Extra arguments override the
defaults: `run.sh real scale:=0.5 max_joint_speed_deg:=15`. A shorter alias for `~/.bashrc`:
`alias rebel='~/dual_arm_control/src/rebel_mocap_teleop/scripts/run.sh'` (then `rebel driver`, `rebel real`, `rebel keyboard`, ...).
`run.sh help` lists all commands; `run.sh env` checks that ROS, `rebel_demo` and `mocap4r2_msgs` are found.

## Troubleshooting
| symptom | fix |
|---|---|
| driver log `... not connected :(` | Motive streaming off or wrong Local Interface; firewall; IPs in the YAML. Relaunch the driver after fixing (it connects only at launch) |
| `ping` to the Motive PC says Destination Host Unreachable | the two PCs are not on the same router/network |
| `message type 'mocap4r2_msgs/msg/RigidBodies' is invalid` | `source ~/mocap_ws/install/setup.bash` |
| `mocap_tf` dies: `No module named 'mocap4r2_msgs'` / `mocap4r2_msgs not found` | same: `source ~/mocap_ws/install/setup.bash` before `source ~/dual_arm_control/install/setup.bash`, then relaunch (needed also for `target:=isaac`) |
| `colcon build` fails: `No module named 'catkin_pkg'`, log shows `miniconda3/bin/python3` | `conda deactivate`, then rebuild (or `--cmake-args -DPython3_EXECUTABLE=/usr/bin/python3`) |
| RViz `could not connect to display` | started over SSH: use `rviz:=false`, run RViz on a machine with a screen |
| RViz loads someone else's robot | another project uses the same `ROS_DOMAIN_ID`: pick another number everywhere |
| `ros2 service call` hangs on `waiting for service`, or prints `sequence size exceeds remaining buffer` | this terminal is on a different ROS setup than teleop (distro, `RMW_IMPLEMENTATION` or `ROS_DOMAIN_ID`). Compare with the running teleop: `tr '\0' '\n' < /proc/$(pgrep -n -f rebel_demo/teleop)/environ \| grep -E '^(ROS_\|RMW_)'`, set the same values, then `ros2 daemon stop` |
| teleop log `holding: no TF arm_left_base_link <- rigid_body_N`, commands stay 0 although engaged | nobody publishes `/dual_arm/isaac_joint_states` (no Isaac in this `ROS_DOMAIN_ID`, or a real robot without a state publisher), so the rig's TF is split. Add `fake_state:=true` (states = commanded joints, others 0), or publish the real joint states there |
| `TF_OLD_DATA ignoring data from the past ... at time 4.4` (all frames, all nodes) | two clocks or two publishers: Isaac's sim time mixed with PC time. With Isaac do not pass `use_sim_time:=false` / `fake_state:=true`; restart the launch after stopping or resetting Isaac; `ros2 topic info /dual_arm/isaac_joint_states -v` must show one publisher |
| teleop and Isaac do not see each other | different `ROS_DOMAIN_ID` (Isaac with none = 0): export the same number in every terminal and in the shell that starts Isaac Sim |
| `colcon build` makes `build/ install/ log/` inside `src/` | you ran it in the wrong folder: delete them, `cd ~/dual_arm_control`, build again |
| `No module named 'xacro'` at launch | `sudo apt install ros-$(ls /opt/ros)-xacro` |
| `package 'igus_rebel_description' not found` | copy it into `src/` if missing, then build it (`colcon build --packages-select igus_rebel_description`) and re-source `install/setup.bash` (section 3) |
| RViz: `No transform from ... to world` for the arm links | nobody publishes the rig's joint states: Isaac not playing / bridge scripts not run (`ros2 topic hz /dual_arm/isaac_joint_states`), or in `target:=rviz` the `joint_merger` node died (see its traceback) |
| `No package metadata was found for rebel_demo` | broken install from a failed build: `rm -rf build/rebel_demo install/rebel_demo`, rebuild |
| robot does not move after engage | `ros2 run tf2_ros tf2_echo arm_left_base_link rigid_body_1` must change as you move; check `hand_frame` |

## Licence and credits
- Robot model: `igus_rebel_description` from the igus mesh LOD bundle, not included (dual-arm rig, rebel2 arm, XEG-32, decimated meshes).
- Mocap driver (not included): [MOCAP4ROS2-Project/mocap4ros2_optitrack](https://github.com/MOCAP4ROS2-Project/mocap4ros2_optitrack).
- Kinematics, IK, `mocap_tf` and `teleop` nodes: written for this project (numpy + rclpy).
- Not suitable for a real robot as is: no collision checking. Use low speed limits and an e-stop, or MoveIt Servo.

## Known issues
- IK targets `tool0`, not the point between the XEG jaws; with `orientation:=true` the fingertips swing.
- The wrist side (joint5 sign) is not constrained; only the elbow is (joint3 >= 15 deg).
- XEG open / closed carriage values are the model joint limits (16.5 mm travel per jaw): check against the datasheet.
