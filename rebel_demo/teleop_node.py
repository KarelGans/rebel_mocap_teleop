"""teleop node - your hand (OptiTrack rigid body, via mocap4ros2_optitrack TF) drives the igus.

Pipeline, every cycle (default 60 Hz):
  1. TF lookup  base_frame <- hand_frame          (hand pose in the robot base frame)
  2. tracking check: missing / stale / frozen pose -> HOLD (robot keeps its last command)
  3. LOW-PASS filter on the hand pose            (position: 1st-order; orientation: slerp)
  4. clutch + mapping: relative (the hand moves the tool by the same amount x scale), workspace clamp
  5. real-time IK  target flange pose -> 6 joint angles, seeded with the last command
  6. joint speed limit                            (max_joint_speed_deg; all joints scaled together, keeps the path)
  7. publish sensor_msgs/JointState on command_topic (/dual_arm/isaac_joint_commands for the Isaac rig)

Clutch : call the service  ~/engage  (std_srvs/SetBool)
  ros2 service call /rebel_teleop/engage std_srvs/srv/SetBool "{data: true}"    # start following
  ros2 service call /rebel_teleop/engage std_srvs/srv/SetBool "{data: false}"   # stop, reposition hand
  ros2 service call /rebel_teleop/toggle std_srvs/srv/Trigger                     # pause <-> resume
  ros2 run rebel_demo teleop_keyboard                                              # SPACE = toggle (own terminal)
Pause = the robot holds its last command; resume = the robot's last pose and your hand's pose at that moment
become the new reference, so you continue from where it stopped (the gripper node is not affected).
Topic ~/engaged (std_msgs/Bool, latched) shows the current state.
While engaged:  robot_target = robot_pose_at_engage + scale_trans * (hand_now - hand_at_engage)
                (orientation:=true also rotates it by scale_rot x the hand's rotation since engage)

Parameters (defaults in brackets)
  base_frame [arm_left_base_link]  hand_frame [HAND]          TF frames (hand = rigid body name in mocap config)
  rate_hz [60]
  scale_trans [1.0]       gain on the hand's translation since engage; `scale` is an alias
  scale_rot [1.0]         gain on the hand's rotation since engage: the wrist rotation with orientation:=true, the joint 6
                          roll with orientation:=semi (unused with orientation:=false)
  orientation [false]     false = position only (wrist free), true = full pose with robot_remote's method
                          (github.com/Patrick-zhuyanxun/robot_remote): task-priority IK (position first, the orientation
                          only with what is left, never at the cost of the position), a rate-limited command that may lead
                          the commanded joints by at most max_tool_lead_m / _deg; at a wrist limit the orientation stops
  rot_axis_sign ["1 1 1"] orientation:=true: sign of the hand's rotation about the robot base x / y / z axes; "-1 1 1"
                          reverses only the roll about x (the real setup: the tool otherwise twists against your hand)
  max_tool_rot_speed_deg [90]  orientation:=true: the commanded tool rotation turns at most this fast (deg/s)
  max_tool_lead_m [0.05]  max_tool_lead_deg [15]  orientation:=true: anti-windup - how far the command may run ahead of
                          the pose of the commanded joints
  limit_follow [true]     orientation:=true: at a joint limit (or the workspace box) forget the hand motion / rotation the
                          arm cannot follow, so it follows again at once when the hand moves another way (false: wait
                          until the hand is back where the arm stopped, as robot_remote does)
  roll_follow [false]     position only + joint 6 follows the hand's roll about the tool axis (launch: orientation:=semi)
  roll_sign [1.0]         joint6 = joint6 at engage + sign * scale_rot * hand twist; sign -1 flips the direction
  cutoff_hz [4.0]         low-pass cutoff for the hand pose
  max_joint_speed_deg [45]  stale_timeout [0.2]      s without a new pose -> hold
  workspace_min [-0.7,-0.7,-0.45]  workspace_max [0.7,0.7,0.95]   target clamp in base_frame [m]
  command_topic [/dual_arm/isaac_joint_commands]   topic Isaac's ROS2 Subscribe Joint State node listens to
                          (the launch file gives each teleop node its own topic and joint_merger forwards them)
  publish_joint_states [false]   true = also publish /joint_states (stand-alone testing; the launch uses joint_merger)
  feedback_from_joint_states [false]  true = seed IK from Isaac's joint states instead of last command
  limit_warn_mm [5]       warn "joint limit reached" when the IK is further than this from the target and a joint
                          sits at a limit (the elbow-up bound counts); also warns when the workspace box clamps the target
  reanchor_after [2.0]    s of lost tracking before the clutch re-anchors (shorter gaps: hold, keep the anchor)
  max_hand_speed [2.0]    m/s; a sample jumping further than this (+1 cm) is a marker glitch and ignored for up to 0.1 s
  max_tool_speed [0.5]    m/s; the IK target never moves faster (smooth catch-up after a tracking gap)
  home_wait [1.0]         s to wait for joint states at start; the robot then moves from there to HOME
  home_deg ["0 0 60 0 50 0"]  HOME pose, 6 joint angles in degrees (launch: home_deg:="0 0 100 0 50 0"); the start move
                          goes there and the IK's posture preference pulls towards it
  skip_home [false]       true = do not move to HOME at start: take the measured joints (all zeros if none arrive within
                          home_wait) as the start pose and allow ~/engage at once. For a robot that already is at HOME.

Startup / HOME: the node starts from the measured joints (all zeros if none arrive: a robot without joint feedback is
assumed to be straight up) and drives to HOME in joint space at max_joint_speed_deg; engage is refused until it is there. Service ~/home (std_srvs/Trigger) does the same later:
  ros2 service call /rebel_teleop/home std_srvs/srv/Trigger
  joint_prefix [arm_left_]  joint names = prefix + joint1..joint6 (arm_left_ / arm_right_ on the dual-arm rig)
  joint_names ['']        6 comma-separated names (base to wrist), overrides joint_prefix, e.g. "a_1,a_2,...,a_6"
  joint_states_topic [/dual_arm/isaac_joint_states]   where Isaac publishes the measured joints (feedback only)
  max_lead_deg [0]        the command is never more than this far ahead of the measured joints on joint_states_topic, so a
                          slower follower / real arm never falls far behind (HOME and teleop wait for it); 0 = off
  feedback_timeout [0.5]  s; measured joints older than this are ignored (the lead limit is then inactive, with a warning)
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .kinematics import (JOINT_NAMES, LOWER, UPPER, drop_unreachable, drop_unreachable_rot, fk, flip_rotation, ik,
                         ik_priority, limit_lead, limits_hit, parse_pose_deg, quat_to_matrix, scale_into_lead,
                         scale_rotation, toward_point, toward_rot, twist_about)


def matrix_to_quat(R):
    """3x3 -> [x, y, z, w]"""
    t = np.trace(R)
    if t > 0:
        s = 2 * math.sqrt(t + 1)
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, s / 4])
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2 * math.sqrt(1 + R[i, i] - R[j, j] - R[k, k])
    q = np.zeros(4)
    q[i] = s / 4
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return q


def slerp(q0, q1, a):
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    d = float(np.dot(q0, q1))
    if d < 0:
        q1, d = -q1, -d
    if d > 0.9995:
        q = q0 + a * (q1 - q0)
        return q / np.linalg.norm(q)
    th = math.acos(d)
    return (math.sin((1 - a) * th) * q0 + math.sin(a * th) * q1) / math.sin(th)


class PoseLowPass:
    """First-order low-pass on position + slerp on orientation; alpha from cutoff and dt."""

    def __init__(self, cutoff_hz):
        self.fc = cutoff_hz
        self.p = None
        self.q = None

    def reset(self, p, q):
        self.p, self.q = np.array(p, float), np.array(q, float)

    def __call__(self, p, q, dt):
        if self.p is None or self.fc <= 0:
            self.reset(p, q)
            return self.p, self.q
        a = 1.0 - math.exp(-2 * math.pi * self.fc * dt)
        self.p = self.p + a * (np.asarray(p) - self.p)
        self.q = slerp(self.q, q, a)
        return self.p, self.q


class Teleop(Node):
    def __init__(self):
        super().__init__("rebel_teleop")
        d = self.declare_parameter
        self.base_frame = d("base_frame", "arm_left_base_link").value
        self.hand_frame = d("hand_frame", "HAND").value
        self.rate = float(d("rate_hz", 60.0).value)
        scale_alias = float(d("scale", 1.0).value)                        # old name of scale_trans
        self.scale_trans = float(d("scale_trans", scale_alias).value)
        self.scale_rot = float(d("scale_rot", 1.0).value)
        self.use_orientation = bool(d("orientation", False).value)
        self.roll_follow = bool(d("roll_follow", False).value) and not self.use_orientation
        self.roll_sign = float(d("roll_sign", 1.0).value)
        self.roll_axis, self.q6_ref, self.twist_prev, self.twist_acc = None, 0.0, 0.0, 0.0
        self.filter = PoseLowPass(float(d("cutoff_hz", 4.0).value))
        self.vmax = math.radians(float(d("max_joint_speed_deg", 45.0).value))
        self.stale = float(d("stale_timeout", 0.2).value)
        self.ws_min = np.array(d("workspace_min", [-0.7, -0.7, -0.45]).value, float)
        self.ws_max = np.array(d("workspace_max", [0.7, 0.7, 0.95]).value, float)
        self.limit_warn = float(d("limit_warn_mm", 5.0).value) * 1e-3
        self.limit_active = False
        self.reanchor_after = float(d("reanchor_after", 2.0).value)
        self.max_hand_speed = float(d("max_hand_speed", 2.0).value)
        self.max_tool_speed = float(d("max_tool_speed", 0.5).value)
        # orientation:=true (robot_remote's method): the commanded rotation turns toward the hand's at most this fast, and
        # the command may lead what the commanded joints reach by at most max_tool_lead_m / _deg (anti-windup)
        self.max_rot_speed = math.radians(float(d("max_tool_rot_speed_deg", 90.0).value))
        self.tool_lead_m = float(d("max_tool_lead_m", 0.05).value)
        self.tool_lead = math.radians(float(d("max_tool_lead_deg", 15.0).value))
        self.limit_follow = bool(d("limit_follow", True).value)
        signs = [float(v) for v in str(d("rot_axis_sign", "1 1 1").value).replace(",", " ").split()]
        if len(signs) != 3 or any(s not in (1.0, -1.0) for s in signs):
            raise ValueError(f"rot_axis_sign needs 3 values, each 1 or -1 (x y z of the robot base), got {signs}")
        self.rot_signs = tuple(signs)
        self.cmd_R = None                                                 # commanded rotation (orientation:=true)
        self.good_p, self.good_t, self.glitch_since = None, None, None   # outlier gate state
        self.target = None                                                # last IK target position (rate limit)
        self.home_wait = float(d("home_wait", 1.0).value)
        self.skip_home = bool(d("skip_home", False).value)
        self.max_lead = math.radians(float(d("max_lead_deg", 0.0).value))   # 0 = off
        self.fb_timeout = float(d("feedback_timeout", 0.5).value)
        self.home = parse_pose_deg(d("home_deg", "0 0 60 0 50 0").value)    # HOME [rad]; raises on bad input
        self.engaged = bool(d("start_engaged", False).value)
        topic = d("command_topic", "/dual_arm/isaac_joint_commands").value
        self.pub_js = bool(d("publish_joint_states", False).value)
        self.use_feedback = bool(d("feedback_from_joint_states", False).value)
        prefix = d("joint_prefix", "arm_left_").value
        custom = [n.strip() for n in d("joint_names", "").value.split(",") if n.strip()]
        if custom and len(custom) != 6:
            raise ValueError(f"joint_names needs 6 comma-separated names, got {len(custom)}: {custom}")
        self.joint_names = custom or [prefix + n for n in JOINT_NAMES]
        js_topic = d("joint_states_topic", "/dual_arm/isaac_joint_states").value

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self, spin_thread=True)   # TF never waits for the IK
        self.pub_cmd = self.create_publisher(JointState, topic, 10)
        self.pub_state = self.create_publisher(JointState, "/joint_states", 10) if self.pub_js else None
        self.pub_target = self.create_publisher(PoseStamped, "~/target", 10)
        if not self.pub_js:                                # measured joints: start pose, optional IK seed
            self.create_subscription(JointState, js_topic, self.on_js, 10)
        self.create_service(SetBool, "~/engage", self.on_engage)
        self.create_service(Trigger, "~/home", self.on_home)
        self.create_service(Trigger, "~/toggle", self.on_toggle)
        self.pub_engaged = self.create_publisher(
            Bool, "~/engaged", QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                          durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.q_cmd = None                  # last commanded joints (set from the measured joints at start)
        self.q_meas = None
        self.t_meas = None                 # receive time [s] of q_meas
        self.t_start = None
        self.homing = True                 # drive to HOME before teleop can engage
        self.lost_since = None
        self.last_stamp = None
        self.last_raw = None
        self.frozen_since = None
        self.why = ""
        self.hand_ref = None               # (p, R) of the hand at engage
        self.robot_ref = None              # flange pose at engage
        self.dt = 1.0 / self.rate
        self.create_timer(self.dt, self.step)
        self.get_logger().info(
            f"teleop: {self.hand_frame} -> {self.base_frame} ({self.joint_names[0]}..), "
            f"{'full pose' if self.use_orientation else 'position + joint6 roll' if self.roll_follow else 'position only'}"
            f", {self.rate:.0f} Hz -> {topic}"
            + ("" if self.engaged else "  (NOT engaged: call ~/engage true, or press SPACE in teleop_keyboard)"))
        self.pub_engaged.publish(Bool(data=self.engaged))

    # ------------------------------------------------------------------ inputs
    def on_js(self, msg):
        if all(n in msg.name for n in self.joint_names):
            self.q_meas = np.array([msg.position[msg.name.index(n)] for n in self.joint_names])
            self.t_meas = self.get_clock().now().nanoseconds * 1e-9

    def set_engaged(self, on):
        """Clutch in / out. Either way the reference is retaken on the next cycle: robot_ref = fk(q_cmd) (the last
        commanded pose) and hand_ref = the hand's pose at that moment, so a resume continues from where it stopped.
        Returns (success, message); engaging is refused while the arm moves to HOME."""
        if on and self.homing:
            msg = "moving to HOME - engage again when 'at HOME' is logged"
            self.get_logger().warn(msg)
            return False, msg
        self.engaged = bool(on)
        self.hand_ref = None                # re-anchor on the next cycle
        self.target = self.cmd_R = None
        self.pub_engaged.publish(Bool(data=self.engaged))
        msg = "engaged (RUNNING)" if self.engaged else "released (PAUSED, robot holds)"
        self.get_logger().info(msg)
        return True, msg

    def on_engage(self, req, res):
        res.success, res.message = self.set_engaged(req.data)
        return res

    def on_toggle(self, req, res):
        res.success, res.message = self.set_engaged(not self.engaged)
        return res

    def on_home(self, req, res):
        self.engaged, self.homing, self.hand_ref, self.target, self.cmd_R = False, True, None, None, None
        self.pub_engaged.publish(Bool(data=False))
        res.success, res.message = True, "moving to HOME (disengaged)"
        self.get_logger().info(res.message)
        return res

    def drive_home(self):
        """One cycle of the joint-space move to HOME: all joints arrive together, none faster than vmax."""
        if self.q_cmd is None:                                     # wait for the measured start pose
            now = self.get_clock().now().nanoseconds * 1e-9
            self.t_start = self.t_start or now
            if self.q_meas is None and now - self.t_start < self.home_wait:
                return
            # start from the measured pose as it is (may be outside the IK bounds, e.g. Isaac's straight-up start with
            # joint3 = 0 < the 15 deg elbow bound); clipping here would jump joint3 in one cycle
            self.q_cmd = (self.q_meas if self.q_meas is not None else np.zeros(6)).copy()
            self.get_logger().info("start pose %s deg%s -> moving to HOME %s deg" % (
                np.degrees(self.q_cmd).round(1).tolist(), "" if self.q_meas is not None else " (assumed: no joint states)",
                np.degrees(self.home).round(1).tolist()))
        if self.skip_home:                                         # robot is already where it should be: no move
            self.homing = False
            self.get_logger().info("skip_home: starting from the current pose - call ~/engage true to start")
            self.publish(self.q_cmd)
            return
        d = self.home - self.q_cmd
        n = np.max(np.abs(d)) / (self.vmax * self.dt)              # cycles the slowest joint needs
        self.q_cmd = self.home.copy() if n <= 1 else self.q_cmd + d / n
        self.q_cmd = self.hold_lead(self.q_cmd)                    # slow robot: HOME waits for it
        if np.array_equal(self.q_cmd, self.home):                  # arrived (only if the lead clamp left it at HOME)
            self.homing = False
            self.get_logger().info("at HOME - call ~/engage true to start")
        self.publish(self.q_cmd)

    def read_hand(self):
        """Hand pose in base_frame, or None if missing / stale / frozen."""
        try:
            t = self.tf.lookup_transform(self.base_frame, self.hand_frame, Time())
        except TransformException as e:
            self.why = f"no TF {self.base_frame} <- {self.hand_frame} ({type(e).__name__})"
            return None
        stamp = Time.from_msg(t.header.stamp).nanoseconds * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - stamp > self.stale:
            self.why = f"hand pose stale ({now - stamp:.2f} s old)"
            return None
        tr, rq = t.transform.translation, t.transform.rotation
        raw = (tr.x, tr.y, tr.z, rq.x, rq.y, rq.z, rq.w)
        if raw == self.last_raw:                           # bit-identical = frozen (marker loss)
            self.frozen_since = self.frozen_since or now
            if now - self.frozen_since > self.stale:
                self.why = "hand pose frozen (bit-identical, marker loss?)"
                return None
        else:
            self.frozen_since = None
        self.last_raw = raw
        p = np.array(raw[:3])
        # outlier gate: a jump faster than a hand can move is a marker glitch (swap / ghost) - skip it; if it
        # persists for 0.1 s it is real (e.g. hand moved during a drop-out) and accepted
        if self.good_p is not None:
            allowed = self.max_hand_speed * max(now - self.good_t, self.dt) + 0.01
            if np.linalg.norm(p - self.good_p) > allowed:
                self.glitch_since = self.glitch_since or now
                if now - self.glitch_since < 0.1:
                    self.why = "marker glitch (pose jump) ignored"
                    return "glitch"
        self.good_p, self.good_t, self.glitch_since = p, now, None
        return p, np.array(raw[3:])

    # ------------------------------------------------------------------ main loop
    def step(self):
        if self.homing:
            self.drive_home()
            return
        hand = self.read_hand()
        now = self.get_clock().now().nanoseconds * 1e-9
        if isinstance(hand, str):                          # glitch sample: hold this cycle, keep everything
            self.publish(self.q_cmd)
            return
        if hand is None:
            self.lost_since = self.lost_since or now
            if now - self.lost_since > self.reanchor_after:  # really lost: re-anchor when tracking returns
                if self.hand_ref is not None and self.engaged:
                    self.get_logger().warn(f"tracking lost: {self.why} - will re-anchor")
                self.hand_ref = None
                self.filter.p = None                       # restart the filter (no glide from a stale pose)
                self.good_p = None
            else:                                          # short gap: hold, keep the anchor
                self.get_logger().warn(f"holding: {self.why}", throttle_duration_sec=2.0)
            self.publish(self.q_cmd)                       # HOLD
            return
        self.lost_since = None
        p_f, q_f = self.filter(hand[0], hand[1], self.dt)  # 3. low-pass
        R_f = quat_to_matrix(q_f)
        seed = self.q_meas if (self.use_feedback and self.q_meas is not None) else self.q_cmd

        if not self.engaged:
            self.publish(self.q_cmd)
            return
        # 4. mapping
        if self.hand_ref is None:                          # anchor: hand now <-> robot now
            self.hand_ref = (p_f.copy(), R_f.copy())
            self.robot_ref = fk(self.q_cmd)
            self.cmd_R = self.robot_ref[:3, :3].copy()
            self.roll_axis = self.robot_ref[:3, 2].copy()          # tool0 z = joint6 axis, base frame
            self.q6_ref, self.twist_prev, self.twist_acc = float(self.q_cmd[5]), 0.0, 0.0
            self.get_logger().info("anchored: tool at (%.3f, %.3f, %.3f) m <-> hand at (%.3f, %.3f, %.3f) m"
                                   % (*self.robot_ref[:3, 3], *p_f))
        T = self.robot_ref.copy()
        T[:3, 3] = self.robot_ref[:3, 3] + self.scale_trans * (p_f - self.hand_ref[0])
        if self.use_orientation:
            # the hand's rotation since engage (robot base axes), each axis optionally reversed (rot_axis_sign)
            T[:3, :3] = scale_rotation(flip_rotation(R_f @ self.hand_ref[1].T, self.rot_signs),
                                       self.scale_rot) @ self.robot_ref[:3, :3]
        # workspace box, widened to contain the anchor pose: engaging never moves the robot
        lo, hi = self.ws_min, self.ws_max
        lo, hi = np.minimum(lo, self.robot_ref[:3, 3]), np.maximum(hi, self.robot_ref[:3, 3])
        wanted = T[:3, 3].copy()
        T[:3, 3] = np.clip(wanted, lo, hi)
        beyond = wanted - T[:3, 3]                         # > 0: past the max, < 0: past the min
        for ax in np.nonzero(np.abs(beyond) > 1e-3)[0]:
            side, bound = ("max", hi[ax]) if beyond[ax] > 0 else ("min", lo[ax])
            self.get_logger().warn(
                f"target clamped by workspace_{side} {'xyz'[ax]} = {bound:+.2f} m "
                f"(hand is {abs(beyond[ax]) * 100:.1f} cm beyond) - raise the limit if the arm can go there",
                throttle_duration_sec=2.0)
        # target speed limit: after a gap the hand may be elsewhere; the tool follows smoothly, never jumps
        if self.target is not None:
            step = T[:3, 3] - self.target
            n = np.linalg.norm(step)
            if n > self.max_tool_speed * self.dt:
                T[:3, 3] = self.target + step * (self.max_tool_speed * self.dt / n)
        self.target = T[:3, 3].copy()
        if self.use_orientation:
            # robot_remote's method: the commanded rotation turns toward the hand's at <= max_tool_rot_speed_deg, and the
            # command may lead the pose of the commanded joints by at most max_tool_lead_m / _deg (anti-windup: a hand
            # asking for more than the arm can do never pulls the IK towards a far-away configuration)
            R_want = T[:3, :3].copy()
            self.cmd_R = toward_rot(R_want if self.cmd_R is None else self.cmd_R, R_want, self.max_rot_speed * self.dt)
            ach = fk(self.q_cmd)
            self.target = toward_point(ach[:3, 3], self.target, self.tool_lead_m)
            self.cmd_R = toward_rot(ach[:3, :3], self.cmd_R, self.tool_lead)
            T[:3, 3], T[:3, :3] = self.target, self.cmd_R
        self.publish_target(T)

        # 5. real-time IK, warm-started from the last command; full pose: task-priority IK (position first, robot_remote)
        q_rest = self.home
        if self.roll_follow and self.roll_axis is not None:
            q_rest = self.home.copy()
            q_rest[5] = seed[5]                            # no pull of joint 6 towards HOME: the hand roll owns it
        if self.use_orientation:
            q_ik, pe, re = ik_priority(T, seed)
        else:
            q_ik, pe, re = ik(T, seed, w_rot=0.0, iters=25, q_rest=q_rest)
        if self.roll_follow and self.roll_axis is not None:
            # joint 6 spins about the tool axis, so it cannot change the tool position: drive it directly from the
            # twist of the hand about that axis (tilts of the hand contribute ~0), continuous across +-180 deg
            twist = twist_about(R_f @ self.hand_ref[1].T, self.roll_axis)
            self.twist_acc += (twist - self.twist_prev + math.pi) % (2 * math.pi) - math.pi
            self.twist_prev = twist
            q_ik[5] = float(np.clip(self.q6_ref + self.roll_sign * self.scale_rot * self.twist_acc,
                                    LOWER[5], UPPER[5]))
        pinned = limits_hit(q_ik)
        follow = self.use_orientation and self.limit_follow       # keep following: orientation:=true only (this branch)
        # with keep following the unreachable part is dropped every cycle, so the IK is never far off: 2 mm / 1 deg count
        pos_blocked = bool(pinned) and pe > (0.002 if follow else self.limit_warn)
        rot_blocked = self.use_orientation and bool(pinned) and re > 1.0   # wrist at a limit: orientation left short
        hits = pinned if (pos_blocked or rot_blocked) else []
        if hits:                                           # a joint limit is what keeps the IK off the target
            self.limit_active = True
            what = ", ".join(
                f"{self.joint_names[i]} at {side} {'elbow-up bound' if (i == 2 and side == 'lower') else 'limit'} "
                f"{np.degrees(LOWER[i] if side == 'lower' else UPPER[i]):.0f} deg" for i, side in hits)
            short = f"{pe * 1000:.0f} mm" + (f" / {re:.0f} deg" if self.use_orientation else "")
            tail = ("hand motion beyond it is ignored, the arm follows again as soon as the hand moves another way"
                    if follow else f"target {short} out of reach, moving to the closest reachable pose")
            self.get_logger().warn(f"joint limit reached: {what} - {tail}", throttle_duration_sec=1.0)
        else:
            if self.limit_active and (not pinned if follow else pe <= self.limit_warn):
                self.limit_active = False
                self.get_logger().info("joint limits: target back in range")
            if pe > 0.02:
                self.get_logger().warn(f"target not reachable ({pe * 1000:.0f} mm off) - moving as close as possible",
                                       throttle_duration_sec=1.0)
        if follow and self.scale_trans > 0 and (pos_blocked or np.any(np.abs(beyond) > 1e-3)):
            # keep following: the hand motion past the limit (or the workspace box) is forgotten, so the target stays at
            # the closest reachable point and the arm follows at once when the hand moves any other way
            reached = fk(q_ik)[:3, 3]
            self.hand_ref = (drop_unreachable(self.hand_ref[0], wanted, reached, self.scale_trans), self.hand_ref[1])
            self.target = reached.copy()
        if follow and rot_blocked and self.scale_rot > 0:
            # the same for the orientation: the hand rotation past the limit is forgotten, the wrist turns back at once
            R_reached = fk(q_ik)[:3, :3]
            self.hand_ref = (self.hand_ref[0], drop_unreachable_rot(R_f, R_reached, self.robot_ref[:3, :3],
                                                                    self.scale_rot, self.rot_signs))
            self.cmd_R = R_reached.copy()
        # 6. joint speed limit: scale ALL joints by the same factor so the step keeps the IK's direction and the tool
        # stays on its path (clipping each joint on its own bends the motion when one joint saturates)
        dq = q_ik - self.q_cmd
        m = float(np.max(np.abs(dq)))                     # largest joint step this cycle [rad]
        max_step = self.vmax * self.dt
        if m > max_step:
            dq *= max_step / m
        q_next = np.clip(self.q_cmd + dq, LOWER, UPPER)
        if self.use_orientation:                           # full pose: the lead limit shortens the whole step, too
            q_next = self.hold_lead_path(self.q_cmd, q_next)
        self.q_cmd = self.hold_lead(q_next)                # robot behind: never more than max_lead ahead of it
        self.publish(self.q_cmd)                           # 7.

    def hold_lead_path(self, q_prev, q_new):
        """Step q_prev -> q_new shortened as a whole (scale_into_lead) so it stays within max_lead_deg of the fresh
        measured joints: the arm waits for the slow real arm and the tool keeps its path. q_new if off / no feedback."""
        age = self.get_clock().now().nanoseconds * 1e-9 - self.t_meas if self.t_meas is not None else None
        if self.max_lead <= 0 or self.q_meas is None or age > self.fb_timeout:
            return q_new                                   # hold_lead warns about missing joint states
        out = scale_into_lead(q_prev, q_new, self.q_meas, self.max_lead)
        if not np.allclose(out, q_new):
            lag = np.abs(q_new - self.q_meas)
            i = int(np.argmax(lag))
            self.get_logger().warn(
                f"robot is behind: waiting for it (joint {i + 1} {math.degrees(lag[i]):.0f} deg behind the command, "
                f"max_lead_deg {math.degrees(self.max_lead):.0f})", throttle_duration_sec=2.0)
        return out

    def hold_lead(self, q):
        """q limited to max_lead_deg ahead of the fresh measured joints (see limit_lead); q itself if off / no feedback."""
        if self.max_lead <= 0:
            return q
        age = self.get_clock().now().nanoseconds * 1e-9 - self.t_meas if self.t_meas is not None else None
        if self.q_meas is None or age > self.fb_timeout:
            self.get_logger().warn(
                "max_lead_deg is set but no fresh joint states - lead limit inactive"
                + ("" if age is None else f" (last {age:.1f} s ago)"), throttle_duration_sec=5.0)
            return q
        out = limit_lead(q, self.q_meas, self.max_lead)
        if not np.array_equal(out, q):
            lag = np.abs(q - self.q_meas)
            i = int(np.argmax(lag))
            self.get_logger().warn(
                f"robot is behind: command held within {math.degrees(self.max_lead):.0f} deg of the measured joints "
                f"(joint {i + 1} lags {math.degrees(lag[i]):.0f} deg)", throttle_duration_sec=2.0)
        return out

    def publish(self, q):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [float(v) for v in q]
        self.pub_cmd.publish(msg)
        if self.pub_state is not None:
            self.pub_state.publish(msg)

    def publish_target(self, T):
        m = PoseStamped()
        m.header.frame_id = self.base_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in T[:3, 3])
        qx, qy, qz, qw = matrix_to_quat(T[:3, :3])
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = (
            float(qx), float(qy), float(qz), float(qw))
        self.pub_target.publish(m)


def main():
    rclpy.init()
    node = Teleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
