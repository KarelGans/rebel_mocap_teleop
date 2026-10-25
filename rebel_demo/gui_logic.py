"""Logic of the rebel_ui window - no Qt and no ROS in here, so it can be unit-tested.

  * profiles: all teleop settings (what you would type on the launch line), saved as JSON under ~/.config
  * build_teleop_cmd(): the profile -> the exact `ros2 launch rebel_demo teleop.launch.py ...` argument list
  * compute_hints(): turns the live status snapshot into "what to do next" messages
  * joint_zone(): colour class of a joint position next to its limits
"""
import copy
import json
import math
import os
import shlex
from pathlib import Path

from .kinematics import LOWER, UPPER, parse_pose_deg

DRIVER_CMD = ["ros2", "launch", "mocap4r2_optitrack_driver", "optitrack2.launch.py"]
DRIVER_NODE = "mocap4r2_optitrack_driver_node"
CONFIG_DIR = Path(os.environ.get("XDG_CONFIG_HOME", Path.home() / ".config")) / "rebel_mocap_teleop"
PROFILE_FILE = CONFIG_DIR / "profiles.json"

# the "best known" settings of the README: the same values as scripts/run.sh real / isaac
_BASE = {
    "rig": "both",
    "left_hand": 2, "left_gripper": 4, "right_hand": 1, "right_gripper": 3,     # Motive streaming IDs
    "scale_trans": 1.0, "scale_rot": 1.0, "cutoff_hz": 6.0, "max_joint_speed_deg": 45.0,
    "orientation": "semi", "roll_sign": 1, "gripper_mode": "orientation",
    "workspace_max": "0.7 0.7 1.4", "robot_ypr": "1.5708 0 0", "home_deg": "0 0 60 0 50 0",
    "skip_home": False, "mocap_bridge": True, "extra": "", "limit_margin_deg": 5.0,
    "robot_tf": "own", "joint_states_topic": "",
}
BUILTIN_PROFILES = {
    "Real robot": dict(_BASE, mode="real"),
    "Isaac Sim": dict(_BASE, mode="isaac"),
    # robot PC TF: its world IS the left arm base (our model turns the bases -90 deg), so no extra yaw
    "Real robot (robot PC TF)": dict(_BASE, mode="real", robot_tf="external", robot_ypr="0 0 0"),  # = run.sh real-ext
}
DEFAULT_PROFILE = "Real robot"


def normalize(profile):
    """A complete profile: missing keys come from the Real-robot defaults."""
    out = copy.deepcopy(BUILTIN_PROFILES[DEFAULT_PROFILE])
    out.update(profile or {})
    return out


def _num(x):
    return repr(float(x))


def arms_of(profile):
    """[(side, node-name suffix, hand streaming ID)] of the running rig: rig left / right use the nodes without suffix."""
    p = normalize(profile)
    if p["rig"] == "both":
        return [("left", "_left", int(p["left_hand"])), ("right", "_right", int(p["right_hand"]))]
    side = p["rig"]
    return [(side, "", int(p[f"{side}_hand"]))]


def build_teleop_cmd(profile):
    """Argument list for Popen: `ros2 launch rebel_demo teleop.launch.py ...` for this profile."""
    p = normalize(profile)
    cmd = ["ros2", "launch", "rebel_demo", "teleop.launch.py", "target:=isaac", f"rig:={p['rig']}", "rviz:=false",
           f"scale_trans:={_num(p['scale_trans'])}", f"cutoff_hz:={_num(p['cutoff_hz'])}",
           f"left_hand_frame:={int(p['left_hand'])}", f"left_gripper_frame:={int(p['left_gripper'])}",
           f"right_hand_frame:={int(p['right_hand'])}", f"right_gripper_frame:={int(p['right_gripper'])}",
           f"gripper_mode:={p['gripper_mode']}", f"orientation:={p['orientation']}",
           f"max_joint_speed_deg:={_num(p['max_joint_speed_deg'])}",
           f"workspace_max:={p['workspace_max'].strip()}", f"robot_ypr:={p['robot_ypr'].strip()}",
           f"home_deg:={p['home_deg'].strip()}"]
    if float(p["scale_rot"]) != 1.0:
        cmd.append(f"scale_rot:={_num(p['scale_rot'])}")
    if int(p["roll_sign"]) != 1:
        cmd.append(f"roll_sign:={int(p['roll_sign'])}")
    if p["skip_home"]:
        cmd.append("skip_home:=true")
    if float(p["limit_margin_deg"]) != 5.0:
        cmd.append(f"limit_margin_deg:={_num(p['limit_margin_deg'])}")
    if p["mode"] == "real":            # no Isaac: PC time
        cmd.append("use_sim_time:=false")
        # the joint-state stream (rig TF) is made from the commands, unless the robot PC publishes the arm TF itself
        cmd.append("robot_tf:=external" if p["robot_tf"] == "external" else "fake_state:=true")
        # start from the robot's measured joints (never assume zeros); never our rig topic, never published by us
        cmd += [f"robot_joint_states_topic:={p['joint_states_topic'].strip() or '/joint_states'}",
                "require_joint_states:=true",
                "stale_timeout:=0.5",   # the mocap network delivers frames late / in bursts: do not hold on short gaps
                # the robot PC's follower moves 0.3 rad/s and switches to catch-up beyond 0.5 rad (28 deg): the command
                # never runs more than 20 deg ahead of the measured joints, so it never gets there
                "max_lead_deg:=20",
                "rot_axis_sign:=-1 1 1",   # orientation:=true on the real setup: the roll about x is reversed
                # pulling back from a joint limit moves the arm at once; flicks forget little (limit_leak_speed)
                "limit_follow:=true"]
    elif p["joint_states_topic"].strip():
        cmd.append(f"joint_states_topic:={p['joint_states_topic'].strip()}")
    if not p["mocap_bridge"]:
        cmd.append("mocap_bridge:=false")
    cmd += shlex.split(p["extra"] or "")
    return cmd


def format_cmd(cmd):
    """The command as you would type it (values with spaces quoted)."""
    out = []
    for c in cmd:
        if ":=" in c:
            key, value = c.split(":=", 1)
            out.append(f"{key}:={shlex.quote(value)}" if " " in value else c)
        else:
            out.append(shlex.quote(c))
    return " ".join(out)


def _floats(text, n, what):
    try:
        v = [float(x) for x in text.replace(",", " ").split()]
    except ValueError:
        raise ValueError(f"{what}: numbers only") from None
    if len(v) != n:
        raise ValueError(f"{what}: need {n} numbers, got {len(v)}")
    return v


def validate(profile):
    """List of problems (empty = ok)."""
    p = normalize(profile)
    errs = []
    if p["rig"] not in ("both", "left", "right"):
        errs.append("rig must be both, left or right")
    if p["mode"] not in ("real", "isaac"):
        errs.append("mode must be real or isaac")
    if p["robot_tf"] not in ("own", "external"):
        errs.append("robot TF must be own or external")
    elif p["robot_tf"] == "external" and p["mode"] != "real":
        errs.append("robot TF 'external' is for the real robot (Isaac's bridge needs our robot model)")
    ids = [int(p[k]) for k in ("left_hand", "left_gripper", "right_hand", "right_gripper")]
    used = ids if p["rig"] == "both" else ids[:2] if p["rig"] == "left" else ids[2:]
    if any(i < 1 for i in used):
        errs.append("mocap IDs must be 1 or more")
    if len(set(used)) != len(used):
        errs.append("mocap IDs must be different (hand and gripper bodies cannot share an ID)")
    try:
        parse_pose_deg(p["home_deg"])
    except ValueError as e:
        errs.append(str(e))
    for key, n, what in (("workspace_max", 3, "workspace_max"), ("robot_ypr", 3, "robot_ypr")):
        try:
            _floats(p[key], n, what)
        except ValueError as e:
            errs.append(str(e))
    for key in ("scale_trans", "scale_rot", "cutoff_hz", "max_joint_speed_deg"):
        if not float(p[key]) > 0:
            errs.append(f"{key} must be above 0")
    if not 0.0 <= float(p["limit_margin_deg"]) < 45.0:
        errs.append("limit_margin_deg must be 0..45")
    elif p["mode"] == "real" and float(p["limit_margin_deg"]) < 3.0:
        errs.append("limit_margin_deg below 3 deg: the robot's mocap follower clamps 2.9 deg inside the limits, "
                    "so near a limit the arm would not follow what we send")
    if float(p["max_joint_speed_deg"]) > 45.0:
        errs.append("max_joint_speed_deg above 45 is faster than the real ReBeL maximum")
    try:
        shlex.split(p["extra"] or "")
    except ValueError:
        errs.append("extra arguments: unbalanced quotes")
    return errs


def ypr_deg_to_text(yaw, pitch, roll):
    return " ".join(_num(round(math.radians(v), 4)) for v in (yaw, pitch, roll))


def ypr_text_to_deg(text):
    return [round(math.degrees(v), 2) for v in _floats(text, 3, "robot_ypr")]


class ProfileStore:
    """Built-in profiles plus the user's, kept in ~/.config/rebel_mocap_teleop/profiles.json."""

    def __init__(self, path=PROFILE_FILE):
        self.path = Path(path)
        self.user, self.last = {}, DEFAULT_PROFILE
        try:
            data = json.loads(self.path.read_text())
            self.user = {k: normalize(v) for k, v in data.get("profiles", {}).items()}
            self.last = data.get("last", DEFAULT_PROFILE)
        except (OSError, ValueError, AttributeError):
            pass

    def names(self):
        return list(BUILTIN_PROFILES) + [n for n in self.user if n not in BUILTIN_PROFILES]

    def get(self, name):
        return normalize(self.user.get(name) or BUILTIN_PROFILES.get(name) or BUILTIN_PROFILES[DEFAULT_PROFILE])

    def is_builtin(self, name):
        return name in BUILTIN_PROFILES and name not in self.user

    def save(self, name, profile):
        self.user[name] = normalize(profile)
        self.last = name
        self.flush()

    def delete(self, name):
        self.user.pop(name, None)
        if self.last == name:
            self.last = DEFAULT_PROFILE
        self.flush()

    def flush(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text(json.dumps({"last": self.last, "profiles": self.user}, indent=2))
        tmp.replace(self.path)


def joint_zone(pos, lo, hi, warn_deg=10.0, limit_deg=3.0):
    """'ok' | 'warn' | 'limit' for a joint angle [rad] close to its limits."""
    margin = min(pos - lo, hi - pos)
    if margin <= math.radians(limit_deg):
        return "limit"
    return "warn" if margin <= math.radians(warn_deg) else "ok"


def joint_limits():
    return list(zip(LOWER, UPPER))


def compute_hints(s, profile, teleop_running):
    """What to do next, from the live snapshot `s` (see rebel_ui.RosBridge) - the mistakes we actually ran into."""
    p = normalize(profile)
    hints = []
    drv = s.get("driver", {})
    if not drv.get("node"):
        hints.append("Mocap driver is not running: press 'Start driver'.")
    elif drv.get("state") == "inactive":
        hints.append("Mocap driver is inactive (configured but not activated): press 'Activate driver'.")
    elif drv.get("state") == "active" and s.get("mocap_hz", 0) < 5:
        hints.append("Driver is active but no frames arrive: is Motive streaming (Broadcast Frame Data, Local Interface, "
                     "firewall)? Restart the driver if Motive was started after it.")
    names = set(s.get("rb_names", []))
    need = {str(i[2]) for i in arms_of(p)} | {str(p[k]) for k in _gripper_keys(p)}
    if names and not need <= names:
        hints.append("Motive does not send streaming ID(s) " + ", ".join(sorted(need - names, key=int)) +
                     " (sees " + ", ".join(sorted(names)) + "): check the IDs or that the bodies are tracked.")
    if p["mode"] == "isaac":
        if s.get("clock_pub", 0) == 0:
            hints.append(f"No /clock: Isaac Sim is not running, not playing, or not on ROS_DOMAIN_ID {s.get('domain', '?')}.")
        elif s.get("states_pub", 0) == 0:
            hints.append("Isaac publishes /clock but no joint states: run the three bridge scripts in its Script Editor.")
    elif s.get("cmd_sub", 0) == 0 and teleop_running:
        hints.append("Nobody listens to /dual_arm/isaac_joint_commands: start your robot driver / relay.")
    if teleop_running:
        missing = [side for side, suffix, _ in arms_of(p) if not s.get("hands", {}).get(side)]
        if missing and s.get("mocap_hz", 0) >= 5:
            hints.append("Hand TF not available for the " + " and ".join(missing) + " arm: that body is not tracked "
                         "(or its ID is wrong).")
    return hints


def _gripper_keys(p):
    return ("left_gripper", "right_gripper") if p["rig"] == "both" else (f"{p['rig']}_gripper",)
