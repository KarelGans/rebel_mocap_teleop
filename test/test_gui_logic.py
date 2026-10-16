"""Tests of the GUI logic (no Qt, no ROS): python3 -m pytest test/ from the repo root."""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from rebel_demo.gui_logic import (BUILTIN_PROFILES, ProfileStore, arms_of, build_teleop_cmd, compute_hints,   # noqa: E402
                                  format_cmd, joint_zone, normalize, validate, ypr_deg_to_text, ypr_text_to_deg)


def as_dict(cmd):
    return {c.split(":=")[0]: c.split(":=", 1)[1] for c in cmd if ":=" in c}


def same_values(a, b):
    """Launch arguments equal, comparing numbers as numbers (45 == 45.0)."""
    if a.keys() != b.keys():
        return False
    for k in a:
        try:
            if not math.isclose(float(a[k]), float(b[k])):
                return False
        except ValueError:
            if a[k].split() != b[k].split():
                return False
    return True


# the "best known" real-robot command of the README / scripts/run.sh real
RUN_SH_REAL = {"target": "isaac", "rig": "both", "rviz": "false", "scale_trans": "1.0", "cutoff_hz": "6.0",
               "left_hand_frame": "2", "left_gripper_frame": "4", "right_hand_frame": "1", "right_gripper_frame": "3",
               "gripper_mode": "orientation", "orientation": "semi", "max_joint_speed_deg": "45",
               "workspace_max": "0.7 0.7 1.4", "robot_ypr": "1.5708 0 0", "home_deg": "0 0 60 0 50 0",
               "use_sim_time": "false", "fake_state": "true"}


def test_real_profile_is_the_run_sh_real_command():
    assert same_values(as_dict(build_teleop_cmd(BUILTIN_PROFILES["Real robot"])), RUN_SH_REAL)


def test_isaac_profile_has_no_real_robot_flags():
    d = as_dict(build_teleop_cmd(BUILTIN_PROFILES["Isaac Sim"]))
    assert "use_sim_time" not in d and "fake_state" not in d
    assert same_values({k: v for k, v in RUN_SH_REAL.items() if k not in ("use_sim_time", "fake_state")}, d)


def test_optional_arguments_only_when_changed():
    base = as_dict(build_teleop_cmd(BUILTIN_PROFILES["Isaac Sim"]))
    assert not {"scale_rot", "roll_sign", "skip_home", "mocap_bridge"} & base.keys()
    p = dict(BUILTIN_PROFILES["Isaac Sim"], scale_rot=0.5, roll_sign=-1, skip_home=True, mocap_bridge=False,
             extra="limit_warn_mm:=10")
    d = as_dict(build_teleop_cmd(p))
    assert d["scale_rot"] == "0.5" and d["roll_sign"] == "-1" and d["skip_home"] == "true"
    assert d["mocap_bridge"] == "false" and d["limit_warn_mm"] == "10"


def test_arms_and_node_suffixes():
    assert arms_of({"rig": "both"}) == [("left", "_left", 2), ("right", "_right", 1)]
    assert arms_of({"rig": "right", "right_hand": 3}) == [("right", "", 3)]


def test_validate():
    assert validate(BUILTIN_PROFILES["Real robot"]) == []
    assert validate(dict(BUILTIN_PROFILES["Real robot"], left_hand=1, right_hand=1))            # duplicate IDs
    assert validate(dict(BUILTIN_PROFILES["Real robot"], home_deg="0 0 60"))                     # not 6 angles
    assert validate(dict(BUILTIN_PROFILES["Real robot"], home_deg="0 0 60 0 120 0"))             # joint 5 over its limit
    assert validate(dict(BUILTIN_PROFILES["Real robot"], max_joint_speed_deg=60))                # above the ReBeL maximum
    assert validate(dict(BUILTIN_PROFILES["Real robot"], extra="a 'b"))                          # unbalanced quote
    assert validate(dict(BUILTIN_PROFILES["Real robot"], rig="left", right_hand=2, right_gripper=2)) == []   # unused side


def test_ypr_conversion_roundtrip():
    assert ypr_deg_to_text(90, 0, 0).split() == ["1.5708", "0.0", "0.0"]
    assert ypr_text_to_deg("1.5708 0 0") == [90.0, 0.0, 0.0]


def test_profile_store_roundtrip(tmp_path):
    path = tmp_path / "profiles.json"
    store = ProfileStore(path)
    assert store.names() == ["Real robot", "Isaac Sim"] and store.is_builtin("Real robot")
    store.save("Lab", dict(BUILTIN_PROFILES["Real robot"], scale_trans=0.5))
    again = ProfileStore(path)
    assert "Lab" in again.names() and again.get("Lab")["scale_trans"] == 0.5 and again.last == "Lab"
    again.delete("Lab")
    assert ProfileStore(path).names() == ["Real robot", "Isaac Sim"]
    path.write_text("not json")                                                                  # a broken file never crashes
    assert ProfileStore(path).names() == ["Real robot", "Isaac Sim"]
    assert normalize({"scale_trans": 2})["rig"] == "both"


def test_joint_zone():
    lo, hi = -math.radians(179), math.radians(179)
    assert joint_zone(0.0, lo, hi) == "ok"
    assert joint_zone(math.radians(172), lo, hi) == "warn"
    assert joint_zone(math.radians(177), lo, hi) == "limit"


def _snap(**kw):
    s = {"domain": "11", "driver": {"node": True, "state": "active"}, "mocap_hz": 119, "rb_names": ["1", "2", "3", "4"],
         "clock_pub": 1, "states_pub": 1, "cmd_sub": 1, "hands": {"left": True, "right": True}}
    s.update(kw)
    return s


def test_hints_cover_the_mistakes_we_made():
    p = BUILTIN_PROFILES["Isaac Sim"]
    assert compute_hints(_snap(), p, True) == []
    assert "not running" in compute_hints(_snap(driver={"node": False, "state": None}, mocap_hz=0), p, False)[0]
    assert "inactive" in compute_hints(_snap(driver={"node": True, "state": "inactive"}, mocap_hz=0), p, False)[0]
    assert "no frames" in compute_hints(_snap(mocap_hz=0), p, False)[0]
    assert "streaming ID" in compute_hints(_snap(rb_names=["1", "2"]), p, False)[0]
    assert "No /clock" in compute_hints(_snap(clock_pub=0, states_pub=0), p, False)[0]
    assert "bridge scripts" in compute_hints(_snap(states_pub=0), p, False)[0]
    assert "Hand TF" in compute_hints(_snap(hands={"left": True, "right": False}), p, True)[0]
    real = BUILTIN_PROFILES["Real robot"]
    assert "robot driver" in compute_hints(_snap(cmd_sub=0), real, True)[0]
    assert compute_hints(_snap(cmd_sub=0, clock_pub=0, states_pub=0), real, False) == []         # nothing started: no nagging


def test_format_cmd_quotes_values_with_spaces():
    out = format_cmd(build_teleop_cmd(BUILTIN_PROFILES["Real robot"]))
    assert "workspace_max:='0.7 0.7 1.4'" in out and out.startswith("ros2 launch rebel_demo teleop.launch.py")
