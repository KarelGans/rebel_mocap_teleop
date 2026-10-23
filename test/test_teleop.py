"""Pure-python tests (no ROS needed): python3 -m pytest test/"""
import math

import numpy as np
import pytest

from rebel_demo.kinematics import (HOME, HW_LOWER, LOWER, UPPER, clamp_roll, drop_unreachable, fk, ik, limit_lead,
                                   limits_hit, parse_pose_deg, quat_to_matrix, rot, scale_rotation, soft_limits,
                                   twist_about)

# the bent-elbow "ready" pose (the former HOME; HOME is now all zeros = arm straight up)
READY = np.radians([0.0, 13.4, 87.9, 0.0, 78.7, 0.0])


def test_home_pose():
    T = fk(READY)                                   # tool0 35 cm in front, 30 cm up, pointing straight down
    assert np.allclose(T[:3, 3], [0.35, 0, 0.30], atol=2e-3)
    assert np.allclose(T[:3, 2], [0, 0, -1], atol=1e-2)
    assert READY[2] > 0                             # elbow bent the normal way


def test_uniform_joint_scaling_keeps_direction():
    max_step = math.radians(45) / 60
    want = np.radians([0.0, 3.0, 1.0, 0.0, 0.5, 0.0])           # IK step, ratio 6 : 2 : 1
    m = float(np.max(np.abs(want)))
    dq = want * (max_step / m) if m > max_step else want
    assert math.isclose(float(np.max(np.abs(dq))), max_step)    # largest joint exactly at the limit
    assert np.allclose(dq / np.linalg.norm(dq), want / np.linalg.norm(want))   # same direction
    small = np.radians([0.0, 0.1, 0.05, 0.0, 0.0, 0.0])        # below the limit: unchanged
    m = float(np.max(np.abs(small)))
    assert np.allclose(small * (max_step / m) if m > max_step else small, small)


def test_analytic_jacobian():
    from rebel_demo.kinematics import fk_jac
    q = np.radians([10, 30, 60, -20, 40, 15])
    T, J = fk_jac(q)
    for i in range(6):
        dq = np.zeros(6)
        dq[i] = 1e-6
        assert np.allclose((fk(q + dq)[:3, 3] - T[:3, 3]) / 1e-6, J[:3, i], atol=1e-5)


def test_real_rebel_limits():
    """The real ReBeL's limits as used by the robot PC's mocap follower (2026-10-21)."""
    deg = np.degrees
    assert np.allclose(deg(UPPER), [179, 139, 139, 179, 94, 179], atol=0.01)
    assert np.allclose(deg(HW_LOWER), [-179, -79, -79, -179, -94, -179], atol=0.01)
    assert math.isclose(deg(LOWER[2]), 15.0)                               # the IK's elbow-up bound on joint 3


def test_ik_round_trip():
    rng = np.random.default_rng(1)
    for _ in range(20):
        q = READY + rng.uniform(-0.4, 0.4, 6)
        _, pe, re = ik(fk(q), q + rng.uniform(-0.1, 0.1, 6))
        assert pe < 1e-3 and re < 0.5


def test_quat_to_matrix():
    a = math.radians(90)
    R = quat_to_matrix([0, 0, math.sin(a / 2), math.cos(a / 2)])
    assert np.allclose(R @ [1, 0, 0], [0, 1, 0], atol=1e-9)


def test_gripper_threshold_hysteresis():
    pytest.importorskip("rclpy")                        # the node modules need ROS 2: skipped (not passed) without it
    from rebel_demo.gripper_node import next_state
    assert next_state(0.02, 1, 0.06, 0.01) == 0           # near -> close
    assert next_state(0.10, 0, 0.06, 0.01) == 1           # far -> open
    assert next_state(0.062, 0, 0.06, 0.01) == 0          # inside dead band -> keep
    assert next_state(0.062, 1, 0.06, 0.01) == 1
    assert next_state(0.02, 0, 0.06, 0.01, invert=True) == 1


def _q(axis, deg):
    a = math.radians(deg) / 2
    n = math.sqrt(sum(c * c for c in axis))
    return tuple(c / n * math.sin(a) for c in axis) + (math.cos(a),)


def test_gripper_orientation_angle():
    pytest.importorskip("rclpy")                        # the node modules need ROS 2: skipped (not passed) without it
    from rebel_demo.gripper_node import next_state_angle, quat_mul, rotation_angle_deg
    assert abs(rotation_angle_deg((0, 0, 0, 1))) < 1e-9
    for axis in [(1, 0, 0), (0, 1, 0), (0, 0, 1), (1, 1, 0)]:
        assert abs(rotation_angle_deg(_q(axis, 180)) - 180) < 1e-6          # flipped about any axis
        assert abs(rotation_angle_deg(_q(axis, 60)) - 60) < 1e-6
    q = _q((0, 1, 0), 60)
    assert abs(rotation_angle_deg(tuple(-c for c in q)) - 60) < 1e-6        # q and -q are the same rotation
    # turning the whole hand (both bodies by the same world rotation) does not change the difference
    palm, finger = _q((0, 0, 1), 20), quat_mul(_q((0, 0, 1), 20), _q((1, 0, 0), 150))
    face = _q((1, 1, 0), 77)
    rel = lambda a, b: quat_mul((-a[0], -a[1], -a[2], a[3]), b)             # a^-1 * b
    assert abs(rotation_angle_deg(rel(palm, finger)) - rotation_angle_deg(rel(quat_mul(face, palm), quat_mul(face, finger)))) < 1e-6
    # calibration: the open pose counts as 0 deg
    q_open = _q((0, 0, 1), 40)
    assert abs(rotation_angle_deg(q_open, q_open)) < 1e-6
    assert abs(rotation_angle_deg(quat_mul(q_open, _q((1, 0, 0), 180)), q_open) - 180) < 1e-6


def test_gripper_angle_bands():
    pytest.importorskip("rclpy")                        # the node modules need ROS 2: skipped (not passed) without it
    from rebel_demo.gripper_node import next_state_angle
    assert next_state_angle(10, 0, 30, 100) == 1           # same plane -> open
    assert next_state_angle(170, 1, 30, 100) == 0          # reversed -> closed
    assert next_state_angle(60, 1, 30, 100) == 1           # in between -> keep
    assert next_state_angle(60, 0, 30, 100) == 0
    assert next_state_angle(10, 1, 30, 100, invert=True) == 0


def test_keyboard_debounce():
    pytest.importorskip("rclpy")                        # the node modules need ROS 2: skipped (not passed) without it
    from rebel_demo.keyboard_node import KeyDebounce
    d = KeyDebounce(0.3)
    assert d(10.0) is True
    assert d(10.1) is False                 # key repeat of a held SPACE
    assert d(10.35) is True


def test_limits_hit():
    from rebel_demo.kinematics import limits_hit
    assert limits_hit(READY) == []                                   # READY is well inside every limit
    q = READY.copy()
    q[1], q[2] = UPPER[1], LOWER[2]                                 # shoulder at its upper limit, elbow at the bound
    assert limits_hit(q) == [(1, "upper"), (2, "lower")]
    q = READY.copy()
    q[2] = LOWER[2] + math.radians(0.5)                             # inside the 1 deg margin
    assert limits_hit(q) == [(2, "lower")]
    assert limits_hit(q, margin_deg=0.1) == []                      # outside a tighter margin


def test_ik_unreachable_target_stays_inside_limits_and_gets_close():
    far = fk(HOME)
    far[:3, 3] = [0.3, 0.0, -1.5]                                    # far below anything the arm can reach
    q, pe, _ = ik(far, HOME, w_rot=0.0, iters=200, q_rest=HOME)
    assert np.all(q >= LOWER - 1e-9) and np.all(q <= UPPER + 1e-9)   # never outside the limits (incl. elbow bound)
    assert pe < np.linalg.norm(far[:3, 3] - fk(HOME)[:3, 3])         # moved towards the target
    # slightly beyond reach: pinning at the limit gives the closest pose, not a corner posture far away
    near = fk(HOME)
    near[:3, 3] = fk(HOME)[:3, 3] + [0.15, 0.0, -0.55]               # 55 cm down, 15 cm forward (just out of reach)
    q = HOME.copy()
    for _ in range(400):
        qi, pe, _ = ik(near, q, w_rot=0.0, iters=25, q_rest=HOME)
        q = np.clip(q + np.clip(qi - q, -0.0125, 0.0125), LOWER, UPPER)
    assert np.linalg.norm(fk(q)[:3, 3] - near[:3, 3]) < 0.14


def test_twist_about_pure_rotation_tilt_and_wrap():
    z = np.array([0.0, 0.0, 1.0])
    for deg in (-170, -90, -30, 0, 30, 90, 170):
        assert math.isclose(twist_about(rot(z, math.radians(deg)), z), math.radians(deg), abs_tol=1e-6)
    assert abs(twist_about(rot(np.array([1.0, 0, 0]), math.radians(8)), z)) < 1e-6       # pure tilt: no twist
    swing_twist = rot(np.array([1.0, 0, 0]), math.radians(8)) @ rot(z, math.radians(40))
    assert math.isclose(twist_about(swing_twist, z), math.radians(40), abs_tol=math.radians(1))
    assert math.isclose(abs(twist_about(rot(z, math.pi), z)), math.pi, abs_tol=1e-6)    # 180 deg, no NaN


def test_parse_pose_deg_and_start_pose():
    q = parse_pose_deg("0 0 100 0 50 0")
    assert np.allclose(np.degrees(q), [0, 0, 100, 0, 50, 0])
    assert np.allclose(parse_pose_deg("0, 0, 100, 0, 50, 0"), q)             # commas work too
    assert np.allclose(parse_pose_deg("0 0 0 0 0 0"), 0)                      # the default (below the 15 deg IK bound) is valid
    for bad in ("0 0 100", "0 0 100 0 50 0 0", "a b c d e f", "0 0 100 0 120 0", "0 0 -100 0 50 0"):
        try:
            parse_pose_deg(bad)
        except ValueError:
            continue
        raise AssertionError(f"accepted {bad!r}")                             # joint5 120 > 95; joint3 -100 < -80
    from rebel_demo.kinematics import limits_hit
    assert limits_hit(q) == []                                               # well inside every limit
    T = fk(q)
    assert np.allclose(T[:3, 3], [0.36, 0, 0.33], atol=0.01)
    for d in ([0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1], [-0.1, 0, -0.1]):       # IK holds targets around the start pose
        Tg = T.copy()
        Tg[:3, 3] += d
        _, pe, _ = ik(Tg, q, w_rot=0.0, iters=25, q_rest=q)
        assert pe < 1e-3


def test_scale_rotation():
    axis = np.array([1.0, 2.0, 2.0]) / 3.0
    for deg in (1, 30, 90, 170):
        R = rot(axis, math.radians(deg))
        assert scale_rotation(R, 1.0) is R                                     # k = 1 is exactly today's behaviour
        assert np.allclose(scale_rotation(R, 0.0), np.eye(3))
        assert np.allclose(scale_rotation(R, 0.5), rot(axis, math.radians(deg) / 2), atol=1e-9)
        assert np.allclose(scale_rotation(R, 2.0), rot(axis, math.radians(deg) * 2), atol=1e-9)
    assert np.allclose(scale_rotation(np.eye(3), 3.0), np.eye(3))               # no rotation stays none
    R = rot(axis, math.radians(40))
    out = scale_rotation(R, 0.25)
    assert math.isclose(twist_about(out, axis), math.radians(10), abs_tol=1e-9)  # same axis, a quarter of the angle


def test_limit_lead_off():
    q = np.radians([10.0, -80.0, 100.0, 5.0, 0.0, 170.0])
    meas = np.zeros(6)
    for out in (limit_lead(q, meas, 0.0), limit_lead(q, meas, -1.0), limit_lead(q, None, 0.3)):
        assert np.array_equal(out, q)
    assert limit_lead(q, None, 0.3) is not q                                    # a copy


def test_limit_lead_clamps_per_joint():
    meas = np.radians([0.0, 10.0, 50.0, -30.0, 0.0, 90.0])
    lead = math.radians(20)
    q = meas + np.radians([5.0, 40.0, -25.0, -19.0, 21.0, 0.0])               # in range, over, under, in, over, in
    out = limit_lead(q, meas, lead)
    assert np.allclose(out - meas, np.radians([5.0, 20.0, -20.0, -19.0, 20.0, 0.0]))
    assert np.array_equal(out[[0, 3, 5]], q[[0, 3, 5]])                       # joints within range are untouched
    assert np.all(np.abs(out - meas) <= lead + 1e-12)


def _follow(max_lead_deg, seconds=5.0, rate=60.0):
    """A joint commanded up at 45 deg/s, a follower arm chasing it at 0.3 rad/s; returns the largest |cmd - arm| [rad]."""
    dt, vmax, follower_v = 1.0 / rate, math.radians(45), 0.3
    cmd, arm, worst = np.zeros(6), np.zeros(6), 0.0
    for _ in range(int(seconds * rate)):
        cmd = cmd + vmax * dt * np.array([0, 0, 0, 0, 0, 1.0])
        cmd = limit_lead(cmd, arm, math.radians(max_lead_deg))                # what Teleop.hold_lead does when fresh
        arm = arm + np.clip(cmd - arm, -follower_v * dt, follower_v * dt)
        worst = max(worst, float(np.max(np.abs(cmd - arm))))
    return worst


def test_lead_step_holds_without_feedback_and_never_jumps():
    from rebel_demo.kinematics import lead_step
    lead, step = math.radians(20), math.radians(45) / 60
    q_prev = np.radians([0.0, 0.0, 60.0, 0.0, 50.0, 0.0])
    q = q_prev + step * np.array([1.0, 0, 0, 0, 0, 0])
    assert np.array_equal(lead_step(q_prev, q, q_prev, lead, step), q)            # inside the window: unchanged, exact
    assert np.array_equal(lead_step(q_prev, q, None, lead, step), q_prev)         # stale / missing feedback: hold
    assert np.array_equal(lead_step(q_prev, q, None, 0.0, step), q)               # limit off: unchanged
    # the arm fell 35 deg behind on joint 1 while the feedback was away: back into the window at the speed limit,
    # never in one step (the follower answers a jump > 0.2 rad between two messages with catch-up)
    far = q_prev + np.radians([35.0, 0, 0, 0, 0, 0])
    out = lead_step(far, far, q_prev, lead, step)
    assert math.isclose(float(np.max(np.abs(out - far))), step)
    for _ in range(int(math.radians(15) / step) + 2):
        out = lead_step(out, far, q_prev, lead, step)
    assert np.allclose(out - q_prev, np.radians([20.0, 0, 0, 0, 0, 0]))         # ends on the window edge


def test_lead_limit_keeps_slow_follower_close():
    assert _follow(0.0) > 1.75                                                # without it the follower would latch
    assert _follow(20.0) <= math.radians(20) + 1e-9


def test_soft_limits_values():
    ik_lo, ik_hi, out_lo, out_hi = soft_limits(10.0)
    deg = np.degrees
    assert np.allclose(deg(ik_hi), [169, 129, 129, 169, 84, 169], atol=0.01)
    assert np.allclose(deg(ik_lo), [-169, -69, 15, -169, -84, -169], atol=0.01)       # joint 3 keeps the elbow-up bound
    assert np.allclose(deg(out_lo), [-169, -69, -69, -169, -84, -169], atol=0.01)     # outputs: hardware limits - margin
    lo0, hi0, _, _ = soft_limits(0.0)
    assert np.allclose(lo0, LOWER) and np.allclose(hi0, UPPER)                          # 0 = the old behaviour


def test_ik_never_leaves_the_soft_limits():
    ik_lo, ik_hi, _, _ = soft_limits(10.0)
    rng = np.random.default_rng(3)
    for _ in range(200):
        q0 = np.clip(rng.uniform(LOWER, UPPER), ik_lo, ik_hi)
        T = fk(np.clip(q0 + rng.uniform(-2.0, 2.0, 6), LOWER, UPPER))              # far targets, also behind the limits
        T[:3, 3] += rng.uniform(-0.4, 0.4, 3)
        for w_rot in (0.0, 0.3):
            q, _, _ = ik(T, q0, w_rot=w_rot, iters=25, lower=ik_lo, upper=ik_hi)
            assert np.all(q >= ik_lo - 1e-9) and np.all(q <= ik_hi + 1e-9)


def test_clamp_roll_anti_windup():
    lo, hi = math.radians(-169), math.radians(169)
    acc = 0.0
    for _ in range(40):                                     # keep turning the hand far past the limit
        acc += math.radians(10)
        q6, acc = clamp_roll(0.0, acc, 1.0, lo, hi)
    assert math.isclose(q6, hi) and math.isclose(acc, hi)  # held at the limit, accumulator not wound up
    q6, acc = clamp_roll(0.0, acc - math.radians(10), 1.0, lo, hi)
    assert math.isclose(math.degrees(q6), 159, abs_tol=1e-6)   # turning back moves it at once
    q6, _ = clamp_roll(0.0, math.radians(400), -1.0, lo, hi)
    assert math.isclose(q6, lo)                            # roll_sign -1 clamps at the other side


def test_full_pose_teleop_loop_stays_inside_the_margin():
    """The case from the real robot: orientation:=true, hand rotating far past what joints 4 / 6 allow."""
    ik_lo, ik_hi, out_lo, out_hi = soft_limits(10.0)
    vmax, dt = math.radians(45), 1 / 60
    q_cmd = np.radians([0, 0, 60, 0, 50, 0])
    ref = fk(q_cmd)
    axis = np.array([0.3, 0.2, 1.0]) / np.linalg.norm([0.3, 0.2, 1.0])
    worst = 0.0
    for k in range(1200):                                   # 20 s, hand turns 2.5 full turns and tilts
        T = ref.copy()
        T[:3, :3] = rot(axis, 2.5 * 2 * math.pi * k / 1200) @ rot(np.array([1.0, 0, 0]), 0.4 * math.sin(k / 90)) @ ref[:3, :3]
        q_ik, _, _ = ik(T, q_cmd, w_rot=0.3, iters=25, q_rest=None, lower=ik_lo, upper=ik_hi)
        dq = np.clip(q_ik - q_cmd, -vmax * dt, vmax * dt)
        q_cmd = np.clip(q_cmd + dq, np.minimum(out_lo, q_cmd), np.maximum(out_hi, q_cmd))
        worst = max(worst, float(np.max(np.abs(np.degrees(q_cmd[[3, 5]])))))
        assert np.all(q_cmd >= out_lo - 1e-9) and np.all(q_cmd <= out_hi + 1e-9)
    assert worst <= math.degrees(out_hi[3]) + 1e-6 < 169.01                # +-169 deg, not the +-179 deg hard limit


def test_default_margin_stays_inside_the_follower_clamp():
    """Default limit_margin_deg = 5: everything published is >= 2 deg inside the follower's clamp (limits - 0.05 rad)."""
    ik_lo, ik_hi, out_lo, out_hi = soft_limits(5.0)
    assert np.allclose(np.degrees(out_hi), [174, 134, 134, 174, 89, 174], atol=0.01)
    assert np.allclose(np.degrees(out_lo), [-174, -74, -74, -174, -89, -174], atol=0.01)
    assert np.allclose(np.degrees(ik_lo)[2], 15.0)                                       # elbow-up bound unchanged
    clamp_lo, clamp_hi = HW_LOWER + 0.05, UPPER - 0.05
    spare = math.radians(2.0)
    assert np.all(out_hi <= clamp_hi - spare) and np.all(out_lo >= clamp_lo + spare)


def test_drop_unreachable_puts_the_target_on_the_reached_point():
    robot_ref, hand_ref, scale = np.array([0.36, 0.0, 0.33]), np.array([1.0, 2.0, 0.5]), 0.7
    hand_now = hand_ref + np.array([0.40, 0.05, -0.02])
    wanted = robot_ref + scale * (hand_now - hand_ref)
    reached = np.array([0.58, 0.03, 0.32])                                   # the arm's limit stopped it here
    new_ref = drop_unreachable(hand_ref, wanted, reached, scale)
    assert np.allclose(robot_ref + scale * (hand_now - new_ref), reached)    # the mapped target is now reachable
    assert np.allclose(robot_ref + scale * (hand_now + [-0.01, 0, 0] - new_ref), reached + [-scale * 0.01, 0, 0])


def _replay(follow):
    """teleop's per-cycle position mapping: hand 80 cm forward (far beyond reach) and back, 0.4 m/s, real limits."""
    dt, vmax = 1 / 60, math.radians(45)
    ik_lo, ik_hi, out_lo, out_hi = soft_limits(5.0)
    q_cmd = np.radians([0, 0, 60, 0, 50, 0])
    ref = fk(q_cmd)
    hand_ref, robot_ref = np.zeros(3), ref[:3, 3].copy()
    tool, cmds = [], []
    for k in range(int(4.0 / dt)):
        t = k * dt
        hand = np.array([0.4 * min(t, 2.0) - 0.4 * max(0.0, t - 2.0), 0.0, 0.0])
        T = ref.copy()
        wanted = robot_ref + (hand - hand_ref)
        T[:3, 3] = np.clip(wanted, [-0.7, -0.7, -0.45], [0.7, 0.7, 0.95])
        q_ik, pe, _ = ik(T, q_cmd, w_rot=0.0, iters=25, q_rest=np.radians([0, 0, 60, 0, 50, 0]), lower=ik_lo, upper=ik_hi)
        hits = limits_hit(q_ik, ik_lo, ik_hi) if pe > 0.002 else []
        if follow and (hits or np.any(np.abs(wanted - T[:3, 3]) > 1e-3)):
            hand_ref = drop_unreachable(hand_ref, wanted, fk(q_ik)[:3, 3], 1.0)
        dq = np.clip(q_ik - q_cmd, -vmax * dt, vmax * dt)
        q_cmd = np.clip(q_cmd + dq, np.minimum(out_lo, q_cmd), np.maximum(out_hi, q_cmd))
        assert np.all(q_cmd >= out_lo - 1e-9) and np.all(q_cmd <= out_hi + 1e-9)
        tool.append(fk(q_cmd)[0, 3])
        cmds.append(q_cmd.copy())
    turn = int(2.0 / dt)                                   # the hand turns back here
    back = next(i for i in range(turn, len(tool)) if tool[i] < tool[turn] - 0.005)
    return (back - turn) * dt                              # time until the tool has visibly (5 mm) come back


def test_keep_following_after_a_limit():
    """The 'same data although I moved my hand' case: with limit_follow the arm comes back as soon as the hand does."""
    assert _replay(follow=False) > 1.0                 # old: more than a second of no response after the hand turned
    assert _replay(follow=True) < 0.25                 # new: comes back at once (the rest is the joint speed limit)


# ---------------------------------------------------------------- orientation:=true: robot_remote's method
from rebel_demo.kinematics import (_log_so3, drop_unreachable_rot, flip_rotation, ik_priority, step_toward,  # noqa: E402
                                   toward_point, toward_rot)

HOME_REAL = np.radians([0, 0, 60, 0, 50, 0])


def test_flip_rotation_reverses_only_the_chosen_axis():
    x, y, z = np.eye(3)
    for axis, kept in ((x, False), (y, True), (z, True)):
        rv = _log_so3(flip_rotation(rot(axis, 0.5), (-1, 1, 1)))
        assert np.allclose(rv, axis * 0.5 * (1 if kept else -1))
    R = rot(np.array([0.3, 0.5, 0.8]) / np.linalg.norm([0.3, 0.5, 0.8]), 2.0)
    assert np.allclose(flip_rotation(flip_rotation(R, (-1, 1, 1)), (-1, 1, 1)), R)      # its own inverse
    assert np.allclose(flip_rotation(R, (1, 1, 1)), R)


def test_ik_priority_reaches_reachable_poses():
    lo, hi, _, _ = soft_limits(5.0)
    rng = np.random.default_rng(1)
    for _ in range(30):
        q_goal = rng.uniform(lo + 0.2, hi - 0.2)
        q_goal[4] = rng.uniform(0.3, 1.4)                    # wrist bent the home way (no singularity at joint 5 = 0)
        seed = np.clip(q_goal + rng.normal(0, 0.05, 6), lo, hi)
        _, pe, re = ik_priority(fk(q_goal), seed, lo, hi)
        assert pe < 1e-4 and re < 0.05


def test_ik_priority_keeps_the_position_at_a_wrist_limit():
    """Twist 30 deg past joint 6's limit: joint 6 stops there, the tool stays put and no other joint takes the twist over
    (re-solving with the other joints, as robot_remote does, turned joint 4 by 24 deg here)."""
    lo, hi, _, _ = soft_limits(5.0)
    q0 = np.radians([0, 0, 60, 0, 50, 170])
    T = fk(q0)
    T[:3, :3] = rot(T[:3, 2], math.radians(30)) @ T[:3, :3]
    q, pe, re = ik_priority(T, q0, lo, hi)
    assert q[5] == pytest.approx(hi[5]) and pe < 1e-4 and re == pytest.approx(26.0, abs=0.5)
    assert np.degrees(np.abs(q - q0)[:5]).max() < 0.5


def test_step_toward_keeps_the_direction():
    q = step_toward(np.zeros(3), np.array([1.0, 0.1, -0.5]), 0.1)
    assert np.allclose(q, [0.1, 0.01, -0.05])                # all joints scaled by the same factor
    assert np.allclose(step_toward(np.zeros(2), np.array([0.05, 0.0]), 0.1), [0.05, 0.0])
    assert np.allclose(toward_point(np.zeros(3), np.array([1.0, 0, 0]), 0.05), [0.05, 0, 0])
    R = toward_rot(np.eye(3), rot(np.array([0, 0, 1.0]), 1.0), 0.2)
    assert np.allclose(R, rot(np.array([0, 0, 1.0]), 0.2))


def test_drop_unreachable_rot_maps_onto_the_reached_rotation():
    R_ref = fk(HOME_REAL)[:3, :3]
    R_hand = rot(np.array([0.3, 0.5, 0.8]) / np.linalg.norm([0.3, 0.5, 0.8]), 0.7)
    R_reached = rot(np.array([1.0, 0, 0]), 0.4) @ R_ref
    for s, signs in ((1.0, (1, 1, 1)), (0.5, (-1, 1, 1)), (2.0, (1, -1, 1))):
        anchor = drop_unreachable_rot(R_hand, R_reached, R_ref, s, signs)
        assert np.allclose(scale_rotation(flip_rotation(R_hand @ anchor.T, signs), s) @ R_ref, R_reached, atol=1e-9)


def _replay_full_pose(axis, amp_deg, signs=(1, 1, 1), rate_deg=30.0):
    """teleop's orientation:=true cycle (rot_axis_sign, rate limit, lead clamp, ik_priority, keep following,
    step_toward), defaults of the node: hand position fixed, the hand turns amp_deg about `axis` (base frame) and back.
    Returns (s until the arm reacts after the hand turned back, max tool offset from where the hand puts it [m], q min,
    q max, tool rotation at the turn point as a rotation vector)."""
    dt, vmax = 1 / 60, math.radians(45)
    lo, hi, out_lo, out_hi = soft_limits(5.0)
    q_cmd = HOME_REAL.copy()
    ref = fk(q_cmd)
    href_R, target, cmd_R = np.eye(3), ref[:3, 3].copy(), ref[:3, :3].copy()
    half = amp_deg / rate_deg
    qs, off, rv_turn = [], 0.0, None
    for k in range(int((2 * half + 1.0) / dt)):
        t = k * dt
        R_h = rot(axis, math.radians(rate_deg) * (min(t, half) - min(max(0.0, t - half), half)))
        target = toward_point(target, ref[:3, 3], 0.5 * dt)
        cmd_R = toward_rot(cmd_R, flip_rotation(R_h @ href_R.T, signs) @ ref[:3, :3], math.radians(90) * dt)
        ach = fk(q_cmd)
        target = toward_point(ach[:3, 3], target, 0.05)
        cmd_R = toward_rot(ach[:3, :3], cmd_R, math.radians(15))
        T = np.eye(4)
        T[:3, 3], T[:3, :3] = target, cmd_R
        q_ik, pe, re = ik_priority(T, q_cmd, lo, hi)
        if limits_hit(q_ik, lo, hi) and re > 1.0:
            R_reached = fk(q_ik)[:3, :3]
            href_R, cmd_R = drop_unreachable_rot(R_h, R_reached, ref[:3, :3], 1.0, signs), R_reached.copy()
        q_cmd = np.clip(step_toward(q_cmd, q_ik, vmax * dt), np.minimum(out_lo, q_cmd), np.maximum(out_hi, q_cmd))
        qs.append(q_cmd.copy())
        off = max(off, float(np.linalg.norm(fk(q_cmd)[:3, 3] - ref[:3, 3])))
        if rv_turn is None and t >= half:
            rv_turn = _log_so3(fk(q_cmd)[:3, :3] @ ref[:3, :3].T)
    turn = int(half / dt)
    back = next(i for i in range(turn, len(qs)) if np.degrees(np.max(np.abs(qs[i] - qs[turn]))) > 2.0)
    qs = np.degrees(np.array(qs))
    return (back - turn) * dt, off, qs.min(0), qs.max(0), rv_turn


def test_full_pose_hand_turning_past_a_limit():
    """The case from the real robot (2026-10-21): with orientation:=true the arm moved 0.8 m while the hand only turned
    past the wrist's limits. Now the tool stays where the hand puts it and the wrist turns back at once."""
    ref = fk(HOME_REAL)
    for axis in (ref[:3, 2], ref[:3, 1], ref[:3, 0], np.array([0, 0, 1.0])):    # tool z / y / x, base z
        back, off, _, _, _ = _replay_full_pose(axis, 300)
        assert off < 0.005 and back < 0.25
    _, _, qmin, qmax, _ = _replay_full_pose(ref[:3, 2], 300)                  # twist past joint 6: only joint 6 moves
    assert np.all((qmax - qmin)[:5] < 1.0) and qmax[5] == pytest.approx(174.0, abs=0.1)


def test_rot_axis_sign_reverses_the_roll_only():
    """rot_axis_sign "-1 1 1" (run.sh real): a roll of the hand about the base x axis turns the tool the other way about
    x; a turn about the base z axis is unchanged."""
    x, z = np.eye(3)[0], np.eye(3)[2]
    *_, rv = _replay_full_pose(x, 40, signs=(-1, 1, 1))
    assert rv[0] == pytest.approx(-math.radians(40), abs=math.radians(2)) and abs(rv[1]) < 0.02 and abs(rv[2]) < 0.02
    *_, rv = _replay_full_pose(x, 40)
    assert rv[0] == pytest.approx(math.radians(40), abs=math.radians(2))
    *_, rv = _replay_full_pose(z, 40, signs=(-1, 1, 1))
    assert rv[2] == pytest.approx(math.radians(40), abs=math.radians(2))


def test_scale_into_lead_keeps_the_step_direction():
    from rebel_demo.kinematics import scale_into_lead
    lead = math.radians(20)
    q_prev, q_meas = np.zeros(6), np.zeros(6)
    q_new = np.radians([5, 0, 0, 40, 0, 10])                  # joint 4 would end 40 deg ahead of the robot
    q = scale_into_lead(q_prev, q_new, q_meas, lead)
    assert np.allclose(q, q_new * 0.5)                        # every joint shortened by the same factor (direction kept)
    assert np.all(np.abs(q - q_meas) <= lead + 1e-12)
    far = np.radians([0, 0, 0, 30, 0, 0])                     # already 30 deg ahead: may only move back
    assert np.allclose(scale_into_lead(far, far + 0.01, q_meas, lead), far)
    assert np.allclose(scale_into_lead(far, far - 0.01, q_meas, lead), far - 0.01)
    assert np.allclose(scale_into_lead(q_prev, q_new, None, lead), q_new)            # no feedback: unchanged
