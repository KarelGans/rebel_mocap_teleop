"""igus ReBeL (rebel2) 6-DoF kinematics: numbers from igus_rebel_description's igus_rebel2_arm_macro.xacro
(the arm of dual_arm_rig_v2.urdf.xacro that the Isaac rig uses, same for arm_left_ / arm_right_). fk() / ik() work in tool0 (link6 + 52.7 mm)."""
import math

import numpy as np
# from moveit_msgs.srv import GetPositionIK   # unused for now; needs MoveIt installed (keep kinematics numpy-only)

JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]
JOINT_LABELS = ["J1 base", "J2 shoulder", "J3 elbow", "J4 forearm roll", "J5 wrist", "J6 flange roll"]
JOINT_COLORS = [(0.90, 0.10, 0.10), (1.00, 0.55, 0.00), (0.95, 0.85, 0.10),
                (0.20, 0.80, 0.20), (0.10, 0.60, 1.00), (0.70, 0.30, 1.00)]

# (origin xyz [m], origin rpy [rad], axis, lower [rad], upper [rad]
# for rviz
# JOINTS = [
#     ((0, 0, 0.100), (0, 0, 0), (0, 0, -1), -math.pi * 179 / 180, math.pi * 179 / 180),
#     ((0, 0, 0.149), (0, math.pi / 6, 0), (0, 1, 0), -math.pi * 11 / 18, math.pi * 11 / 18),
#     ((0, 0, 0.2384), (0, math.pi / 6, 0), (0, 1, 0), -math.pi * 11 / 18, math.pi * 11 / 18),
#     ((0, 0, 0.149 - 0.03), (0, 0, 0), (0, 0, 1), -math.pi * 179 / 180, math.pi * 179 / 180),
#     ((0, 0, 0.14 + 0.03), (0, -math.pi / 24, 0), (0, 1, 0),
#      -math.pi * 19 / 36 + math.pi / 24, math.pi * 19 / 36 + math.pi / 24),
#     ((0, 0, 0.1208), (0, 0, 0), (0, 0, 1), -math.pi * 179 / 180, math.pi * 179 / 180),
# ]

# geometry: igus_rebel2_arm_macro.xacro (fixed motor joints merged).
# Limits: the real ReBeL, as used by the robot PC's mocap follower (its URDF; 2026-10-21): joint 2 -79..139 deg,
# joint 3 -79..139 deg, joint 5 +-94 deg, joints 1/4/6 +-178 deg (the Isaac rig model allows 1 deg more on every joint).
# The follower clamps targets 0.05 rad (2.9 deg) inside these, so teleop's limit_margin_deg must be larger than that.
# Limits = the robot's URDF / controller (fani8j/Igus_rebel, branch box-seam-detection): joints 1/4/6 +-178 deg
# (were +-179 here: our soft limit then had only 1.1 deg spare to the follower's clamp, 178 - 2.9 deg).
JOINTS = [
    ((0, 0, 0.1462),     (0, 0, math.pi), (0, 0, 1),  math.radians(-178.0), math.radians(178.0)),  # joint1
    ((0, 0.0265, 0.106),  (0, 0, 0),       (0, -1, 0), math.radians(-79.0), math.radians(139.0)),  # joint2
    ((0, -0.0265, 0.24152), (0, 0, 0),     (0, -1, 0), math.radians(15.0), math.radians(139.0)),  # joint3 (lower: elbow-up bound, not the robot's -79)
    ((0.001345, 0, 0.157479), (0, 0, 0),   (0, 0, 1),  math.radians(-178.0), math.radians(178.0)),  # joint4
    ((0, 0, 0.142),       (0, 0, 0),       (0, -1, 0), math.radians(-94.0), math.radians(94.0)),   # joint5
    ((0, 0, 0.0768),      (0, 0, 0),       (0, 0, 1),  math.radians(-178.0), math.radians(178.0)),  # joint6
]

LOWER = np.array([j[3] for j in JOINTS])
UPPER = np.array([j[4] for j in JOINTS])
HW_LOWER = LOWER.copy()
HW_LOWER[2] = math.radians(-79.0)                 # the robot's joint-3 limit; LOWER[2] is the IK's 15 deg elbow-up bound
VMAX = math.radians(45.0)                       # rad/s, every joint (URDF velocity limit)

# HOME: all joints at 0 = the arm straight up (a wrist singularity, gripper at the ceiling).
# Previous "ready" pose (tool0 35 cm in front of the base, 30 cm up, pointing down, elbow bent):
# HOME = np.radians([0.0, 13.4, 87.9, 0.0, 78.7, 0.0])
HOME = np.radians([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])


def parse_pose_deg(text):
    """"0 0 100 0 50 0" (degrees, spaces or commas) -> 6 joint angles [rad]. Raises ValueError if it is not 6 numbers
    or a joint is outside its limit."""
    try:
        vals = [float(v) for v in str(text).replace(",", " ").split()]
    except ValueError as e:
        raise ValueError(f"home_deg {text!r}: not numbers") from e
    if len(vals) != 6:
        raise ValueError(f"home_deg {text!r}: need 6 angles in degrees, got {len(vals)}")
    q = np.radians(vals)
    lo = HW_LOWER
    bad = [f"joint{i + 1}={vals[i]:g} (allowed {math.degrees(lo[i]):.0f}..{math.degrees(UPPER[i]):.0f})"
           for i in range(6) if q[i] < lo[i] - 1e-9 or q[i] > UPPER[i] + 1e-9]
    if bad:
        raise ValueError(f"home_deg {text!r}: outside the joint limits: " + ", ".join(bad))
    return q


def soft_limits(margin_deg):
    """Joint ranges that keep `margin_deg` away from the robot's limits.
    Returns (ik_lo, ik_hi, out_lo, out_hi) [rad]: the IK range (also keeps the 15 deg elbow-up bound) and the range a
    published command may never leave (the hardware limits minus the margin)."""
    if not 0.0 <= margin_deg < 45.0:
        raise ValueError(f"limit_margin_deg must be 0..45, got {margin_deg}")
    m = math.radians(margin_deg)
    out_lo, out_hi = HW_LOWER + m, UPPER - m
    return np.maximum(LOWER, out_lo), out_hi.copy(), out_lo, out_hi


def clamp_roll(q6_ref, twist_acc, gain, lo, hi):
    """Joint-6 target q6_ref + gain * twist_acc clipped to [lo, hi], with anti-windup: when it clips, the accumulated
    twist is wound back to the limit, so turning the hand back moves the joint at once. Returns (q6, twist_acc)."""
    q6 = q6_ref + gain * twist_acc
    clipped = float(np.clip(q6, lo, hi))
    if clipped != q6 and gain != 0.0:
        twist_acc = (clipped - q6_ref) / gain
    return clipped, twist_acc


def drop_unreachable(hand_ref_p, wanted_p, reached_p, scale_trans):
    """Keep following at a limit: move the hand anchor so that the mapped target (anchor-relative hand motion x scale)
    equals the point the arm can actually reach. The hand motion beyond the limit is forgotten, so the arm follows again as
    soon as the hand moves any other way. Returns the new anchor position."""
    return np.asarray(hand_ref_p, float) + (np.asarray(wanted_p, float) - np.asarray(reached_p, float)) / scale_trans


def rot(axis, a):
    x, y, z = axis
    c, s, C = math.cos(a), math.sin(a), 1 - math.cos(a)
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                     [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                     [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


def rpy(r, p, y):
    return rot((0, 0, 1), y) @ rot((0, 1, 0), p) @ rot((1, 0, 0), r)


_FLANGE_R = rpy(0, 0, math.pi)            # link_8 -> flange (tool0): x axis points out of the flange
_FLANGE_P = np.array([0, 0, 0.0527])


def quat_to_matrix(q):
    """[x, y, z, w] -> 3x3 rotation matrix."""
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def joint_frames(q):
    """[(position, rotation axis)] of each joint in base_link for joint angles q [rad]."""
    R, p, out = np.eye(3), np.zeros(3), []
    for (xyz, r_p_y, ax, _, _), qi in zip(JOINTS, q):
        p = p + R @ np.array(xyz, float)
        R = R @ rpy(*r_p_y)
        out.append((p.copy(), R @ np.array(ax, float)))
        R = R @ rot(ax, qi)
    return out


def fk(q):
    """4x4 flange (tool0) pose in base_link."""
    R, p = np.eye(3), np.zeros(3)
    for (xyz, r_p_y, ax, _, _), qi in zip(JOINTS, q):
        p = p + R @ np.array(xyz, float)
        R = R @ rpy(*r_p_y) @ rot(ax, qi)
    T = np.eye(4)
    T[:3, :3] = R @ _FLANGE_R
    T[:3, 3] = p + R @ _FLANGE_P
    return T


def scale_rotation(R, k):
    """Rotation matrix R with its angle multiplied by k about the same axis (k = 1: R itself, k = 0: identity)."""
    if k == 1.0:
        return R
    rv = _log_so3(R)                                # rotation vector = axis * angle
    angle = float(np.linalg.norm(rv))
    if angle < 1e-9:
        return np.eye(3)
    return rot(rv / angle, k * angle)


def twist_about(R, axis):
    """Signed angle [rad, [-pi, pi]] of rotation matrix R about the unit vector `axis` (swing-twist decomposition:
    only the part of R that spins around `axis`; a tilt away from it contributes ~0)."""
    t = np.trace(R)
    if t > 0:                                       # rotation matrix -> quaternion (x, y, z, w), Shepperd's method
        s = 2 * math.sqrt(t + 1)
        v, w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / s, s / 4
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2 * math.sqrt(1 + R[i, i] - R[j, j] - R[k, k])
        v = np.zeros(3)
        v[i], v[j], v[k] = s / 4, (R[j, i] + R[i, j]) / s, (R[k, i] + R[i, k]) / s
        w = (R[k, j] - R[j, k]) / s
    if w < 0:                                       # q and -q are the same rotation: keep the short way round
        v, w = -v, -w
    return 2.0 * math.atan2(float(np.dot(v, np.asarray(axis, float))), w)


# ---------------------------------------------------------------- IK (damped least squares)
def _log_so3(Rm):
    c = np.clip((np.trace(Rm) - 1) / 2, -1, 1)
    th = math.acos(c)
    v = np.array([Rm[2, 1] - Rm[1, 2], Rm[0, 2] - Rm[2, 0], Rm[1, 0] - Rm[0, 1]])
    if th < 1e-6:
        return 0.5 * v
    if th > math.pi - 1e-4:                         # near 180 deg: robust fallback
        w, V = np.linalg.eigh((Rm + Rm.T) / 2)
        axis = V[:, np.argmax(w)]
        return th * axis
    return th / (2 * math.sin(th)) * v


def _err(T, Tg, w_rot):
    return np.r_[Tg[:3, 3] - T[:3, 3], w_rot * _log_so3(Tg[:3, :3] @ T[:3, :3].T)]


_CHAIN = [(np.array(xyz, float), rpy(*r_p_y), np.array(ax, float), ax) for xyz, r_p_y, ax, _, _ in JOINTS]


def limits_hit(q, lower=None, upper=None, margin_deg=1.0):
    """Joints within margin_deg of a limit: [(joint_index, "lower" | "upper")] (default: LOWER / UPPER)."""
    lo = LOWER if lower is None else lower
    up = UPPER if upper is None else upper
    m = math.radians(margin_deg)
    out = []
    for i, qi in enumerate(np.asarray(q, float)):
        if qi <= lo[i] + m:
            out.append((i, "lower"))
        elif qi >= up[i] - m:
            out.append((i, "upper"))
    return out


def limit_lead(q_cmd, q_meas, max_lead):
    """q_cmd clipped to q_meas +- max_lead per joint [rad]: the command never runs further ahead of the measured joints.
    max_lead <= 0 or no measurement -> q_cmd unchanged (a copy)."""
    q_cmd = np.asarray(q_cmd, float)
    if q_meas is None or max_lead <= 0:
        return q_cmd.copy()
    q_meas = np.asarray(q_meas, float)
    return np.clip(q_cmd, q_meas - max_lead, q_meas + max_lead)


def lead_step(q_prev, q, q_meas, max_lead, max_step):
    """One cycle of the lead limit for a slow real arm: the command q clipped to q_meas +- max_lead (limit_lead), moving
    from q_prev by at most max_step on the largest joint (all joints scaled together), so coming back into the window
    after the arm fell behind never jumps. q_meas None (the measured joints are missing or stale) -> hold at q_prev.
    max_lead <= 0 -> q unchanged."""
    q = np.asarray(q, float)
    if max_lead <= 0:
        return q.copy()
    if q_meas is None:
        return np.array(q_prev, float)
    out = limit_lead(q, q_meas, max_lead)
    q_prev = np.asarray(q_prev, float)
    m = float(np.max(np.abs(out - q_prev)))
    return out if m <= max_step else q_prev + (out - q_prev) * (max_step / m)


def fk_jac(q):
    """Flange pose (4x4, base_link) and its geometric Jacobian (6x6: rows 0-2 linear, 3-5 angular) in one pass."""
    R, p = np.eye(3), np.zeros(3)
    origins, axes = [], []
    for (xyz, R0, axv, ax), qi in zip(_CHAIN, q):
        p = p + R @ xyz
        R = R @ R0
        a = R @ axv
        origins.append(p.copy())
        axes.append(a)
        R = R @ rot(ax, qi)
    T = np.eye(4)
    T[:3, :3] = R @ _FLANGE_R
    T[:3, 3] = p + R @ _FLANGE_P
    J = np.empty((6, 6))
    for i, (o, a) in enumerate(zip(origins, axes)):
        J[:3, i] = np.cross(a, T[:3, 3] - o)
        J[3:, i] = a
    return T, J


def _dls_step(J, e, free, lam, q, q_rest, k_null):
    """Damped least-squares joint step using only the `free` joints (pinned joints stay put)."""
    Jf = J[:, free]
    Jp = Jf.T @ np.linalg.inv(Jf @ Jf.T + lam ** 2 * np.eye(J.shape[0]))
    sf = Jp @ e
    if q_rest is not None:
        sf = sf + (np.eye(Jf.shape[1]) - Jp @ Jf) @ (k_null * (q_rest - q)[free])
    step = np.zeros(6)
    step[free] = sf
    return step


def ik(T_goal, q_seed, w_rot=0.3, iters=60, lam=0.02, q_rest=HOME, k_null=0.05, lower=None, upper=None):
    """Flange pose T_goal (4x4, base_link) -> joint angles [rad] (damped least squares, analytic Jacobian).
    w_rot = 0 -> position only (wrist free, pulled gently towards q_rest).
    Joint limits are handled by pinning: a joint whose step would cross LOWER / UPPER is held at the limit and the
    step is solved again for the remaining joints, so an unreachable target gives the closest reachable pose (the arm
    slides along the limit) instead of a corner posture far from it.
    Returns (q, position_error_m, rotation_error_deg)."""
    lo = LOWER if lower is None else np.asarray(lower, float)     # joint range the IK may use (default: the limits)
    hi = UPPER if upper is None else np.asarray(upper, float)
    q = np.clip(np.asarray(q_seed, float).copy(), lo, hi)
    rows = 3 if w_rot == 0 else 6
    for _ in range(iters):
        T, J = fk_jac(q)
        e = _err(T, T_goal, w_rot)[:rows]
        J = J[:rows].copy()
        J[3:] *= w_rot
        free = np.ones(6, bool)
        fixed = np.zeros(6)                                   # joint moves that are pinned at a limit
        for _pass in range(4):
            step = fixed + _dls_step(J, e - J @ fixed, free, lam, q, q_rest, k_null)
            step = np.where(free, np.clip(step, -0.3, 0.3), fixed)
            bad = free & ((q + step < lo) | (q + step > hi))
            if not bad.any():
                break
            free &= ~bad                                      # pin them at the limit, re-solve the others
            fixed[bad] = np.clip(q + step, lo, hi)[bad] - q[bad]
        if np.linalg.norm(e[:3]) < 1e-4 and np.linalg.norm(e[3:]) <= 1e-3 * w_rot and np.linalg.norm(step) < 1e-4:
            break
        q = np.clip(q + step, lo, hi)
    T = fk(q)
    pe = float(np.linalg.norm(T_goal[:3, 3] - T[:3, 3]))
    re = float(math.degrees(np.linalg.norm(_log_so3(T_goal[:3, :3] @ T[:3, :3].T))))
    return q, pe, re


# ---------------------------------------------------------------- full pose (orientation:=true): robot_remote's method
# github.com/Patrick-zhuyanxun/robot_remote, quest_teleop/quest_arm_teleop.py (IK, step_toward) and
# quest_dual_arm_teleop.py (rate-limited Cartesian command, anti-windup lead clamp).
def ik_priority(T_goal, q_seed, lower=None, upper=None, iters=30, damping=0.02, rot_weight=1.0, tol=1e-4):
    """Task-priority damped-least-squares IK (robot_remote's SerialChain.ik): position is the primary task, orientation
    is solved in the null space of the position task, so an orientation the arm cannot realise never moves the tool.
    A joint sitting on a limit and pushed further by the position step is frozen for that iteration (the position slides
    along the limit). One change: the orientation step is shortened so that no joint leaves its range, instead of
    re-solving it with the other joints. At a wrist limit the orientation is left short and the arm does not reconfigure
    (re-solving made joints 4 / 5 travel up to 100 deg and the tool wander 0.7 m while the hand only turned).
    Returns (q, position_error_m, rotation_error_deg) like ik()."""
    lo = LOWER if lower is None else np.asarray(lower, float)
    hi = UPPER if upper is None else np.asarray(upper, float)
    q = np.clip(np.asarray(q_seed, float).copy(), lo, hi)
    lam2 = damping ** 2
    for _ in range(iters):
        T, J = fk_jac(q)
        e_p = T_goal[:3, 3] - T[:3, 3]
        e_r = rot_weight * _log_so3(T_goal[:3, :3] @ T[:3, :3].T)
        if np.linalg.norm(e_p) < tol and np.linalg.norm(e_r) < tol:
            break
        free = np.ones(6, bool)
        for _pass in range(2):
            jp = J[:3] * free
            jp_pinv = jp.T @ np.linalg.inv(jp @ jp.T + lam2 * np.eye(3))
            dq1 = jp_pinv @ e_p
            # 1e-9: joint 6 does not move the tool point (dq1 ~ 1e-17 noise); freezing it on that noise would hand its
            # rotation to the other joints
            blocked = free & (((q <= lo + 1e-6) & (dq1 < -1e-9)) | ((q >= hi - 1e-6) & (dq1 > 1e-9)))
            if not blocked.any():
                break
            free &= ~blocked
        jr = J[3:] * free
        null = np.eye(6) - jp_pinv @ jp
        jr_n = jr @ null
        dq2 = null @ jr_n.T @ np.linalg.solve(jr_n @ jr_n.T + lam2 * np.eye(3), e_r - jr @ dq1)
        dq1, dq2 = dq1 * free, dq2 * free
        base = np.clip(q + dq1, lo, hi)
        s = 1.0                                       # largest share of the orientation step that stays in range
        for j in np.nonzero(np.abs(dq2) > 1e-12)[0]:
            s = min(s, max(0.0, ((hi[j] if dq2[j] > 0 else lo[j]) - base[j]) / dq2[j]))
        q = np.clip(base + s * dq2, lo, hi)
    T = fk(q)
    pe = float(np.linalg.norm(T_goal[:3, 3] - T[:3, 3]))
    re = float(math.degrees(np.linalg.norm(_log_so3(T_goal[:3, :3] @ T[:3, :3].T))))
    return q, pe, re


def step_toward(q_from, q_to, max_dq):
    """robot_remote's joint speed limit: move toward q_to by at most max_dq on the largest joint, scaling ALL joints
    uniformly. Per-joint clipping bends the joint-space direction and made the tool swing up to 15 cm during fast wrist
    turns; uniform scaling keeps the IK's direction."""
    q_from, q_to = np.asarray(q_from, float), np.asarray(q_to, float)
    dq = q_to - q_from
    m = float(np.max(np.abs(dq)))
    return q_from + (dq * (max_dq / m) if m > max_dq else dq)


def toward_point(p_from, p_to, max_dist):
    """p_from moved toward p_to by at most max_dist (rate limit; anti-windup lead clamp with p_from = the reached point)."""
    p_from, p_to = np.asarray(p_from, float), np.asarray(p_to, float)
    d = p_to - p_from
    n = float(np.linalg.norm(d))
    return p_to.copy() if n <= max_dist else p_from + d * (max_dist / n)


def toward_rot(R_from, R_to, max_angle):
    """R_from turned toward R_to by at most max_angle [rad] about their common axis (rate limit / lead clamp)."""
    rv = _log_so3(np.asarray(R_to, float) @ np.asarray(R_from, float).T)
    a = float(np.linalg.norm(rv))
    return np.array(R_to, float) if a <= max_angle else rot(rv / a, max_angle) @ R_from


def flip_rotation(R, signs):
    """Rotation R with its rotation-vector components (base-frame x, y, z) multiplied by `signs` (+-1 each): e.g.
    (-1, 1, 1) reverses the turn about x and keeps the turns about y and z. Its own inverse: flip(flip(R)) = R."""
    rv = np.asarray(signs, float) * _log_so3(np.asarray(R, float))
    a = float(np.linalg.norm(rv))
    return np.eye(3) if a < 1e-12 else rot(rv / a, a)


def drop_unreachable_rot(R_hand, R_reached, R_robot_ref, scale_rot, signs=(1.0, 1.0, 1.0)):
    """Keep following at a limit, rotation: the hand anchor rotation that makes the mapped target
    scale_rotation(flip_rotation(R_hand @ anchor.T, signs), scale_rot) @ R_robot_ref equal the rotation the arm reaches.
    The hand rotation beyond the limit is forgotten, so the wrist follows again as soon as the hand turns back."""
    E = scale_rotation(np.asarray(R_reached, float) @ np.asarray(R_robot_ref, float).T, 1.0 / scale_rot)
    if any(s < 0 for s in signs):
        E = flip_rotation(E, signs)
    return E.T @ np.asarray(R_hand, float)


def scale_into_lead(q_prev, q_new, q_meas, max_lead):
    """The step q_prev -> q_new shortened (all joints by the same factor) so no joint leaves q_meas +- max_lead that was
    inside it; a joint already outside may only move back. Like limit_lead, but keeps the step's direction, so the tool
    stays on its path while the slow real arm catches up (per-joint clipping bent the path by ~15 mm in a wrist turn)."""
    q_prev, q_new = np.asarray(q_prev, float), np.asarray(q_new, float)
    if q_meas is None or max_lead <= 0:
        return q_new.copy()
    lo, hi = np.asarray(q_meas, float) - max_lead, np.asarray(q_meas, float) + max_lead
    d = q_new - q_prev
    s = 1.0
    for j in np.nonzero(np.abs(d) > 1e-12)[0]:
        room = (hi[j] if d[j] > 0 else lo[j]) - q_prev[j]
        s = min(s, max(0.0, room / d[j]))
    return q_prev + s * d
