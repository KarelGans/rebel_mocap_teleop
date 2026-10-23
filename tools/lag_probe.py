#!/usr/bin/env python3
"""How far the real arm lags behind our joint commands - a passive probe for the real robot (publishes nothing).

Run in its own terminal on the robot's ROS domain while you teleoperate:
    ROS_DOMAIN_ID=12 python3 tools/lag_probe.py
Every few seconds, per arm:
    commands  60 Hz, max gap 25 ms   = our stream (/dual_arm/isaac_joint_commands): steady 60 Hz is all teleop can do
    arm 480 ms behind                = the robot's /joint_states compared with our commands, time-shifted to fit best
    error now / max                  = how far the real joints are from our command (deg)
    follower <state> rejected +N     = /mocap_follower_<arm>/status, if the robot PC runs the mocap follower
Only windows where our commands moved are measured (move the hand). The lag is the robot side's: the follower's kp
(lag ~ 1/kp s), its max_velocity / max_acceleration, MoveIt Servo and the controller.

  --arm both | left | right     --window SECONDS (6)     --max-lag SECONDS (3)
"""
import argparse
import json
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class Stream:
    """Receive time + 6 joint positions of one arm, from any JointState topic."""

    def __init__(self, names):
        self.names, self.t, self.q = names, [], []

    def add(self, msg):
        if all(n in msg.name for n in self.names):
            self.t.append(time.time())
            self.q.append([msg.position[msg.name.index(n)] for n in self.names])

    def window(self, t0):
        t, q = np.array(self.t), np.array(self.q)
        keep = t >= t0 - 5.0                                   # a few seconds of history for the time shift
        self.t, self.q = list(t[keep]), list(q[keep])
        return t, q


def best_lag(ct, cq, at, aq, t0, max_lag):
    """Shift tau that makes the arm's joints best match our commands from tau seconds earlier."""
    m = at >= t0
    best = None
    for tau in np.arange(0.0, max_lag, 0.01):
        c = np.stack([np.interp(at[m] - tau, ct, cq[:, j]) for j in range(6)], 1)
        cost = np.mean((aq[m] - c) ** 2)
        if best is None or cost < best[1]:
            best = (tau, cost)
    return best[0]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm", default="both", choices=["both", "left", "right"])
    ap.add_argument("--window", type=float, default=6.0)
    ap.add_argument("--max-lag", type=float, default=3.0)
    ap.add_argument("--commands", default="/dual_arm/isaac_joint_commands")
    ap.add_argument("--joint-states", default="/joint_states")
    args = ap.parse_args()
    arms = ["left", "right"] if args.arm == "both" else [args.arm]

    rclpy.init()
    n = Node("lag_probe")
    cmd = {a: Stream([f"arm_{a}_joint{i}" for i in range(1, 7)]) for a in arms}
    act = {a: Stream([f"arm_{a}_joint{i}" for i in range(1, 7)]) for a in arms}
    status = {a: {} for a in arms}
    n.create_subscription(JointState, args.commands, lambda m: [cmd[a].add(m) for a in arms], qos_profile_sensor_data)
    n.create_subscription(JointState, args.joint_states, lambda m: [act[a].add(m) for a in arms], qos_profile_sensor_data)

    def on_status(arm, msg):
        try:
            status[arm].update(json.loads(msg.data))
        except ValueError:
            pass
    for a in arms:
        n.create_subscription(String, f"/mocap_follower_{a}/status", lambda m, a=a: on_status(a, m), 10)
    print(f"lag_probe: {args.commands} vs {args.joint_states}, every {args.window:.0f} s - move the hand (Ctrl-C to stop)")

    rejected = {a: None for a in arms}
    t0 = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.01)
            if time.time() - t0 < args.window:
                continue
            for a in arms:
                ct, cq = cmd[a].window(t0)
                at, aq = act[a].window(t0)
                s = status[a]
                fol = ""
                if s:
                    r = s.get("rejected")
                    new = "" if r is None or rejected[a] is None else f", rejected +{r - rejected[a]}"
                    rejected[a] = r if r is not None else rejected[a]
                    fol = f" | follower {s.get('state', '?')}{new}" + (f", clamped {s['clamped']}" if s.get("clamped") else "")
                w = ct >= t0
                if np.count_nonzero(w) < 2:
                    print(f"[{a:5s}] no commands from teleop on {args.commands}{fol}")
                    continue
                gaps = np.diff(ct[w])
                head = f"[{a:5s}] commands {1 / np.median(gaps):3.0f} Hz, max gap {gaps.max() * 1000:4.0f} ms"
                if np.count_nonzero(at >= t0) < 2:
                    print(f"{head} | no robot joints on {args.joint_states}{fol}")
                    continue
                moved = np.degrees(np.ptp(cq[w], axis=0).max())
                err = np.degrees(np.abs(aq[at >= t0] - np.stack(
                    [np.interp(at[at >= t0], ct, cq[:, j]) for j in range(6)], 1)).max(axis=1))
                tail = f"error now {err[-1]:4.1f} deg, max {err.max():4.1f} deg{fol}"
                if moved < 1.0:
                    print(f"{head} | commands still ({moved:.1f} deg) - no lag measured | {tail}")
                    continue
                lag = best_lag(ct, cq, at, aq, t0, args.max_lag)
                print(f"{head} | arm {lag * 1000:4.0f} ms behind (commands moved {moved:.0f} deg) | {tail}")
            t0 = time.time()
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
