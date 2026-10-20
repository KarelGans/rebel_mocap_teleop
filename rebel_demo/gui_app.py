"""rebel_ui - one window instead of five terminals.

Start / stop the mocap driver and the teleop launch (with live logs), see at a glance what works (status lights and
"what to do next" hints), engage / pause / home / calibrate with buttons or the keyboard, and edit the launch settings
as named profiles. It only wraps the same commands as scripts/run.sh; the teleop nodes are unchanged.

  scripts/run.sh ui            (sources ROS + both workspaces and the domain, then starts this window)
  SPACE = pause / resume both arms     L / R = left / right arm only     (when the window has focus)

The ROS domain is taken from the environment the window is started in (ROS_DOMAIN_ID=12 scripts/run.sh ui).
"""
import os
import queue
import re
import signal
import subprocess
import sys
import threading
import time

from PyQt5 import QtCore, QtGui, QtWidgets

from .gui_logic import (DRIVER_CMD, DRIVER_NODE, BUILTIN_PROFILES, ProfileStore, arms_of, build_teleop_cmd, compute_hints,
                        format_cmd, joint_limits, joint_zone, normalize, validate, ypr_deg_to_text, ypr_text_to_deg)

GREEN, AMBER, RED, GRAY = "#2e9e4f", "#e0a000", "#d03030", "#8a8a8a"
HOME_RE = re.compile(r"\[(rebel_teleop\w*)\]: at HOME")


# ----------------------------------------------------------------------------------------------------- ROS side
class RosBridge(QtCore.QObject):
    """rclpy in its own thread: subscriptions for the status lights, service calls for the buttons.
    Everything touching rclpy runs in that thread; the window talks to it through a queue and Qt signals."""
    snapshot = QtCore.pyqtSignal(dict)
    reply = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._jobs = queue.Queue()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self.error = None

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2.0)

    def configure(self, arms):
        self._jobs.put(("configure", arms))

    def engage(self, suffix, on):
        self._jobs.put(("engage", suffix, on))

    def trigger(self, node, service):
        self._jobs.put(("trigger", node, service))

    def lifecycle(self, transition):
        self._jobs.put(("lifecycle", transition))

    # --------------------------------------------------------------------------------------------- thread body
    def _spin(self):
        try:
            import rclpy
            from lifecycle_msgs.msg import Transition
            from lifecycle_msgs.srv import ChangeState, GetState
            from rclpy.duration import Duration
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
            from rclpy.time import Time
            from rosgraph_msgs.msg import Clock
            from sensor_msgs.msg import JointState
            from std_msgs.msg import Bool
            from std_srvs.srv import SetBool, Trigger
            from tf2_ros import Buffer, TransformListener
        except ImportError as e:
            self.error = f"ROS python packages not found ({e}). Start the window with scripts/run.sh ui."
            self.reply.emit(self.error)
            return
        try:
            from mocap4r2_msgs.msg import RigidBodies
        except ImportError:
            RigidBodies = None

        rclpy.init()
        node = Node("rebel_ui")
        best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        counts = {"mocap": 0, "clock": 0, "states": 0, "cmd": 0}
        data = {"rb_names": [], "joints": {}, "engaged": {}, "driver_state": None}
        arms = []                                    # [(side, suffix, hand ID)]
        tf_buffer = Buffer()
        TransformListener(tf_buffer, node)

        if RigidBodies is not None:
            def on_rb(msg):
                counts["mocap"] += 1
                data["rb_names"] = [str(rb.rigid_body_name) for rb in msg.rigidbodies]
            node.create_subscription(RigidBodies, "/rigid_bodies", on_rb, best_effort)

        def on_cmd(msg):
            counts["cmd"] += 1
            data["joints"] = dict(zip(msg.name, msg.position))
        node.create_subscription(Clock, "/clock", lambda m: counts.__setitem__("clock", counts["clock"] + 1), best_effort)
        node.create_subscription(JointState, "/dual_arm/isaac_joint_states",
                                 lambda m: counts.__setitem__("states", counts["states"] + 1), best_effort)
        node.create_subscription(JointState, "/dual_arm/isaac_joint_commands", on_cmd, best_effort)
        for suffix in ("", "_left", "_right"):
            node.create_subscription(Bool, f"/rebel_teleop{suffix}/engaged",
                                     lambda m, s=suffix: data["engaged"].__setitem__(s, bool(m.data)), latched)

        clients = {}

        def client(kind, srv_type, name):
            key = (kind, name)
            if key not in clients:
                clients[key] = node.create_client(srv_type, name)
            return clients[key]

        def call(c, request, done):
            if not c.service_is_ready():
                self.reply.emit(f"service {c.srv_name} is not available (node not running, or another ROS domain?)")
                return
            c.call_async(request).add_done_callback(lambda f: self.reply.emit(done(f.result())))

        def handle(job):
            kind = job[0]
            if kind == "configure":
                arms[:] = job[1]
            elif kind == "engage":
                _, suffix, on = job
                req = SetBool.Request()
                req.data = bool(on)
                call(client("engage", SetBool, f"/rebel_teleop{suffix}/engage"), req,
                     lambda r, s=suffix: f"teleop{s}: {r.message}")
            elif kind == "trigger":
                _, nodename, service = job
                call(client("trig", Trigger, f"/{nodename}/{service}"), Trigger.Request(),
                     lambda r, n=nodename, sv=service: f"{n}/{sv}: {r.message}")
            elif kind == "lifecycle":
                req = ChangeState.Request()
                req.transition = Transition(id=job[1])
                call(client("life", ChangeState, f"/{DRIVER_NODE}/change_state"), req,
                     lambda r: "driver: transition " + ("done" if r.success else "refused"))

        last = {"t": time.monotonic(), "counts": dict(counts), "poll": 0.0}
        rates = {k: 0.0 for k in counts}
        graph = {"nodes": set(), "clock_pub": 0, "states_pub": 0, "cmd_pub": 0, "cmd_sub": 0, "driver_node": False}

        def poll_driver_state():
            c = client("getstate", GetState, f"/{DRIVER_NODE}/get_state")
            if c.service_is_ready():
                c.call_async(GetState.Request()).add_done_callback(
                    lambda f: data.__setitem__("driver_state", f.result().current_state.label))
            else:
                data["driver_state"] = None

        def tick():
            now = time.monotonic()
            dt = now - last["t"]
            if dt >= 1.0:
                for k in counts:
                    rates[k] = (counts[k] - last["counts"][k]) / dt
                last["t"], last["counts"] = now, dict(counts)
                graph["nodes"] = {n for n, _ in node.get_node_names_and_namespaces()}
                graph["driver_node"] = DRIVER_NODE in graph["nodes"]
                def others(infos):            # without this window's own subscriptions
                    return len([i for i in infos if i.node_name != "rebel_ui"])
                graph["clock_pub"] = others(node.get_publishers_info_by_topic("/clock"))
                graph["states_pub"] = others(node.get_publishers_info_by_topic("/dual_arm/isaac_joint_states"))
                graph["cmd_pub"] = others(node.get_publishers_info_by_topic("/dual_arm/isaac_joint_commands"))
                graph["cmd_sub"] = others(node.get_subscriptions_info_by_topic("/dual_arm/isaac_joint_commands"))
                if graph["driver_node"]:
                    poll_driver_state()
                else:
                    data["driver_state"] = None
            hands = {}
            for side, suffix, hand_id in arms:
                try:
                    hands[side] = tf_buffer.can_transform(f"arm_{side}_base_link", f"rigid_body_{hand_id}", Time(),
                                                          Duration(seconds=0))
                except Exception:
                    hands[side] = False
            self.snapshot.emit({
                "domain": os.environ.get("ROS_DOMAIN_ID", "0 (unset)"),
                "driver": {"node": graph["driver_node"], "state": data["driver_state"]},
                "mocap_hz": rates["mocap"], "rb_names": list(data["rb_names"]),
                "clock_pub": graph["clock_pub"], "clock_hz": rates["clock"],
                "states_pub": graph["states_pub"], "states_hz": rates["states"],
                "cmd_pub": graph["cmd_pub"], "cmd_sub": graph["cmd_sub"], "cmd_hz": rates["cmd"],
                "nodes": sorted(graph["nodes"]), "engaged": dict(data["engaged"]), "hands": hands,
                "joints": dict(data["joints"]), "have_mocap_msgs": RigidBodies is not None,
            })
        node.create_timer(0.25, tick)

        executor = SingleThreadedExecutor()
        executor.add_node(node)
        while rclpy.ok() and not self._stop.is_set():
            try:
                while True:
                    handle(self._jobs.get_nowait())
            except queue.Empty:
                pass
            executor.spin_once(timeout_sec=0.05)
        node.destroy_node()
        rclpy.try_shutdown()


# ------------------------------------------------------------------------------------------ process management
class LineReader(QtCore.QThread):
    line = QtCore.pyqtSignal(str)

    def __init__(self, stream):
        super().__init__()
        self._stream = stream

    def run(self):
        for raw in iter(self._stream.readline, ""):
            self.line.emit(raw.rstrip("\n"))


class ManagedProcess(QtCore.QObject):
    """One command in its own process group; output goes to `line`, stop() sends Ctrl+C like a terminal would."""
    line = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal(int)

    def __init__(self, name):
        super().__init__()
        self.name, self.proc, self._reader = name, None, None
        self._poll = QtCore.QTimer(self)
        self._poll.timeout.connect(self._check)
        self._stop_t0 = None

    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def start(self, cmd):
        if self.running():
            return
        self.line.emit("$ " + format_cmd(cmd))
        try:
            self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1,
                                         start_new_session=True)
        except OSError as e:
            self.line.emit(f"cannot start: {e}")
            self.proc = None
            return
        self._reader = LineReader(self.proc.stdout)
        self._reader.line.connect(self.line)
        self._reader.start()
        self._stop_t0 = None
        self._poll.start(300)

    def stop(self):
        if self.running() and self._stop_t0 is None:
            self._stop_t0 = time.monotonic()
            self._signal(signal.SIGINT)

    def _signal(self, sig):
        try:
            os.killpg(os.getpgid(self.proc.pid), sig)
        except (ProcessLookupError, PermissionError):
            pass

    def _check(self):
        if not self.running():
            self._poll.stop()
            rc = self.proc.returncode if self.proc else 0
            self.line.emit(f"[{self.name} ended, exit code {rc}]")
            self.finished.emit(rc)
            self.proc = None
            return
        if self._stop_t0 is not None:
            waited = time.monotonic() - self._stop_t0
            if waited > 12:
                self._signal(signal.SIGKILL)
            elif waited > 6:
                self._signal(signal.SIGTERM)


# --------------------------------------------------------------------------------------------------- widgets
def dot(color):
    return f'<span style="color:{color}; font-size:16px">&#9679;</span>'


class JointBar(QtWidgets.QWidget):
    """One joint: bar from its lower to its upper limit, marker at the commanded angle, amber / red near the limits."""

    def __init__(self, name, lo, hi):
        super().__init__()
        self.name, self.lo, self.hi, self.pos = name, lo, hi, None
        self.setMinimumHeight(20)
        self.setMinimumWidth(230)

    def set_pos(self, pos):
        if pos != self.pos:
            self.pos = pos
            self.update()

    def paintEvent(self, _):
        import math
        p = QtGui.QPainter(self)
        w, h = self.width(), self.height()
        x0, x1 = 78, w - 74
        p.setPen(QtGui.QColor("#555"))
        p.drawText(2, 0, 74, h, QtCore.Qt.AlignVCenter, self.name)
        p.fillRect(x0, 6, x1 - x0, h - 12, QtGui.QColor("#d9d9d9"))
        span = self.hi - self.lo
        for frac, col in ((10 / 180 * math.pi / span, "#f3d58a"), (3 / 180 * math.pi / span, "#e8a0a0")):
            ww = int((x1 - x0) * frac)
            p.fillRect(x0, 6, ww, h - 12, QtGui.QColor(col))
            p.fillRect(x1 - ww, 6, ww, h - 12, QtGui.QColor(col))
        if self.pos is not None:
            zone = joint_zone(self.pos, self.lo, self.hi)
            col = QtGui.QColor({"ok": GREEN, "warn": AMBER, "limit": RED}[zone])
            x = x0 + int((x1 - x0) * min(max((self.pos - self.lo) / span, 0), 1))
            p.fillRect(x - 2, 2, 5, h - 4, col)
            p.setPen(col)
            p.drawText(x1 + 6, 0, 68, h, QtCore.Qt.AlignVCenter, f"{math.degrees(self.pos):6.1f}°")


class LogPane(QtWidgets.QPlainTextEdit):
    def __init__(self):
        super().__init__()
        self.setReadOnly(True)
        self.setMaximumBlockCount(4000)
        self.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        self.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont))

    def add(self, text):
        bar = self.verticalScrollBar()
        at_end = bar.value() >= bar.maximum() - 4
        self.appendPlainText(text)
        if at_end:
            bar.setValue(bar.maximum())


# ------------------------------------------------------------------------------------------------ main window
class MainWindow(QtWidgets.QMainWindow):
    STEPS = ["Mocap driver running", "Driver activated", "Mocap frames arriving", "Robot side ready",
             "Teleop started", "Arms at HOME"]

    def __init__(self):
        super().__init__()
        self.setWindowTitle("rebel_mocap_teleop")
        self.resize(1180, 760)
        self.store = ProfileStore()
        self.profile = self.store.get(self.store.last if self.store.last in self.store.names() else "Real robot")
        self.snap = {}
        self.home_seen = set()
        self.last_toggle = 0.0
        self.sequence = None                       # running "Start all": dict(step, t0, ...)
        self.driver = ManagedProcess("driver")
        self.teleop = ManagedProcess("teleop")
        self.driver.line.connect(lambda t: self.driver_log.add(t))
        self.teleop.line.connect(self.on_teleop_line)
        self.teleop.finished.connect(lambda rc: self.home_seen.clear())

        self.bridge = RosBridge()
        self.bridge.snapshot.connect(self.on_snapshot)
        self.bridge.reply.connect(self.on_reply)

        self.build_ui()
        self.load_profile_into_widgets(self.profile)
        self.bridge.start()
        self.bridge.configure(arms_of(self.profile))

        for key, fn in ((QtCore.Qt.Key_Space, lambda: self.toggle(None)), (QtCore.Qt.Key_L, lambda: self.toggle("left")),
                        (QtCore.Qt.Key_R, lambda: self.toggle("right"))):
            sc = QtWidgets.QShortcut(QtGui.QKeySequence(key), self)
            sc.setContext(QtCore.Qt.ApplicationShortcut)
            sc.activated.connect(fn)
        QtWidgets.QShortcut(QtGui.QKeySequence(QtCore.Qt.Key_Escape), self).activated.connect(self.clear_focus)

        self.seq_timer = QtCore.QTimer(self)
        self.seq_timer.timeout.connect(self.advance_sequence)
        self.seq_timer.start(300)

    # ------------------------------------------------------------------------------------------- building
    def build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)

        top = QtWidgets.QHBoxLayout()
        top.addWidget(QtWidgets.QLabel("Profile:"))
        self.profile_box = QtWidgets.QComboBox()
        self.profile_box.addItems(self.store.names())
        self.profile_box.setCurrentText(self.store.last if self.store.last in self.store.names() else "Real robot")
        self.profile_box.currentTextChanged.connect(self.on_profile_changed)
        top.addWidget(self.profile_box)
        self.domain_label = QtWidgets.QLabel()
        self.domain_label.setText(f"ROS_DOMAIN_ID = <b>{os.environ.get('ROS_DOMAIN_ID', '0 (unset)')}</b>")
        top.addWidget(self.domain_label)
        top.addStretch(1)
        self.start_all_btn = self.button("Start all", self.start_all, "Driver, activate, teleop, in order, waiting for each step")
        self.stop_all_btn = self.button("Stop all", self.stop_all, "Stop what this window started (teleop first, then the driver)")
        top.addWidget(self.start_all_btn)
        top.addWidget(self.stop_all_btn)
        root.addLayout(top)

        body = QtWidgets.QHBoxLayout()
        root.addLayout(body, 1)

        left = QtWidgets.QVBoxLayout()
        body.addLayout(left, 0)
        status_box = QtWidgets.QGroupBox("Status")
        grid = QtWidgets.QGridLayout(status_box)
        self.lights = {}
        for row, (key, title) in enumerate((("driver", "Mocap driver"), ("mocap", "Mocap frames"), ("bodies", "Rigid bodies"),
                                            ("robot", "Isaac / robot side"), ("teleop", "Teleop nodes"), ("home", "At HOME"),
                                            ("hands", "Hands tracked"), ("engaged", "Engaged"), ("cmd", "Commands out"))):
            label = QtWidgets.QLabel(dot(GRAY) + " " + title)
            value = QtWidgets.QLabel("-")
            value.setStyleSheet("color:#444")
            grid.addWidget(label, row, 0)
            grid.addWidget(value, row, 1)
            self.lights[key] = (label, title, value)
        left.addWidget(status_box)
        steps_box = QtWidgets.QGroupBox("Start all - steps")
        sl = QtWidgets.QVBoxLayout(steps_box)
        self.step_labels = []
        for text in self.STEPS:
            lab = QtWidgets.QLabel("○ " + text)
            sl.addWidget(lab)
            self.step_labels.append(lab)
        left.addWidget(steps_box)
        hints_box = QtWidgets.QGroupBox("What to do next")
        hl = QtWidgets.QVBoxLayout(hints_box)
        self.hints = QtWidgets.QLabel("")
        self.hints.setWordWrap(True)
        self.hints.setMinimumWidth(300)
        self.hints.setAlignment(QtCore.Qt.AlignTop)
        hl.addWidget(self.hints)
        left.addWidget(hints_box, 1)

        self.tabs = QtWidgets.QTabWidget()
        body.addWidget(self.tabs, 1)
        self.tabs.addTab(self.build_control_tab(), "Control")
        self.tabs.addTab(self.build_settings_tab(), "Settings")
        logs = QtWidgets.QTabWidget()
        self.driver_log, self.teleop_log = LogPane(), LogPane()
        logs.addTab(self.teleop_log, "Teleop")
        logs.addTab(self.driver_log, "Mocap driver")
        self.tabs.addTab(logs, "Logs")
        self.statusBar().showMessage("Ready")

    def button(self, text, slot, tip=""):
        b = QtWidgets.QPushButton(text)
        b.setFocusPolicy(QtCore.Qt.NoFocus)           # SPACE must not click the last pressed button
        b.setToolTip(tip)
        b.clicked.connect(slot)
        return b

    def build_control_tab(self):
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        self.toggle_btn = self.button("PAUSE / RESUME   (SPACE)", lambda: self.toggle(None),
                                      "Engaged: pause all arms. Paused: engage all arms (only after 'at HOME').")
        self.toggle_btn.setMinimumHeight(90)
        self.toggle_btn.setStyleSheet("font-size:24px; font-weight:bold")
        v.addWidget(self.toggle_btn)
        row = QtWidgets.QHBoxLayout()
        self.left_btn = self.button("Left arm  (L)", lambda: self.toggle("left"))
        self.right_btn = self.button("Right arm  (R)", lambda: self.toggle("right"))
        self.home_btn = self.button("Home both", self.home_all, "Back to home_deg (also disengages)")
        self.calib_btn = self.button("Calibrate grippers", self.calibrate, "Hold both hands open; calibrates 3 seconds after the click")
        for b in (self.left_btn, self.right_btn, self.home_btn, self.calib_btn):
            b.setMinimumHeight(40)
            row.addWidget(b)
        v.addLayout(row)
        row2 = QtWidgets.QHBoxLayout()
        self.start_drv_btn = self.button("Start driver", self.start_driver)
        self.activate_btn = self.button("Activate driver", lambda: self.bridge.lifecycle(3))
        self.start_tel_btn = self.button("Start teleop", self.start_teleop)
        self.stop_tel_btn = self.button("Stop teleop", self.teleop.stop)
        self.stop_drv_btn = self.button("Stop driver", self.driver.stop)
        for b in (self.start_drv_btn, self.activate_btn, self.start_tel_btn, self.stop_tel_btn, self.stop_drv_btn):
            row2.addWidget(b)
        v.addLayout(row2)
        v.addWidget(QtWidgets.QLabel("<b>Commanded joints</b> (amber: within 10° of a limit, red: within 3°)"))
        cols = QtWidgets.QHBoxLayout()
        self.joint_bars = {}
        limits = joint_limits()
        for side in ("left", "right"):
            col = QtWidgets.QVBoxLayout()
            col.addWidget(QtWidgets.QLabel(f"{side} arm"))
            for i in range(6):
                name = f"arm_{side}_joint{i + 1}"
                bar = JointBar(f"joint {i + 1}", *limits[i])
                self.joint_bars[name] = bar
                col.addWidget(bar)
            cols.addLayout(col)
        v.addLayout(cols)
        v.addStretch(1)
        return w

    def build_settings_tab(self):
        w = QtWidgets.QWidget()
        outer = QtWidgets.QVBoxLayout(w)
        form = QtWidgets.QFormLayout()
        self.f = {}

        def spin(lo, hi, step, decimals=0):
            s = QtWidgets.QDoubleSpinBox() if decimals else QtWidgets.QSpinBox()
            s.setRange(lo, hi)
            s.setSingleStep(step)
            if decimals:
                s.setDecimals(decimals)
            s.valueChanged.connect(self.update_preview)
            return s

        def combo(items):
            c = QtWidgets.QComboBox()
            c.addItems(items)
            c.currentTextChanged.connect(self.update_preview)
            return c

        def line():
            e = QtWidgets.QLineEdit()
            e.textChanged.connect(self.update_preview)
            return e

        self.f["mode"] = combo(["real", "isaac"])
        form.addRow("Robot side", self.f["mode"])
        tf_row = QtWidgets.QHBoxLayout()
        self.f["robot_tf"] = combo(["own", "external"])
        self.f["robot_tf"].setToolTip("external: the robot PC publishes the arm TF on this ROS domain - do not start a second "
                                      "robot model (avoids TF_OLD_DATA)")
        self.f["joint_states_topic"] = line()
        self.f["joint_states_topic"].setPlaceholderText("robot joint states topic (optional, for the start pose)")
        tf_row.addWidget(self.f["robot_tf"])
        tf_row.addWidget(self.f["joint_states_topic"], 1)
        form.addRow("Robot TF / joint states", tf_row)
        self.f["rig"] = combo(["both", "left", "right"])
        form.addRow("Arms (rig)", self.f["rig"])
        ids = QtWidgets.QHBoxLayout()
        for key, label in (("left_hand", "left hand"), ("left_gripper", "left finger"), ("right_hand", "right hand"),
                           ("right_gripper", "right finger")):
            self.f[key] = spin(1, 999, 1)
            ids.addWidget(QtWidgets.QLabel(label))
            ids.addWidget(self.f[key])
        form.addRow("Motive streaming IDs", ids)
        self.f["scale_trans"] = spin(0.05, 5.0, 0.05, 2)
        self.f["scale_rot"] = spin(0.05, 5.0, 0.05, 2)
        self.f["cutoff_hz"] = spin(0.5, 30.0, 0.5, 1)
        self.f["max_joint_speed_deg"] = spin(1.0, 45.0, 1.0, 1)
        form.addRow("Translation scale", self.f["scale_trans"])
        form.addRow("Rotation scale", self.f["scale_rot"])
        form.addRow("Filter cutoff (Hz)", self.f["cutoff_hz"])
        form.addRow("Max joint speed (deg/s)", self.f["max_joint_speed_deg"])
        self.f["limit_margin_deg"] = spin(0.0, 44.0, 1.0, 1)
        self.f["limit_margin_deg"].setToolTip("Never command a joint closer than this to its hardware limit (default 10)")
        form.addRow("Joint limit margin (deg)", self.f["limit_margin_deg"])
        rec = QtWidgets.QHBoxLayout()
        self.f["limit_recovery"] = QtWidgets.QCheckBox("at a limit: back off, then follow the hand again")
        self.f["limit_recovery"].toggled.connect(self.update_preview)
        self.f["limit_backoff_deg"] = spin(0.5, 20.0, 0.5, 1)
        rec.addWidget(self.f["limit_recovery"])
        rec.addWidget(QtWidgets.QLabel("back-off (deg)"))
        rec.addWidget(self.f["limit_backoff_deg"])
        form.addRow("Limit recovery", rec)
        self.f["orientation"] = combo(["false", "semi", "true"])
        self.f["roll_sign"] = combo(["1", "-1"])
        self.f["gripper_mode"] = combo(["orientation", "distance"])
        form.addRow("Orientation mode", self.f["orientation"])
        form.addRow("Joint-6 roll direction (semi)", self.f["roll_sign"])
        form.addRow("Gripper mode", self.f["gripper_mode"])
        self.f["home_deg"] = line()
        form.addRow("Home pose, 6 angles (deg)", self.f["home_deg"])
        ypr = QtWidgets.QHBoxLayout()
        for key, label in (("yaw", "yaw"), ("pitch", "pitch"), ("roll", "roll")):
            self.f[key] = spin(-360, 360, 1, 1)
            ypr.addWidget(QtWidgets.QLabel(label))
            ypr.addWidget(self.f[key])
        form.addRow("Robot frame rotation (deg)", ypr)
        self.f["workspace_max"] = line()
        form.addRow("workspace_max (x y z, m)", self.f["workspace_max"])
        self.f["skip_home"] = QtWidgets.QCheckBox("robot already is at the start pose: no start move")
        self.f["skip_home"].toggled.connect(self.update_preview)
        form.addRow("Skip home", self.f["skip_home"])
        self.f["mocap_bridge"] = QtWidgets.QCheckBox("convert /rigid_bodies to TF (leave on for real mocap)")
        self.f["mocap_bridge"].toggled.connect(self.update_preview)
        form.addRow("mocap_tf", self.f["mocap_bridge"])
        self.f["extra"] = line()
        self.f["extra"].setPlaceholderText("more launch arguments, e.g. limit_warn_mm:=10")
        form.addRow("Extra arguments", self.f["extra"])
        outer.addLayout(form)
        btns = QtWidgets.QHBoxLayout()
        self.save_btn = self.button("Save", self.save_profile)
        self.save_as_btn = self.button("Save as...", self.save_profile_as)
        self.delete_btn = self.button("Delete profile", self.delete_profile)
        self.reset_btn = self.button("Reset to built-in", self.reset_profile)
        for b in (self.save_btn, self.save_as_btn, self.delete_btn, self.reset_btn):
            btns.addWidget(b)
        btns.addStretch(1)
        outer.addLayout(btns)
        self.problems = QtWidgets.QLabel("")
        self.problems.setStyleSheet(f"color:{RED}")
        self.problems.setWordWrap(True)
        outer.addWidget(self.problems)
        outer.addWidget(QtWidgets.QLabel("Command that 'Start teleop' runs (changes apply the next time teleop is started):"))
        self.preview = QtWidgets.QPlainTextEdit()
        self.preview.setReadOnly(True)
        self.preview.setMaximumHeight(110)
        self.preview.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont))
        outer.addWidget(self.preview)
        outer.addStretch(1)
        return w

    # ------------------------------------------------------------------------------------------- profiles
    def load_profile_into_widgets(self, profile):
        p = normalize(profile)
        self._loading = True
        for key, widget in self.f.items():
            if key in ("yaw", "pitch", "roll"):
                continue
            value = p[key]
            if isinstance(widget, QtWidgets.QComboBox):
                widget.setCurrentText(str(value))
            elif isinstance(widget, QtWidgets.QCheckBox):
                widget.setChecked(bool(value))
            elif isinstance(widget, QtWidgets.QLineEdit):
                widget.setText(str(value))
            else:
                widget.setValue(value if isinstance(widget, QtWidgets.QDoubleSpinBox) else int(value))
        try:
            for key, v in zip(("yaw", "pitch", "roll"), ypr_text_to_deg(p["robot_ypr"])):
                self.f[key].setValue(v)
        except ValueError:
            pass
        self._loading = False
        self.update_preview()

    def profile_from_widgets(self):
        p = {}
        for key, widget in self.f.items():
            if key in ("yaw", "pitch", "roll"):
                continue
            if isinstance(widget, QtWidgets.QComboBox):
                text = widget.currentText()
                p[key] = int(text) if key == "roll_sign" else text
            elif isinstance(widget, QtWidgets.QCheckBox):
                p[key] = widget.isChecked()
            elif isinstance(widget, QtWidgets.QLineEdit):
                p[key] = widget.text()
            else:
                p[key] = widget.value()
        p["robot_ypr"] = ypr_deg_to_text(self.f["yaw"].value(), self.f["pitch"].value(), self.f["roll"].value())
        return normalize(p)

    def update_preview(self, *_):
        if getattr(self, "_loading", False):
            return
        p = self.profile_from_widgets()
        self.profile = p
        errs = validate(p)
        self.problems.setText("\n".join("✖ " + e for e in errs))
        self.preview.setPlainText(format_cmd(build_teleop_cmd(p)))
        self.bridge.configure(arms_of(p))
        self.delete_btn.setEnabled(not self.store.is_builtin(self.profile_box.currentText()))

    def on_profile_changed(self, name):
        if name:
            self.load_profile_into_widgets(self.store.get(name))
            self.store.last = name
            self.store.flush()

    def save_profile(self):
        name = self.profile_box.currentText()
        if self.store.is_builtin(name):
            return self.save_profile_as()
        self.store.save(name, self.profile_from_widgets())
        self.statusBar().showMessage(f"Saved profile '{name}'", 4000)

    def save_profile_as(self):
        name, ok = QtWidgets.QInputDialog.getText(self, "Save profile", "Name of the new profile:")
        name = name.strip()
        if ok and name:
            self.store.save(name, self.profile_from_widgets())
            self.profile_box.blockSignals(True)
            self.profile_box.clear()
            self.profile_box.addItems(self.store.names())
            self.profile_box.setCurrentText(name)
            self.profile_box.blockSignals(False)
            self.update_preview()
            self.statusBar().showMessage(f"Saved profile '{name}'", 4000)

    def delete_profile(self):
        name = self.profile_box.currentText()
        if self.store.is_builtin(name):
            return
        self.store.delete(name)
        self.profile_box.blockSignals(True)
        self.profile_box.clear()
        self.profile_box.addItems(self.store.names())
        self.profile_box.blockSignals(False)
        self.profile_box.setCurrentText(self.store.last)
        self.load_profile_into_widgets(self.store.get(self.store.last))

    def reset_profile(self):
        name = self.profile_box.currentText()
        base = BUILTIN_PROFILES.get(name, BUILTIN_PROFILES["Real robot"])
        self.load_profile_into_widgets(base)

    # ---------------------------------------------------------------------------------------- start / stop
    def start_driver(self):
        if self.snap.get("driver", {}).get("node") and not self.driver.running():
            self.statusBar().showMessage("A mocap driver is already running (started elsewhere)", 5000)
            return
        self.driver.start(DRIVER_CMD)

    def start_teleop(self):
        errs = validate(self.profile)
        if errs:
            self.tabs.setCurrentIndex(1)
            QtWidgets.QMessageBox.warning(self, "Settings", "Cannot start teleop:\n\n" + "\n".join(errs))
            return False
        running = [n for n in self.snap.get("nodes", []) if n.startswith("rebel_teleop")]
        if running and not self.teleop.running():
            self.statusBar().showMessage("Teleop nodes already running (started elsewhere)", 5000)
            return False
        self.home_seen.clear()
        self.teleop_log.clear()
        self.bridge.configure(arms_of(self.profile))
        self.teleop.start(build_teleop_cmd(self.profile))
        return True

    def stop_all(self):
        self.sequence = None
        self.teleop.stop()
        self.driver.stop()
        self.statusBar().showMessage("Stopping what this window started...", 5000)

    def on_teleop_line(self, text):
        self.teleop_log.add(text)
        m = HOME_RE.search(text)
        if m:
            self.home_seen.add(m.group(1))

    # ----------------------------------------------------------------------------------------- start all
    def start_all(self):
        errs = validate(self.profile)
        if errs:
            self.tabs.setCurrentIndex(1)
            QtWidgets.QMessageBox.warning(self, "Settings", "Fix first:\n\n" + "\n".join(errs))
            return
        self.sequence = {"step": 0, "t0": time.monotonic(), "sent": False, "failed": None}
        for lab, text in zip(self.step_labels, self.STEPS):
            lab.setText("○ " + text)
            lab.setStyleSheet("")
        self.statusBar().showMessage("Start all: running...", 5000)

    def set_step(self, i, mark, color=""):
        self.step_labels[i].setText(f"{mark} {self.STEPS[i]}")
        self.step_labels[i].setStyleSheet(f"color:{color}" if color else "")

    def advance_sequence(self):
        q = self.sequence
        if not q or not self.snap or q["failed"] is not None:
            return
        s, step, waited = self.snap, q["step"], time.monotonic() - q["t0"]
        mode_isaac = self.profile["mode"] == "isaac"

        def done():
            self.set_step(step, "✔", GREEN)
            q.update(step=step + 1, t0=time.monotonic(), sent=False)

        def fail(msg):
            self.set_step(step, "✖", RED)
            q["failed"] = msg
            self.statusBar().showMessage("Start all stopped: " + msg, 15000)

        drv = s.get("driver", {})
        if step == 0:                                                   # driver process
            if drv.get("node"):
                done()
            else:
                if not q["sent"]:
                    self.start_driver()
                    q["sent"] = True
                elif waited > 25:
                    fail("the mocap driver did not appear (see Logs > Mocap driver)")
        elif step == 1:                                                 # activate
            if drv.get("state") == "active":
                done()
            elif drv.get("state") == "inactive" and (not q["sent"] or waited > 6):
                self.bridge.lifecycle(3)
                q["sent"], q["t0"] = True, time.monotonic()
            elif waited > 30:
                fail("the driver did not become active")
        elif step == 2:                                                 # frames
            if s.get("mocap_hz", 0) > 20:
                done()
            elif waited > 20:
                fail("driver is active but sends no frames: is Motive streaming?")
        elif step == 3:                                                 # robot side
            if not mode_isaac:
                done()
            elif s.get("clock_pub", 0) > 0 and s.get("states_pub", 0) > 0:
                done()
            elif waited > 180:
                fail("Isaac Sim is not ready (Play + bridge scripts, same ROS_DOMAIN_ID)")
            else:
                self.statusBar().showMessage("Waiting for Isaac Sim: start it, run the bridge scripts and press Play...")
        elif step == 4:                                                 # teleop
            if not q["sent"]:
                started = self.start_teleop() or any(n.startswith("rebel_teleop") for n in s.get("nodes", []))
                q["sent"] = True
                if not started:
                    fail("teleop could not be started")
                    return
            elif any(n.startswith("rebel_teleop") for n in s.get("nodes", [])):
                done()
            elif waited > 30:
                fail("teleop nodes did not appear (see Logs > Teleop)")
        elif step == 5:                                                 # at HOME
            need = {"rebel_teleop" + suffix for _, suffix, _ in arms_of(self.profile)}
            if need <= self.home_seen or (not self.teleop.running() and need <= set(s.get("nodes", []))):
                done()
                self.statusBar().showMessage("Ready. Hold both hands open and press 'Calibrate grippers', then SPACE to engage.", 20000)
            elif waited > 120:
                fail("arms did not reach HOME (check the teleop log)")
        elif step >= len(self.STEPS):
            self.sequence = None

    # ------------------------------------------------------------------------------------------- controls
    def arms(self):
        return arms_of(self.profile)

    def toggle(self, side):
        if time.monotonic() - self.last_toggle < 0.3:                  # key repeat of a held key
            return
        self.last_toggle = time.monotonic()
        engaged = self.snap.get("engaged", {})
        if side is None:
            targets = [suffix for _, suffix, _ in self.arms()]
            on = not any(engaged.get(suffix) for suffix in targets)
        else:
            targets = [suffix for sd, suffix, _ in self.arms() if sd == side]
            if not targets:
                self.statusBar().showMessage(f"No {side} arm in this profile", 4000)
                return
            on = not engaged.get(targets[0], False)
        for suffix in targets:
            self.bridge.engage(suffix, on)

    def home_all(self):
        for _, suffix, _ in self.arms():
            self.bridge.trigger("rebel_teleop" + suffix, "home")

    def calibrate(self):
        self.statusBar().showMessage("Hold both hands OPEN - calibrating in 3 s...", 3000)
        QtCore.QTimer.singleShot(3000, self._calibrate_now)

    def _calibrate_now(self):
        for _, suffix, _ in self.arms():
            self.bridge.trigger("rebel_gripper" + suffix, "calibrate")

    def clear_focus(self):
        w = QtWidgets.QApplication.focusWidget()
        if w is not None:
            w.clearFocus()

    def on_reply(self, text):
        self.statusBar().showMessage(text, 8000)
        self.teleop_log.add("[ui] " + text)

    # --------------------------------------------------------------------------------------------- status
    def set_light(self, key, color, text):
        label, title, value = self.lights[key]
        label.setText(dot(color) + " " + title)
        value.setText(text)

    def on_snapshot(self, s):
        self.snap = s
        drv = s["driver"]
        if not drv["node"]:
            self.set_light("driver", RED, "not running")
        else:
            state = drv["state"] or "?"
            self.set_light("driver", GREEN if state == "active" else AMBER, state)
        hz = s["mocap_hz"]
        if not s["have_mocap_msgs"]:
            self.set_light("mocap", RED, "mocap4r2_msgs missing")
        else:
            self.set_light("mocap", GREEN if hz > 20 else (AMBER if hz > 0 else RED), f"{hz:.0f} Hz")
        names = s["rb_names"]
        self.set_light("bodies", GREEN if names else GRAY, ", ".join(sorted(names, key=lambda n: (len(n), n))) or "none")
        if self.profile["mode"] == "isaac":
            ok = s["clock_pub"] > 0 and s["states_pub"] > 0
            self.set_light("robot", GREEN if ok else (AMBER if s["clock_pub"] > 0 else RED),
                           f"clock {s['clock_hz']:.0f} Hz, states {s['states_hz']:.0f} Hz" if ok else "Isaac not ready")
        else:
            n = s["cmd_sub"]
            self.set_light("robot", GREEN if n else AMBER, f"{n} listener(s) on the command topic" if n else "no listener yet")
        wanted = ["rebel_teleop" + suffix for _, suffix, _ in self.arms()] + ["rebel_gripper" + suffix for _, suffix, _ in self.arms()]
        present = [w for w in wanted if w in s["nodes"]]
        self.set_light("teleop", GREEN if len(present) == len(wanted) else (AMBER if present else GRAY),
                       f"{len(present)}/{len(wanted)} nodes")
        homes = {"rebel_teleop" + suffix for _, suffix, _ in self.arms()} & self.home_seen
        total = len(self.arms())
        self.set_light("home", GREEN if len(homes) == total else (AMBER if homes else GRAY),
                       f"{len(homes)}/{total} arms" if self.teleop.running() else "-")
        hands = s["hands"]
        ok = [side for side, v in hands.items() if v and s["mocap_hz"] > 5]      # a TF that is no longer updated is not tracking
        self.set_light("hands", GREEN if hands and len(ok) == len(hands) else (AMBER if ok else GRAY),
                       ", ".join(ok) if ok else "none")
        eng = s["engaged"]
        engaged_now = [suffix for _, suffix, _ in self.arms() if eng.get(suffix)]
        self.set_light("engaged", GREEN if engaged_now else (AMBER if present else GRAY),
                       "RUNNING" if engaged_now else ("paused" if eng else "-"))
        self.set_light("cmd", GREEN if s["cmd_hz"] > 20 else GRAY, f"{s['cmd_hz']:.0f} Hz, {s['cmd_sub']} listener(s)")
        for name, bar in self.joint_bars.items():
            bar.set_pos(s["joints"].get(name))
        self.toggle_btn.setStyleSheet("font-size:24px; font-weight:bold; " +
                                      (f"background:{GREEN}; color:white" if engaged_now else f"background:{AMBER}; color:black"))
        self.toggle_btn.setText("RUNNING - press to PAUSE  (SPACE)" if engaged_now else "PAUSED - press to RESUME  (SPACE)")
        hints = compute_hints(s, self.profile, bool(present))
        self.hints.setText("<br>".join("• " + h for h in hints) if hints else
                           f'<span style="color:{GREEN}">Nothing to fix.</span>')
        self.start_drv_btn.setEnabled(not self.driver.running())
        self.stop_drv_btn.setEnabled(self.driver.running())
        self.start_tel_btn.setEnabled(not self.teleop.running())
        self.stop_tel_btn.setEnabled(self.teleop.running())
        self.domain_label.setText(f"ROS_DOMAIN_ID = <b>{s['domain']}</b>")

    def closeEvent(self, event):
        if self.driver.running() or self.teleop.running():
            r = QtWidgets.QMessageBox.question(
                self, "Quit", "Stop the driver / teleop that this window started?",
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No | QtWidgets.QMessageBox.Cancel)
            if r == QtWidgets.QMessageBox.Cancel:
                event.ignore()
                return
            if r == QtWidgets.QMessageBox.Yes:
                self.teleop.stop()
                self.driver.stop()
                deadline = time.monotonic() + 8
                while (self.teleop.running() or self.driver.running()) and time.monotonic() < deadline:
                    QtWidgets.QApplication.processEvents()
                    time.sleep(0.05)
        self.bridge.stop()
        event.accept()


def main():
    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName("rebel_mocap_teleop")
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    ticker = QtCore.QTimer()                       # lets Python see Ctrl+C in the terminal that started the window
    ticker.start(500)
    ticker.timeout.connect(lambda: None)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
