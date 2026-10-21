#!/usr/bin/env python3

import sys
import threading

import rclpy
from geometry_msgs.msg import Twist, Vector3
from livelybot_msg.msg import PowerSwitch
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QKeyEvent
from PyQt5.QtWidgets import (
    QApplication,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QVBoxLayout,
    QWidget,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty

from bitbots_msgs.action import PlayAnimation
from bitbots_msgs.msg import HeadMode, JointCommand
from bitbots_msgs.srv import MoveBall, SimulatorPush

# Movement vectors: (vx, vy, vth)
MOVE_BINDINGS = {
    Qt.Key_W: (1.0, 0.0, 0.0),
    Qt.Key_S: (-1.0, 0.0, 0.0),
    Qt.Key_A: (0.0, 1.0, 0.0),
    Qt.Key_D: (0.0, -1.0, 0.0),
    Qt.Key_Q: (0.0, 0.0, 1.0),
    Qt.Key_E: (0.0, 0.0, -1.0),
}

# Head delta vectors: (dpitch, dyaw)
HEAD_BINDINGS = {
    Qt.Key_U: (1, 1),
    Qt.Key_I: (1, 0),
    Qt.Key_O: (1, -1),
    Qt.Key_J: (0, 1),
    Qt.Key_L: (0, -1),
    Qt.Key_M: (-1, 1),
    Qt.Key_Comma: (-1, 0),
    Qt.Key_Period: (-1, -1),
}

# Ball movement vectors: (dx, dy)
BALL_MOVE_BINDINGS = {
    Qt.Key_T: (1.0, 0.0),
    Qt.Key_B: (-1.0, 0.0),
    Qt.Key_H: (0.0, 1.0),
    Qt.Key_N: (0.0, -1.0),
}

HEAD_MODE_NAMES = {
    HeadMode.TRACK_BALL: "0: Track Ball",
    HeadMode.SEARCH_FIELD_FEATURES: "1: Search Field Features",
    HeadMode.LOOK_FORWARD: "2: Look Forward",
    HeadMode.DONT_MOVE: "3: Don't Move",
    HeadMode.SEARCH_BALL_PENALTY: "4: Penalty Ball Mode",
    HeadMode.SEARCH_FRONT: "5: Search Front",
}


class TeleopNode(Node):
    """ROS 2 Node handling publishers, action clients, and service clients for teleop."""

    def __init__(self):
        super().__init__("teleop_gui")

        self.declare_parameter("speed_x", 0.5)
        self.declare_parameter("speed_y", 0.2)
        self.declare_parameter("speed_turn", 0.5)

        self.speed_x = self.get_parameter("speed_x").get_parameter_value().double_value
        self.speed_y = self.get_parameter("speed_y").get_parameter_value().double_value
        self.speed_turn = self.get_parameter("speed_turn").get_parameter_value().double_value

        # Walking Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 1)

        # Head Publishers & Subscribers
        self.head_mode_pub = self.create_publisher(HeadMode, "head_mode", 1)
        if self.get_parameter("use_sim_time").get_parameter_value().bool_value:
            self.head_pub = self.create_publisher(JointCommand, "head_motor_goals", 1)
        else:
            self.head_pub = self.create_publisher(JointCommand, "joint_command", 1)

        self.joint_state_sub = self.create_subscription(JointState, "joint_states", self.joint_state_cb, 1)

        # Power switch & simulation clients
        self.power_switch_pub = self.create_publisher(PowerSwitch, "/power_switch_control", 10)
        self.reset_robot_client = self.create_client(Empty, "/reset_pose")
        self.reset_ball_client = self.create_client(Empty, "/reset_ball")
        self.move_ball_client = self.create_client(MoveBall, "/move_ball")
        self.simulator_push_client = self.create_client(SimulatorPush, "/simulator_push")

        # Animation Action Client
        self.animation_client = ActionClient(self, PlayAnimation, "animation")

        # Head state tracking
        self.head_yaw_pos = 0.0
        self.head_pitch_pos = 0.0
        self.head_msg = JointCommand()
        self.head_msg.max_torques = [-1.0] * 2
        self.head_msg.velocities = [5.0] * 2
        self.head_msg.accelerations = [40.0] * 2
        self.head_msg.joint_names = ["head_yaw_joint", "head_pitch_joint"]
        self.head_msg.positions = [0.0] * 2

        self.head_mode_msg = HeadMode(head_mode=HeadMode.DONT_MOVE)
        self.head_step = 0.05

        # Motion state
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        self.a_x = -1.0  # -1.0 = full stop / walk stopped, 0.0 = walk enabled

        # Simulation push forces
        self.push_force_x = 0.0
        self.push_force_y = 0.0

    def joint_state_cb(self, msg: JointState):
        if "head_yaw_joint" in msg.name and "head_pitch_joint" in msg.name:
            self.head_yaw_pos = msg.position[msg.name.index("head_yaw_joint")]
            self.head_pitch_pos = msg.position[msg.name.index("head_pitch_joint")]

    def publish_twist(self):
        twist = Twist()
        twist.linear = Vector3(x=float(self.vx), y=float(self.vy), z=0.0)
        twist.angular = Vector3(x=float(self.a_x), y=0.0, z=float(self.vth))
        self.cmd_vel_pub.publish(twist)

    def publish_head_mode(self):
        self.head_mode_pub.publish(self.head_mode_msg)

    def move_head(self, dpitch: float, dyaw: float):
        self.head_msg.positions[0] = self.head_yaw_pos + dyaw * self.head_step
        self.head_msg.positions[1] = self.head_pitch_pos + dpitch * self.head_step
        self.head_pub.publish(self.head_msg)

    def zero_head(self):
        self.head_msg.positions[0] = 0.0
        self.head_msg.positions[1] = 0.0
        self.head_pub.publish(self.head_msg)

    def set_head_mode(self, mode: int):
        self.head_mode_msg.head_mode = mode
        self.publish_head_mode()

    def play_walkready(self):
        self.animation_client.send_goal_async(PlayAnimation.Goal(animation="walkready"))

    def emergency_stop(self):
        self.power_switch_pub.publish(PowerSwitch(control_switch=1, power_switch=0))
        self.full_stop()

    def full_stop(self):
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        self.a_x = -1.0
        self.publish_twist()

    def walk_in_place(self):
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        self.a_x = 0.0
        self.publish_twist()

    def reset_robot(self):
        try:
            self.reset_robot_client.call_async(Empty.Request())
        except Exception:
            pass

    def reset_ball(self):
        try:
            self.reset_ball_client.call_async(Empty.Request())
        except Exception:
            pass

    def move_ball(self, dx: float, dy: float):
        try:
            req = MoveBall.Request()
            req.offset.x = float(dx)
            req.offset.y = float(dy)
            self.move_ball_client.call_async(req)
        except Exception:
            pass

    def push_robot(self):
        try:
            req = SimulatorPush.Request()
            req.force.x = float(self.push_force_x)
            req.force.y = float(self.push_force_y)
            req.relative = True
            self.simulator_push_client.call_async(req)
        except Exception:
            pass


class TeleopGUIWindow(QMainWindow):
    """Minimal GUI window capturing WASD and action keys for robot teleoperation."""

    def __init__(self, node: TeleopNode):
        super().__init__()
        self.node = node
        self.pressed_keys: set[int] = set()
        self.shift_pressed = False
        self.last_action_text = "Ready (Click to focus, use WASD to walk)"

        self.init_ui()

        # Timer for publishing ROS 2 commands and updating GUI (20 Hz)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(50)

    def init_ui(self):
        self.setWindowTitle("Bit-Bots Teleop GUI")
        self.setMinimumSize(540, 560)
        self.setFocusPolicy(Qt.StrongFocus)

        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(12, 12, 12, 12)

        # 1. Header Focus Status Banner
        self.focus_label = QLabel("🎮 TELEOP ACTIVE — Hold WASD to Move", self)
        self.focus_label.setAlignment(Qt.AlignCenter)
        self.focus_label.setStyleSheet(
            "QLabel { background-color: #2e7d32; color: white; font-weight: bold; "
            "font-size: 14px; padding: 8px; border-radius: 4px; }"
        )
        main_layout.addWidget(self.focus_label)

        # 2. Live State Dashboard
        state_group = QGroupBox("Motion & Robot State", self)
        state_layout = QGridLayout(state_group)
        state_layout.setContentsMargins(10, 10, 10, 10)
        state_layout.setSpacing(8)

        self.lbl_walk_status = QLabel("STOPPED", self)
        self.lbl_walk_status.setStyleSheet("font-weight: bold; font-size: 13px; color: #c62828;")
        self.lbl_vel_x = QLabel("0.00 m/s", self)
        self.lbl_vel_y = QLabel("0.00 m/s", self)
        self.lbl_vel_th = QLabel("0.00 rad/s", self)
        self.lbl_keys_held = QLabel("None", self)
        self.lbl_keys_held.setStyleSheet("font-weight: bold; color: #1565c0;")

        self.lbl_head_pos = QLabel("Yaw: 0.00 rad | Pitch: 0.00 rad", self)
        self.lbl_head_mode = QLabel(HEAD_MODE_NAMES[HeadMode.DONT_MOVE], self)
        self.lbl_push_force = QLabel("Fx: 0.0 N | Fy: 0.0 N", self)

        state_layout.addWidget(QLabel("Motion Mode:"), 0, 0)
        state_layout.addWidget(self.lbl_walk_status, 0, 1)
        state_layout.addWidget(QLabel("Active Keys:"), 0, 2)
        state_layout.addWidget(self.lbl_keys_held, 0, 3)

        state_layout.addWidget(QLabel("Linear X (Fwd):"), 1, 0)
        state_layout.addWidget(self.lbl_vel_x, 1, 1)
        state_layout.addWidget(QLabel("Linear Y (Left):"), 1, 2)
        state_layout.addWidget(self.lbl_vel_y, 1, 3)

        state_layout.addWidget(QLabel("Angular Z (Turn):"), 2, 0)
        state_layout.addWidget(self.lbl_vel_th, 2, 1)
        state_layout.addWidget(QLabel("Head Mode:"), 2, 2)
        state_layout.addWidget(self.lbl_head_mode, 2, 3)

        state_layout.addWidget(QLabel("Head Position:"), 3, 0)
        state_layout.addWidget(self.lbl_head_pos, 3, 1)
        state_layout.addWidget(QLabel("Push Force:"), 3, 2)
        state_layout.addWidget(self.lbl_push_force, 3, 3)

        main_layout.addWidget(state_group)

        # 3. Quick Action Buttons
        btn_layout = QHBoxLayout()
        btn_stop = QPushButton("Full Stop [F]", self)
        btn_stop.clicked.connect(self.trigger_full_stop)
        btn_layout.addWidget(btn_stop)

        btn_inplace = QPushButton("Walk in Place [Space]", self)
        btn_inplace.clicked.connect(self.trigger_walk_in_place)
        btn_layout.addWidget(btn_inplace)

        btn_walkready = QPushButton("Walkready [Shift+F]", self)
        btn_walkready.clicked.connect(self.trigger_walkready)
        btn_layout.addWidget(btn_walkready)

        btn_head_zero = QPushButton("Center Head [K]", self)
        btn_head_zero.clicked.connect(self.trigger_zero_head)
        btn_layout.addWidget(btn_head_zero)

        btn_estop = QPushButton("EMERGENCY STOP [ESC]", self)
        btn_estop.setStyleSheet("background-color: #d32f2f; color: white; font-weight: bold;")
        btn_estop.clicked.connect(self.trigger_emergency_stop)
        btn_layout.addWidget(btn_estop)

        main_layout.addLayout(btn_layout)

        # 4. Keybinding Reference / Guide
        guide_group = QGroupBox("Keybind Controls Reference", self)
        guide_layout = QGridLayout(guide_group)
        guide_layout.setContentsMargins(10, 8, 10, 8)
        guide_layout.setSpacing(6)

        ref_walking = (
            "<b>Walking (Hold to Move):</b><br>"
            "<b>W / S:</b> Forward / Backward<br>"
            "<b>A / D:</b> Left / Right<br>"
            "<b>Q / E:</b> Turn Left / Right<br>"
            "<i>(Hold multiple for diagonal, Shift for 2x speed)</i>"
        )
        ref_head = (
            "<b>Head Control & Modes:</b><br>"
            "<b>U I O / J K L / M , .:</b> Move Head (K = Center)<br>"
            "<b>0:</b> Ball Track &nbsp;|&nbsp; <b>1:</b> Field Search<br>"
            "<b>2:</b> Look Fwd &nbsp;|&nbsp; <b>3:</b> Don't Move<br>"
            "<b>4:</b> Penalty &nbsp;|&nbsp; <b>5:</b> Front Search"
        )
        ref_actions = (
            "<b>Quick Actions & Simulation:</b><br>"
            "<b>F:</b> Full Stop &nbsp;|&nbsp; <b>Space:</b> Walk In Place<br>"
            "<b>Shift+F:</b> Walkready Animation<br>"
            "<b>ESC:</b> Emergency Power Stop<br>"
            "<b>R / Shift+R:</b> Reset Robot / Ball<br>"
            "<b>T B H N:</b> Move Ball &nbsp;|&nbsp; <b>P:</b> Push"
        )

        lbl_walk_ref = QLabel(ref_walking, self)
        lbl_head_ref = QLabel(ref_head, self)
        lbl_act_ref = QLabel(ref_actions, self)

        guide_layout.addWidget(lbl_walk_ref, 0, 0)
        guide_layout.addWidget(lbl_head_ref, 0, 1)
        guide_layout.addWidget(lbl_act_ref, 0, 2)

        main_layout.addWidget(guide_group)

        # 5. Last Action Status Label
        self.status_bar_label = QLabel(self.last_action_text, self)
        self.status_bar_label.setStyleSheet("color: #555; font-style: italic; padding: 4px;")
        main_layout.addWidget(self.status_bar_label)

    def trigger_full_stop(self):
        self.pressed_keys.clear()
        self.node.full_stop()
        self.last_action_text = "Full Stop executed."
        self.setFocus()

    def trigger_walk_in_place(self):
        self.pressed_keys.clear()
        self.node.walk_in_place()
        self.last_action_text = "Walking in place."
        self.setFocus()

    def trigger_walkready(self):
        self.node.play_walkready()
        self.last_action_text = "Playing walkready animation."
        self.setFocus()

    def trigger_zero_head(self):
        self.node.zero_head()
        self.last_action_text = "Head centered."
        self.setFocus()

    def trigger_emergency_stop(self):
        self.pressed_keys.clear()
        self.node.emergency_stop()
        self.last_action_text = "EMERGENCY STOP executed (servo power off)."
        self.setFocus()

    def _update_walk_velocities(self):
        """Computes current movement velocities based on actively held movement keys."""
        if self.pressed_keys:
            vx_sum = 0.0
            vy_sum = 0.0
            vth_sum = 0.0
            for k in self.pressed_keys:
                if k in MOVE_BINDINGS:
                    vx, vy, vth = MOVE_BINDINGS[k]
                    vx_sum += vx
                    vy_sum += vy
                    vth_sum += vth

            speed_factor = 2.0 if self.shift_pressed else 1.0
            self.node.vx = round(vx_sum * self.node.speed_x * speed_factor, 2)
            self.node.vy = round(vy_sum * self.node.speed_y * speed_factor, 2)
            self.node.vth = round(vth_sum * self.node.speed_turn * speed_factor, 2)
            self.node.a_x = 0.0
        else:
            self.node.vx = 0.0
            self.node.vy = 0.0
            self.node.vth = 0.0
            self.node.a_x = -1.0

    def keyPressEvent(self, event: QKeyEvent):  # noqa: N802
        key = event.key()
        self.shift_pressed = bool(event.modifiers() & Qt.ShiftModifier)

        # Ignore auto-repeats for one-shot actions, but update movement
        if key in MOVE_BINDINGS:
            self.pressed_keys.add(key)
            self._update_walk_velocities()
            self.last_action_text = f"Walking ({self._get_key_names()})"
            event.accept()
            return

        if event.isAutoRepeat():
            event.accept()
            return

        # Special Action Keys
        if key == Qt.Key_Space:
            self.pressed_keys.clear()
            self.node.walk_in_place()
            self.last_action_text = "Walking in place"
        elif key == Qt.Key_F:
            if self.shift_pressed:
                self.node.play_walkready()
                self.last_action_text = "Walkready animation triggered"
            else:
                self.pressed_keys.clear()
                self.node.full_stop()
                self.last_action_text = "Full stop"
        elif key == Qt.Key_G:
            self.pressed_keys.clear()
            self.node.walk_in_place()
            self.last_action_text = "Walking on spot / speeds cleared"
        elif key == Qt.Key_Escape:
            self.pressed_keys.clear()
            self.node.emergency_stop()
            self.last_action_text = "EMERGENCY STOP (Power Off)"
        elif key == Qt.Key_K:
            self.node.zero_head()
            self.last_action_text = "Head centered"
        elif key in HEAD_BINDINGS:
            step_mult = 10.0 if self.shift_pressed else 1.0
            dpitch, dyaw = HEAD_BINDINGS[key]
            self.node.move_head(dpitch * step_mult, dyaw * step_mult)
            self.last_action_text = f"Head moved: pitch={dpitch * step_mult:+}, yaw={dyaw * step_mult:+}"
        elif key in (Qt.Key_0, Qt.Key_1, Qt.Key_2, Qt.Key_3, Qt.Key_4, Qt.Key_5):
            mode = key - Qt.Key_0
            self.node.set_head_mode(mode)
            self.last_action_text = f"Head mode set to {HEAD_MODE_NAMES.get(mode, mode)}"
        elif key == Qt.Key_R:
            if self.shift_pressed:
                self.node.reset_ball()
                self.last_action_text = "Simulation: Ball reset"
            else:
                self.node.reset_robot()
                self.last_action_text = "Simulation: Robot reset"
        elif key in BALL_MOVE_BINDINGS:
            dx, dy = BALL_MOVE_BINDINGS[key]
            self.node.move_ball(dx, dy)
            self.last_action_text = f"Simulation: Moved ball ({dx:+}, {dy:+})"
        elif key == Qt.Key_P:
            if self.shift_pressed:
                self.node.push_force_x = 0.0
                self.node.push_force_y = 0.0
                self.last_action_text = "Simulation: Reset push forces"
            else:
                self.node.push_robot()
                self.last_action_text = (
                    f"Simulation: Applied push ({self.node.push_force_x} N, {self.node.push_force_y} N)"
                )
        elif event.text() in ("ü", "Ü"):
            self.node.push_force_x += 10.0 if self.shift_pressed else 1.0
            self.last_action_text = f"Simulation: Push force X = {self.node.push_force_x} N"
        elif event.text() in ("ä", "Ä"):
            self.node.push_force_x -= 10.0 if self.shift_pressed else 1.0
            self.last_action_text = f"Simulation: Push force X = {self.node.push_force_x} N"
        elif event.text() in ("+", "*"):
            self.node.push_force_y += 10.0 if self.shift_pressed else 1.0
            self.last_action_text = f"Simulation: Push force Y = {self.node.push_force_y} N"
        elif event.text() in ("#", "'"):
            self.node.push_force_y -= 10.0 if self.shift_pressed else 1.0
            self.last_action_text = f"Simulation: Push force Y = {self.node.push_force_y} N"
        else:
            self.last_action_text = f"Unhandled key: {event.text() or key}"

        event.accept()

    def keyReleaseEvent(self, event: QKeyEvent):  # noqa: N802
        if event.isAutoRepeat():
            event.accept()
            return

        key = event.key()
        self.shift_pressed = bool(event.modifiers() & Qt.ShiftModifier)

        if key in MOVE_BINDINGS:
            self.pressed_keys.discard(key)
            self._update_walk_velocities()
            self.last_action_text = (
                f"Walking ({self._get_key_names()})" if self.pressed_keys else "Key released (Stopped)"
            )

        event.accept()

    def _get_key_names(self) -> str:
        names = []
        name_map = {
            Qt.Key_W: "W",
            Qt.Key_S: "S",
            Qt.Key_A: "A",
            Qt.Key_D: "D",
            Qt.Key_Q: "Q",
            Qt.Key_E: "E",
        }
        for k in self.pressed_keys:
            if k in name_map:
                names.append(name_map[k])
        return ", ".join(names) if names else "None"

    def tick(self):
        """Periodic loop: publish ROS 2 commands and update UI displays."""
        # 1. Publish commands
        self.node.publish_twist()
        self.node.publish_head_mode()

        # 2. Update UI
        if self.node.a_x == -1.0:
            self.lbl_walk_status.setText("STOPPED")
            self.lbl_walk_status.setStyleSheet("font-weight: bold; font-size: 13px; color: #c62828;")
        elif self.node.vx == 0.0 and self.node.vy == 0.0 and self.node.vth == 0.0:
            self.lbl_walk_status.setText("WALKING IN PLACE")
            self.lbl_walk_status.setStyleSheet("font-weight: bold; font-size: 13px; color: #f57f17;")
        else:
            self.lbl_walk_status.setText("WALKING")
            self.lbl_walk_status.setStyleSheet("font-weight: bold; font-size: 13px; color: #2e7d32;")

        self.lbl_vel_x.setText(f"{self.node.vx:+.2f} m/s")
        self.lbl_vel_y.setText(f"{self.node.vy:+.2f} m/s")
        self.lbl_vel_th.setText(f"{self.node.vth:+.2f} rad/s")
        self.lbl_keys_held.setText(self._get_key_names())

        self.lbl_head_pos.setText(f"Yaw: {self.node.head_yaw_pos:+.2f} | Pitch: {self.node.head_pitch_pos:+.2f}")
        self.lbl_head_mode.setText(HEAD_MODE_NAMES.get(self.node.head_mode_msg.head_mode, "Unknown"))
        self.lbl_push_force.setText(f"Fx: {self.node.push_force_x:+.1f} N | Fy: {self.node.push_force_y:+.1f} N")

        self.status_bar_label.setText(self.last_action_text)

    def closeEvent(self, event):  # noqa: N802
        self.node.full_stop()
        self.timer.stop()
        event.accept()


def main():
    rclpy.init(args=None)
    node = TeleopNode()

    # Spin ROS node in background daemon thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    app = QApplication.instance() or QApplication(sys.argv)
    window = TeleopGUIWindow(node)
    window.show()

    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        pass
    finally:
        node.full_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
