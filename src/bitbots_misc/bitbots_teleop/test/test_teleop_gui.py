import sys

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QKeyEvent
from PyQt5.QtWidgets import QApplication
import pytest
import rclpy

from bitbots_teleop.teleop_gui import TeleopGUIWindow, TeleopNode


@pytest.fixture(scope="module")
def rclpy_init():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture(scope="module")
def qapp():
    return QApplication.instance() or QApplication(sys.argv)


def test_teleop_gui_wasd_controls(rclpy_init, qapp):
    node = TeleopNode()
    win = TeleopGUIWindow(node)

    # Initially stopped
    assert node.vx == 0.0
    assert node.vy == 0.0
    assert node.vth == 0.0
    assert node.a_x == -1.0

    # 1. Press W (Forward)
    win.keyPressEvent(QKeyEvent(QKeyEvent.KeyPress, Qt.Key_W, Qt.NoModifier))
    assert Qt.Key_W in win.pressed_keys
    assert node.vx == 0.5
    assert node.vy == 0.0
    assert node.vth == 0.0
    assert node.a_x == 0.0

    # 2. Press D simultaneously (Forward + Right)
    win.keyPressEvent(QKeyEvent(QKeyEvent.KeyPress, Qt.Key_D, Qt.NoModifier))
    assert Qt.Key_W in win.pressed_keys
    assert Qt.Key_D in win.pressed_keys
    assert node.vx == 0.5
    assert node.vy == -0.2
    assert node.a_x == 0.0

    # 3. Release W, keep D
    win.keyReleaseEvent(QKeyEvent(QKeyEvent.KeyRelease, Qt.Key_W, Qt.NoModifier))
    assert Qt.Key_W not in win.pressed_keys
    assert Qt.Key_D in win.pressed_keys
    assert node.vx == 0.0
    assert node.vy == -0.2
    assert node.a_x == 0.0

    # 4. Release D -> Complete stop
    win.keyReleaseEvent(QKeyEvent(QKeyEvent.KeyRelease, Qt.Key_D, Qt.NoModifier))
    assert len(win.pressed_keys) == 0
    assert node.vx == 0.0
    assert node.vy == 0.0
    assert node.a_x == -1.0

    # 5. Shift modifier increases speed (2x)
    win.keyPressEvent(QKeyEvent(QKeyEvent.KeyPress, Qt.Key_W, Qt.ShiftModifier))
    assert node.vx == 1.0
    win.keyReleaseEvent(QKeyEvent(QKeyEvent.KeyRelease, Qt.Key_W, Qt.ShiftModifier))
    assert node.vx == 0.0

    # 6. Space (Walk in place)
    win.keyPressEvent(QKeyEvent(QKeyEvent.KeyPress, Qt.Key_Space, Qt.NoModifier))
    assert node.vx == 0.0
    assert node.a_x == 0.0

    # 7. Full stop (F)
    win.keyPressEvent(QKeyEvent(QKeyEvent.KeyPress, Qt.Key_F, Qt.NoModifier))
    assert node.a_x == -1.0

    # 8. Head Mode selection
    win.keyPressEvent(QKeyEvent(QKeyEvent.KeyPress, Qt.Key_1, Qt.NoModifier))
    assert node.head_mode_msg.head_mode == 1

    # Clean up
    win.close()
    node.destroy_node()
