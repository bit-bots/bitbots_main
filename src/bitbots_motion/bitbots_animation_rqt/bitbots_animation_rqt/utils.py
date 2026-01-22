from typing import Any, Callable

from PyQt5.QtCore import QObject, Qt, pyqtSignal
from PyQt5.QtWidgets import QListWidget, QWidget
from sensor_msgs.msg import JointState


class DragDropList(QListWidget):
    """QListWidget with an event that is called when a drag and drop action was performed."""

    key_pressed = pyqtSignal()

    def __init__(self, parent: QWidget, frame_order_callback: Callable[[list[str]], None]):
        super().__init__(parent)

        self.frame_order_callback = frame_order_callback
        self.setAcceptDrops(True)

    # fmt: off
    def dropEvent(self, e): # noqa: N802
        # fmt: on
        super().dropEvent(e)
        items = []
        for i in range(self.count()):
            items.append(self.item(i).text())  # type: ignore[union-attr]
        self.frame_order_callback(items)

    # fmt: off
    def keyPressEvent(self, event): # noqa: N802
        # fmt: on
        if event.key() == Qt.Key_Delete:  # type: ignore[attr-defined]
            super().keyPressEvent(event)
            self.key_pressed.emit()
        elif event.key() == Qt.Key_Up and self.currentRow() - 1 >= 0:  # type: ignore[attr-defined]
            self.setCurrentRow(self.currentRow() - 1)
        elif event.key() == Qt.Key_Down and self.currentRow() + 1 < self.count():  # type: ignore[attr-defined]
            self.setCurrentRow(self.currentRow() + 1)


class JointStateCommunicate(QObject):
    signal = pyqtSignal(JointState)


def flatten_dict(input_dict: dict, parent_key: str = "", sep: str = ".") -> dict:
    """Flatten a nested dictionary."""
    items: list[Any] = []
    for k, v in input_dict.items():
        new_key = f"{parent_key}{sep}{k}" if parent_key else k
        if isinstance(v, dict):
            items.extend(flatten_dict(v, new_key, sep=sep).items())
        else:
            items.append((new_key, v))
    return dict(items)


def flatten_dict_of_lists(input_dict: dict, sep: str = ".") -> dict:
    """Flatten a nested dictionary and convert lists to separate keys."""
    flat_dict = flatten_dict(input_dict, sep=sep)
    new_dict = {}
    for key, value in flat_dict.items():
        if isinstance(value, list):
            for i, list_value in enumerate(value):
                new_dict[f"{key}{sep}{i}"] = list_value
        else:
            new_dict[key] = value
    return new_dict
