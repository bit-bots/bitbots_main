from typing import Callable

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import QListWidget, QWidget


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
        for i in range(0, self.count()):
            items.append(self.item(i).text())
        self.frame_order_callback(items)

    # fmt: off
    def keyPressEvent(self, event): # noqa: N802
        # fmt: on
        if event.key() == Qt.Key_Delete:
            super().keyPressEvent(event)
            self.key_pressed.emit()
        elif event.key() == Qt.Key_Up and self.currentRow() - 1 >= 0:
            self.setCurrentRow(self.currentRow() - 1)
        elif event.key() == Qt.Key_Down and self.currentRow() + 1 < self.count():
            self.setCurrentRow(self.currentRow() + 1)
