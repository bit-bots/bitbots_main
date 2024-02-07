from python_qt_binding.QtCore import Qt, pyqtSignal
from python_qt_binding.QtWidgets import QListWidget


class DragDropList(QListWidget):
    """QListWidget with an event that is called when a drag and drop action was performed."""

    key_pressed = pyqtSignal()

    def __init__(self, parent, ui):
        super().__init__(parent)

        self.ui = ui
        self.setAcceptDrops(True)

    # fmt: off
    def dropEvent(self, e): # noqa: N802
        # fmt: on
        super().dropEvent(e)
        items = []
        for i in range(0, self.count()):
            items.append(self.item(i).text())
        self.ui.change_frame_order(items)

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
