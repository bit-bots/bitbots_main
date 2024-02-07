from python_qt_binding.QtCore import Qt, pyqtSignal
from python_qt_binding.QtWidgets import QListWidget

class DragDropList(QListWidget):
    ''' QListWidget with an event that is called when a drag and drop action was performed.'''
    keyPressed = pyqtSignal()

    def __init__(self, parent, ui):
        super(DragDropList, self).__init__(parent)

        self.ui = ui
        self.setAcceptDrops(True)


    def dropEvent(self, e):
        super(DragDropList, self).dropEvent(e)
        items = []
        for i in range(0, self.count()):
            items.append(self.item(i).text())
        self.ui.change_frame_order(items)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Delete:
            super(DragDropList, self).keyPressEvent(event)
            self.keyPressed.emit()
        elif event.key() == Qt.Key_Up and self.currentRow()-1 >= 0:
                self.setCurrentRow(self.currentRow()-1)
        elif event.key() == Qt.Key_Down and self.currentRow()+1 < self.count():
            self.setCurrentRow(self.currentRow()+1)
