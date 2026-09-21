import traceback

from qgis.PyQt.QtCore import QObject, pyqtSignal


class WorkerPreview(QObject):
    """Recomputes a lightweight flight geometry preview for a given direction."""

    finished = pyqtSignal(object, object)
    error = pyqtSignal(Exception, str)

    def __init__(self, inputs, direction):
        super().__init__()
        self.inputs = inputs
        self.direction = direction
        self.killed = False

    def run(self):
        try:
            pc_layer, photo_layer = self.inputs.compute(self.direction)
            if self.killed:
                return
            self.finished.emit(pc_layer, photo_layer)
        except Exception as e:
            if not self.killed:
                self.error.emit(e, traceback.format_exc())
