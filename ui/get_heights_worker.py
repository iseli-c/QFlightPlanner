import traceback

from qgis.PyQt.QtCore import QObject, pyqtSignal

from .terrain_utils import clipped_raster_minmax, create_buffer_around_line


class WorkerGetHeights(QObject):
    """Worker that calculates terrain min/max heights for the AoI or corridor."""

    finished = pyqtSignal(object)
    error = pyqtSignal(Exception, str)
    progress = pyqtSignal(int)

    def __init__(self, **data):
        super().__init__()
        self.is_block = data.get('is_block')
        self.aoi_layer = data.get('aoi_layer')
        self.path_line = data.get('path_line')
        self.buffer_value = data.get('buffer_value')
        self.dtm_layer = data.get('dtm_layer')
        self.gdal_ds = data.get('gdal_ds')
        self.killed = False

    def run(self):
        try:
            self.progress.emit(10)

            min_buf = None
            if self.is_block:
                vector_for_stats = self.aoi_layer
            else:
                vector_for_stats, min_buf = create_buffer_around_line(
                    self.path_line, self.gdal_ds, self.dtm_layer, self.buffer_value)

            if self.killed:
                self.finished.emit(None)
                return

            self.progress.emit(50)
            h_min, h_max = clipped_raster_minmax(vector_for_stats, self.dtm_layer)
            self.progress.emit(100)
            self.finished.emit((h_min, h_max, min_buf))

        except Exception as e:
            self.error.emit(e, traceback.format_exc())
            self.progress.emit(0)
