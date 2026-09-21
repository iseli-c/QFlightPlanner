from qgis.core import QgsRasterLayer, QgsVectorLayer
from qgis.PyQt.QtCore import Qt, QThread, QObject
from qgis.PyQt.QtWidgets import QApplication

from ..error_reporting import QgsPrint, QgsMessBox
from .get_heights_worker import WorkerGetHeights


class TerrainSectionHandler(QObject):
    """Keeps terrain min/max heights in sync with the current inputs."""

    def __init__(self, dialog):
        super().__init__()
        self.dlg = dialog
        self.dtm: QgsRasterLayer = None
        self.aoi: QgsVectorLayer = None
        self.path_line: QgsVectorLayer = None
        self.gdal_ds = None
        self.thread = None
        self.worker = None
        self._pending = False
        self._previous_format = None
        self._previous_range = None

    def set_dtm(self, dtm_layer: QgsRasterLayer, gdal_dataset):
        """Set the DTM layer and its GDAL dataset"""
        self.dtm = dtm_layer
        self.gdal_ds = gdal_dataset

    def set_aoi(self, aoi_layer: QgsVectorLayer):
        """Set the AoI vector layer"""
        self.aoi = aoi_layer

    def set_corridor_line(self, line_layer: QgsVectorLayer):
        """Set the Corridor Line layer"""
        self.path_line = line_layer

    def refresh_heights(self, *args):
        """Recalculate terrain min/max heights for the current inputs."""
        if self.dtm is None:
            return
        if self.dlg.tabBlock:
            if self.aoi is None:
                return
        else:
            if self.path_line is None:
                return

        if self.thread is not None and self.thread.isRunning():
            self._pending = True
            return

        progress = self.dlg.progressBar
        self._previous_format = progress.format()
        self._previous_range = (progress.minimum(), progress.maximum())
        progress.setRange(0, 100)
        progress.setValue(0)
        progress.setFormat("Updating heights from DTM...")
        QApplication.setOverrideCursor(Qt.CursorShape.WaitCursor)

        worker = WorkerGetHeights(
            is_block=self.dlg.tabBlock,
            aoi_layer=self.aoi,
            path_line=self.path_line,
            buffer_value=self.dlg.doubleSpinBoxBuffer.value(),
            dtm_layer=self.dtm,
            gdal_ds=self.gdal_ds,
        )
        thread = QThread(self.dlg)
        worker.moveToThread(thread)

        worker.progress.connect(progress.setValue)
        worker.finished.connect(self._on_finished)
        worker.error.connect(self._on_error)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)
        thread.started.connect(worker.run)

        thread.start()
        self.thread = thread
        self.worker = worker

    def _on_finished(self, result):
        self._restore()
        if result is not None:
            h_min, h_max, min_buf = result
            self.dlg.doubleSpinBoxMinHeight.setValue(h_min)
            self.dlg.doubleSpinBoxMaxHeight.setValue(h_max)
            if min_buf is not None:
                buffer_box = self.dlg.doubleSpinBoxBuffer
                buffer_box.blockSignals(True)
                buffer_box.setMinimum(min_buf / 2)
                buffer_box.blockSignals(False)
        self._rerun_if_pending()

    def _on_error(self, exception, traceback_str):
        self._restore()
        QgsMessBox(title="Error", text=f"Failed to get heights from DTM:\n{exception}", level="Critical")
        QgsPrint(traceback_str, level="Critical")
        self._rerun_if_pending()

    def _rerun_if_pending(self):
        if self._pending:
            self._pending = False
            self.refresh_heights()

    def _restore(self):
        if self.thread is not None:
            self.thread.quit()
            self.thread.wait()
            self.thread = None
        self.worker = None
        QApplication.restoreOverrideCursor()
        if self._previous_format is not None:
            progress = self.dlg.progressBar
            progress.setFormat(self._previous_format)
            progress.setMinimum(self._previous_range[0])
            progress.setMaximum(self._previous_range[1])
            progress.setValue(0)
