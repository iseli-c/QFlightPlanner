from math import atan2, degrees

from qgis.PyQt.QtCore import Qt, pyqtSignal
from qgis.PyQt.QtGui import QColor
from qgis.gui import QgsMapTool, QgsRubberBand
from qgis.core import (
    QgsCoordinateReferenceSystem,
    QgsCoordinateTransform,
    QgsProject,
    QgsWkbTypes,
)

from ..error_reporting import QgsTraceback


class DirectionSectionHandler:
    """Handler for synchronizing a dial and spin box representing direction in degrees."""
    def __init__(self, dial, spinBox):
        self.dial = dial
        self.spinBox = spinBox
        self.dial.valueChanged.connect(self.on_dial_valueChanged)
        self.spinBox.valueChanged.connect(self.on_spinBoxDirection_valueChanged)

    def on_dial_valueChanged(self):
        """Handle changes in the dial and update the spin box"""
        v = self.dial.value()
        self.spinBox.setValue(v - 180 if v > 180 else v + 180)

    def on_spinBoxDirection_valueChanged(self):
        """Handle changes in the spin box and update the dial"""
        v = self.spinBox.value()
        self.dial.setValue(v - 180 if v > 180 else v + 180)


class DirectionMapTool(QgsMapTool):
    """Map tool that sets the flight direction by clicking two points on the canvas."""

    directionPicked = pyqtSignal(object)
    directionDragging = pyqtSignal(int)

    def __init__(self, canvas, target_crs, previous_tool=None):
        super().__init__(canvas)
        self.canvas = canvas
        self.target_crs = QgsCoordinateReferenceSystem(target_crs)
        self.previous_tool = previous_tool
        self.start_point = None
        self.rubber_band = QgsRubberBand(canvas, QgsWkbTypes.LineGeometry)
        self.rubber_band.setColor(QColor(255, 0, 0, 180))
        self.rubber_band.setWidth(2)
        self.setCursor(Qt.CursorShape.CrossCursor)

    def canvasPressEvent(self, event):
        """Collect the two points defining the flight direction."""
        if event.button() != Qt.MouseButton.LeftButton:
            return

        point = self.toMapCoordinates(event.pos())
        if self.start_point is None:
            self.start_point = point
            self.rubber_band.reset(QgsWkbTypes.LineGeometry)
            self.rubber_band.addPoint(point, False)
            self.rubber_band.addPoint(point, False)
            self.rubber_band.show()
        else:
            self.finish(point)

    def canvasMoveEvent(self, event):
        """Update the preview line while moving the mouse."""
        if self.start_point is None:
            return
        end_point = self.toMapCoordinates(event.pos())
        self.rubber_band.movePoint(1, end_point)
        try:
            self.directionDragging.emit(self._calculate_direction(self.start_point, end_point))
        except Exception:
            pass

    def keyPressEvent(self, event):
        """Cancel drawing with the Escape key."""
        if event.key() == Qt.Key.Key_Escape:
            self.cancel()
            return
        super().keyPressEvent(event)

    def finish(self, end_point):
        """Emit the azimuth of the drawn segment and release the tool."""
        try:
            direction = self._calculate_direction(self.start_point, end_point)
        except Exception:
            QgsTraceback()
            direction = None

        self._release_tool()
        if direction is not None:
            self.directionPicked.emit(direction)

    def cancel(self):
        """Cancel the drawing operation."""
        self._release_tool()

    def deactivate(self):
        """Reset the tool state when it is deactivated."""
        self._reset()
        super().deactivate()

    def _release_tool(self):
        """Restore the previously active map tool."""
        if self.previous_tool is not None:
            self.canvas.setMapTool(self.previous_tool)
        else:
            self.canvas.unsetMapTool(self)

    def _calculate_direction(self, start_point, end_point):
        """Return the compass azimuth (0-359 degrees) of the drawn segment."""
        source_crs = self.canvas.mapSettings().destinationCrs()
        if source_crs != self.target_crs:
            transform = QgsCoordinateTransform(
                source_crs, self.target_crs, QgsProject.instance())
            start_point = transform.transform(start_point)
            end_point = transform.transform(end_point)

        dx = end_point.x() - start_point.x()
        dy = end_point.y() - start_point.y()
        azimuth = degrees(atan2(dx, dy)) % 360
        return int(round(azimuth)) % 360

    def _reset(self):
        """Clear the start point and the preview line."""
        self.start_point = None
        self.rubber_band.reset(QgsWkbTypes.LineGeometry)
