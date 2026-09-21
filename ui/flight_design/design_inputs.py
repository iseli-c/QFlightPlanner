from dataclasses import dataclass
from math import sqrt

from qgis.core import (
    QgsCoordinateReferenceSystem,
    QgsCoordinateTransform,
    QgsGeometry,
    QgsProject,
)

from ...error_reporting import QgsPrint
from ..dtm_window import DtmWindow


def _flight_parameters(ui):
    """Mirror of flight_parameters.calculate_flight_parameters without UI side effects."""
    camera = ui.camera_handler.camera
    gsd = ui.doubleSpinBoxGSD.value() / 100
    len_along = camera.pixels_along_track * gsd
    len_across = camera.pixels_across_track * gsd

    p = ui.doubleSpinBoxOverlap.value() / 100
    q = ui.doubleSpinBoxSidelap.value() / 100

    return len_along * (1 - p), len_across * (1 - q), len_along, len_across


def _altitude_values(ui):
    """Mirror of altitude_calculation.calculate_altitude without UI side effects."""
    camera = ui.camera_handler.camera
    min_h = ui.doubleSpinBoxMinHeight.value()
    if ui.radioButtonGSD.isChecked():
        gsd = ui.doubleSpinBoxGSD.value() / 100
        altitude_AGL = gsd / camera.sensor_size * camera.focal_length
    else:
        altitude_AGL = ui.doubleSpinBoxAltAGL.value()
    return min_h + altitude_AGL, altitude_AGL


@dataclass
class DesignInputs:
    """Snapshot of the direction-independent inputs of a block flight design."""

    aoi_geom: QgsGeometry
    crs: QgsCoordinateReferenceSystem
    Bx: float
    By: float
    len_along: float
    len_across: float
    exceed: int
    multiple_base: int
    altitude_ASL: float
    altitude_AGL: float
    adaptive: bool
    dtm_layer: object
    p_spec: float
    q_spec: float

    @classmethod
    def from_ui(cls, ui):
        if getattr(ui, 'tabCorridor', False):
            raise ValueError("Live preview supports block flights only.")
        if getattr(ui, 'AreaOfInterest', None) is None:
            raise ValueError("An Area of Interest is required for the live preview.")
        if getattr(ui.camera_handler, 'camera', None) is None:
            raise ValueError("Camera parameters are required for the live preview.")

        target_crs = QgsCoordinateReferenceSystem(ui.epsg_code)
        feature = next(iter(ui.AreaOfInterest.getFeatures()), None)
        if feature is None:
            raise ValueError("The Area of Interest layer is empty.")

        geometry = feature.geometry()
        source_crs = ui.AreaOfInterest.crs()
        if source_crs != target_crs:
            transform = QgsCoordinateTransform(source_crs, target_crs, QgsProject.instance())
            geometry = QgsGeometry(geometry)
            geometry.transform(transform)

        Bx, By, len_along, len_across = _flight_parameters(ui)
        altitude_ASL, altitude_AGL = _altitude_values(ui)

        return cls(
            aoi_geom=geometry,
            crs=target_crs,
            Bx=Bx,
            By=By,
            len_along=len_along,
            len_across=len_across,
            exceed=ui.spinBoxExceedExtremeStrips.value(),
            multiple_base=ui.spinBoxMultipleBase.value(),
            altitude_ASL=altitude_ASL,
            altitude_AGL=altitude_AGL,
            adaptive=(ui.checkBoxIncreaseOverlap.isChecked()
                      and getattr(ui, 'checkBoxPreviewAdaptive', None) is not None
                      and ui.checkBoxPreviewAdaptive.isChecked()),
            dtm_layer=getattr(ui, 'DTM', None),
            p_spec=ui.doubleSpinBoxOverlap.value() / 100,
            q_spec=ui.doubleSpinBoxSidelap.value() / 100,
        )

    def _adaptive_plan(self, angle):
        from .altitudes_utils.adaptive_layout import plan_adaptive_layout
        from ...mathgeo_utils.algebra import bounding_box_at_angle

        a, b, a2, b2, Dx, Dy = bounding_box_at_angle(angle, self.aoi_geom)
        margin = sqrt(self.len_along ** 2 + self.len_across ** 2) * (2 + self.exceed)
        bbox = self.aoi_geom.boundingBox()
        bbox.grow(margin)
        window = DtmWindow.from_layer(self.dtm_layer, bbox, bbox_crs=self.crs)
        return plan_adaptive_layout(
            window, self.aoi_geom, self.crs, angle, a, b, a2, b2, Dx, Dy,
            self.len_along, self.len_across, self.p_spec, self.q_spec,
            self.altitude_ASL, self.altitude_AGL, self.exceed, self.multiple_base
        )

    def compute(self, direction):
        from .altitudes_utils.process_modes import compute_design_geometry, direction_to_angle

        angle = direction_to_angle(direction)
        plan = None
        if self.adaptive and self.dtm_layer is not None:
            try:
                plan = self._adaptive_plan(angle)
            except Exception as e:
                QgsPrint(
                    f"Adaptive preview unavailable, using nominal layout: {e}",
                    level="Warning")
                plan = None

        return compute_design_geometry(
            angle,
            self.aoi_geom,
            self.crs,
            self.Bx,
            self.By,
            self.len_along,
            self.len_across,
            self.exceed,
            self.multiple_base,
            self.altitude_ASL,
            adaptive_plan=plan,
        )
