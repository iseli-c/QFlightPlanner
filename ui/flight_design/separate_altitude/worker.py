import os
from ....error_reporting import QgsPrint
import numpy as np
from qgis.PyQt.QtCore import QObject, pyqtSignal
from qgis.core import (
    QgsFeature,
    QgsGeometry,
    QgsPointXY,
)

from ...dtm_window import DtmWindow

from ....geoprocessing_utils import create_flight_line, create_waypoints, change_layer_style


class WorkerSeparate(QObject):
    """Worker for mode 'Separate Altitude ASL For Each Strip'."""

    finished = pyqtSignal(object, str)
    error = pyqtSignal(Exception, str)
    progress = pyqtSignal(int)
    enabled = pyqtSignal(bool)

    def __init__(self, **data):
        super().__init__()
        self.layer = data.get('pointLayer')
        self.crs_vct = data.get('crsVectorLayer')
        self.DTM = data.get('DTM')
        self.raster = data.get('raster')
        self.layer_pol = data.get('polygonLayer')
        self.crs_rst = data.get('crsRasterLayer')
        self.altitude_AGL = data.get('altitude_AGL')
        self.tab_widg_cor = data.get('tabWidg')
        self.g_line_list = data.get('LineRangeList')
        self.geom_aoi = data.get('Range')
        self.theta = data.get('theta')
        self.dist = data.get('distance')
        self.start_progress = data.get('start_progress', 0)
        self.killed = False

    def run_altitudeStrip(self):
        result = []
        try:
            strips_count = int(self.layer.maximumValue(0))
            progress_c = 0
            step = int(strips_count // 1000)

            if (self.crs_rst is None or not self.crs_rst.isValid() or self.crs_rst.isGeographic() or
                self.crs_vct is None or not self.crs_vct.isValid() or self.crs_vct.isGeographic()):
                raise ValueError("CRS must be valid (not geographic).")

            photos_list = [
                f.attributes()[:2] + [f.id(), f.geometry()]
                for f in self.layer_pol.getFeatures()
            ]
            photos_list.sort(key=lambda x: x[1])

            bbox = self.layer.extent()
            bbox.combineExtentWith(self.layer_pol.extent())
            bbox.grow(max(bbox.width(), bbox.height()) * 0.1 + 1.0)
            window = DtmWindow.from_layer(self.DTM, bbox, bbox_crs=self.crs_vct)

            for t in range(1, strips_count + 1):
                if self.killed:
                    self.handle_cancel()
                    return

                strip_nr = f"{t:04d}"
                nrP_max = 0
                nrP_min = 1000000
                BuffNr = None
                kappa = 0.0
                strip_feats = []

                for f in self.layer.getFeatures(f'"Strip" = \'{strip_nr}\''):
                    if self.killed:
                        self.handle_cancel()
                        return
                    num = int(f.attribute('Photo Number'))
                    nrP_max = max(nrP_max, num)
                    nrP_min = min(nrP_min, num)
                    if self.tab_widg_cor:
                        BuffNr = int(f.attribute('BuffNr'))
                    kappa = float(f.attribute('Kappa [deg]'))
                    point = f.geometry().asPoint()
                    strip_feats.append((f.id(), point.x(), point.y()))

                strip_photos = [f for f in photos_list if int(f[0]) == t]

                if not strip_photos or not strip_feats:
                    continue

                first_photo = strip_photos[0][-1].asPolygon()[0]
                last_photo = strip_photos[-1][-1].asPolygon()[0]
                points = first_photo + last_photo

                x_pnt = np.array([p.x() for p in points]).reshape(-1, 1)
                y_pnt = np.array([p.y() for p in points]).reshape(-1, 1)
                pnts = np.hstack((x_pnt, y_pnt))

                pnt1 = pnts[np.argmin(pnts[:, 0])]
                pnt2 = pnts[np.argmin(pnts[:, 1])]
                pnt3 = pnts[np.argmax(pnts[:, 0])]
                pnt4 = pnts[np.argmax(pnts[:, 1])]

                one_strip = [
                    QgsPointXY(pnt1[0], pnt1[1]),
                    QgsPointXY(pnt2[0], pnt2[1]),
                    QgsPointXY(pnt3[0], pnt3[1]),
                    QgsPointXY(pnt4[0], pnt4[1])
                ]
                g_strip = QgsGeometry.fromPolygonXY([one_strip])
                if kappa in [-90, 0, 90, 180]:
                    g_strip = QgsGeometry.fromPolygonXY([points])
                    g_strip = QgsGeometry.fromRect(g_strip.boundingBox())

                if self.tab_widg_cor:
                    common = g_strip.intersection(self.g_line_list[BuffNr - 1])
                    if common.isEmpty():
                        QgsPrint(f"Strip {t}: Intersection with corridor segment {BuffNr - 1} is empty, using full strip geometry instead.")
                        common = g_strip
                else:
                    common = g_strip.intersection(self.geom_aoi)
                    if common.isEmpty():
                        QgsPrint(f"Strip {t}: Intersection with Area of Interest is empty, using full strip geometry instead.")
                        common = g_strip

                mask = window.mask_for_geometry(common, src_crs=self.crs_vct)
                h_min, h_max = window.minmax(mask)
                avg_terrain_height = h_max - (h_max - h_min) / 3
                altitude_ASL = self.altitude_AGL + avg_terrain_height

                xs = np.array([item[1] for item in strip_feats])
                ys = np.array([item[2] for item in strip_feats])
                terrain_heights = window.sample(xs, ys, src_crs=self.crs_vct)

                self.layer.startEditing()
                for (feature_id, _, _), terrain_height in zip(strip_feats, terrain_heights):
                    if self.killed:
                        self.handle_cancel()
                        return
                    if np.isnan(terrain_height):
                        continue
                    altitude_AGL = altitude_ASL - terrain_height
                    self.layer.changeAttributeValue(feature_id, 5, round(altitude_AGL, 2))
                    self.layer.changeAttributeValue(feature_id, 4, round(altitude_ASL, 2))
                self.layer.commitChanges()

                progress_c += 1
                if step == 0 or progress_c % step == 0:
                    progress_value = self.start_progress + int(progress_c / strips_count * (100 - self.start_progress))
                    self.progress.emit(progress_value)

            waypoints_layer = create_waypoints(self.layer, self.crs_vct)
            waypoints_layer.setCrs(self.crs_vct)

            if not self.killed:
                self.progress.emit(100)
                flight_line = create_flight_line(waypoints_layer, self.crs_vct)
                flight_line.setCrs(self.crs_vct)

                style_path = os.path.join(
                    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                    'altitudes_utils',
                    'flight_line_style.qml'
                )
                flight_line.loadNamedStyle(style_path)

                if self.tab_widg_cor:
                    self.layer.startEditing()
                    self.layer.deleteAttributes([9, 10, 11])
                    self.layer.commitChanges()
                    self.layer_pol.startEditing()
                    self.layer_pol.deleteAttributes([2, 3])
                    self.layer_pol.commitChanges()

                change_layer_style(self.layer_pol, {'color': '200,200,200,30', 'color_border': '#000000', 'width_border': '0.2'})
                change_layer_style(self.layer, {'size': '1.0'})
                self.layer_pol.setName('photos')
                self.layer_pol.setCrs(self.crs_vct)
                self.layer.setName('projection_centres')
                self.layer.setCrs(self.crs_vct)

                result.extend([self.layer, flight_line, waypoints_layer, self.layer_pol])
        except Exception as e:
            import traceback
            self.error.emit(e, traceback.format_exc())
            self.progress.emit(0)
            self.enabled.emit(True)
            return

        self.finished.emit(result, "flight_design")
        self.enabled.emit(True)

    def handle_cancel(self):
        """Handle pressed Cancel button"""
        self.progress.emit(0)
        self.enabled.emit(True)
        self.finished.emit(None, "flight_design")
