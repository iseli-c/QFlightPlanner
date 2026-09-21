from math import sqrt

from qgis.core import QgsCoordinateReferenceSystem, QgsProject, QgsRasterLayer
from ....error_reporting import QgsMessBox
from ...terrain_utils import aoi_within_dtm
from qgis import processing


def _flight_bbox_in_target(ui, margin=0.0):
    """Bounding box of the AoI/corridor (already in target CRS) expanded by margin."""
    layer = ui.AreaOfInterest if ui.tabBlock else ui.CorLine
    bbox = layer.extent()
    if margin:
        bbox.grow(margin)
    return bbox


def _design_margin(ui):
    """Generous margin covering the photo footprint and exceeded strips."""
    camera = getattr(ui.camera_handler, 'camera', None)
    if camera is None:
        return 0.0
    gsd = ui.doubleSpinBoxGSD.value() / 100
    diagonal = sqrt((camera.pixels_along_track * gsd) ** 2
                    + (camera.pixels_across_track * gsd) ** 2)
    return diagonal * (2 + ui.spinBoxExceedExtremeStrips.value())


def initialize_crs_and_progressbar(ui):
    """Validate CRS of DTM and configure progress bar format"""
    target_crs = QgsCoordinateReferenceSystem(ui.epsg_code)

    if ui.DTM and ui.DTM.crs().isValid():
        if ui.DTM.crs().authid() != ui.epsg_code:
            try:
                bbox = _flight_bbox_in_target(ui, _design_margin(ui))
                res = processing.run("gdal:warpreproject", {
                    'INPUT': ui.DTM,
                    'SOURCE_CRS': ui.DTM.crs().authid(),
                    'TARGET_CRS': ui.epsg_code,
                    'RESAMPLING': 0,
                    'NODATA': None,
                    'OPTIONS': '',
                    'DATA_TYPE': 0,
                    'EXTRA': f'-te {bbox.xMinimum()} {bbox.yMinimum()} '
                             f'{bbox.xMaximum()} {bbox.yMaximum()}',
                    'OUTPUT': 'TEMPORARY_OUTPUT'
                })
                ui.DTM = QgsRasterLayer(res['OUTPUT'], "Reprojected DTM Layer")
            except Exception as e:
                QgsMessBox('Reprojection Error', f'Failed to reproject DTM: {str(e)}')

        ui.crs_rst = ui.DTM.crs()
    else:
        ui.crs_rst = target_crs
        QgsMessBox('DTM CRS Error', f'Your DTM did not have CRS.\n{ui.epsg_code} set.')

    ui.pushButtonRunDesign.setEnabled(False)
    ui.progressBar.setValue(0)
    ui.progressBar.setFormat("%p%")
    ui.progressBar.setRange(0, 100)


def initialize_design_environment(ui):
    ui.progressBar.setFormat("Initializing")

    target_crs_auth = ui.epsg_code

    # Vector layers reprojection logic
    if ui.tabBlock:
        if ui.AreaOfInterest.crs().authid() != target_crs_auth:
            res = processing.run("native:reprojectlayer", {
                'INPUT': ui.AreaOfInterest,
                'TARGET_CRS': target_crs_auth,
                'OUTPUT': 'memory:Reprojected AoI Layer'
            })
            ui.AreaOfInterest = res['OUTPUT']

    elif ui.tabCorridor:
        if ui.CorLine.crs().authid() != target_crs_auth:
            res = processing.run("native:reprojectlayer", {
                'INPUT': ui.CorLine,
                'TARGET_CRS': QgsCoordinateReferenceSystem(target_crs_auth),
                'OUTPUT': 'memory:Reprojected Corridor Layer'
            })

            ui.CorLine = res['OUTPUT']
            ui.pathLine = ui.CorLine.source()
            QgsProject.instance().addMapLayer(ui.CorLine, False)

    initialize_crs_and_progressbar(ui)

    try:
        if ui.tabBlock:
            aoi_within_dtm(ui.AreaOfInterest, ui.DTM)
        elif ui.tabCorridor:
            aoi_within_dtm(ui.CorLine, ui.DTM)
    except ValueError as e:
        QgsMessBox(title="AoI not in DTM", text=str(e), level="Critical")
        ui.pushButtonRunDesign.setEnabled(True)
        ui.progressBar.setValue(0)
        raise e
