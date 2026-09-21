import numpy as np

from ...dtm_window import DtmWindow


def enrich_projection_centres_with_agl(ui, pc_lay):
    """Enrich projection centres layer with altitude AGL"""
    if not hasattr(ui, 'DTM') or ui.DTM is None:
        return

    asl_field = pc_lay.fields().indexOf('Alt. ASL [m]')
    agl_field = pc_lay.fields().indexOf('Alt. AGL [m]')

    bbox = pc_lay.extent()
    window = DtmWindow.from_layer(ui.DTM, bbox, bbox_crs=ui.crs_vct)

    ids, xs, ys, asls = [], [], [], []
    for f in pc_lay.getFeatures():
        point = f.geometry().asPoint()
        ids.append(f.id())
        xs.append(point.x())
        ys.append(point.y())
        asls.append(f.attribute(asl_field))

    if not ids:
        ui.progressBar.setValue(70)
        return

    terrain_heights = window.sample(np.array(xs), np.array(ys), src_crs=ui.crs_vct)

    pc_lay.startEditing()
    for feature_id, altitude_asl, terrain_height in zip(ids, asls, terrain_heights):
        if np.isnan(terrain_height) or altitude_asl is None:
            continue
        pc_lay.changeAttributeValue(feature_id, agl_field,
                                    round(altitude_asl - terrain_height, 2))
    pc_lay.commitChanges()
    ui.progressBar.setValue(70)
