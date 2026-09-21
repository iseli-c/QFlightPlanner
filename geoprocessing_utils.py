from qgis.core import (
    QgsFeature,
    QgsFeatureRequest,
    QgsField,
    QgsGeometry,
    QgsPointXY,
    QgsPoint,
    QgsProject,
    QgsVectorLayer,
    Qgis
)
from qgis.PyQt.QtCore import QMetaType, QVariant
import re


def selected_features_subset(layer):
    """Return a memory layer with only the selected features of ``layer``.

    Returns None when the layer is missing or nothing is selected.
    """
    if layer is None:
        return None
    feature_ids = layer.selectedFeatureIds()
    if not feature_ids:
        return None
    request = QgsFeatureRequest().setFilterFids(feature_ids)
    subset = layer.materialize(request)
    if subset is not None:
        subset.setName(layer.name())
        subset.updateExtents()
    return subset

def add_to_canvas(layers, group_name, counter=1, name=None):
    """Adding layer/layers to canvas"""
    root = QgsProject.instance().layerTreeRoot()
    group = root.insertGroup(0, f"{name or group_name}_{counter}")

    if isinstance(layers, list):
        QgsProject.instance().addMapLayers(layers, False)
        iterable_layers = layers
    else:
        QgsProject.instance().addMapLayer(layers)
        iterable_layers = [layers]

    prefix = f"{name}_" if name else ""
    for layer in iterable_layers:
        layer.setName(prefix + layer.name() + f"_{counter}")
        group.addLayer(layer)


def change_layer_style(layer, properties):
    """Change layer style according to the properties."""
    renderer = layer.renderer()
    symbol = renderer.symbol()
    renderer.setSymbol(symbol.createSimple(properties))
    layer.triggerRepaint()


def create_flight_line(waypoints_lyr, crs_vect):
    """Create one flight line feature per strip through its waypoints."""
    flight_line = QgsVectorLayer("LineStringZ?crs=" + str(crs_vect),
                                        "flight_line", "memory")
    pr = flight_line.dataProvider()

    strips = {}
    for w in waypoints_lyr.getFeatures():
        x = w.geometry().asPoint().x()
        y = w.geometry().asPoint().y()
        z = w.attribute('Alt. ASL [m]')
        strip = w.attribute('Strip')
        strips.setdefault(strip, []).append(QgsPoint(x, y, z))

    for waypoints in strips.values():
        if len(waypoints) < 2:
            continue
        feat = QgsFeature()
        feat.setGeometry(QgsGeometry.fromPolyline(waypoints))
        pr.addFeature(feat)
    flight_line.updateExtents()
    return flight_line

def create_waypoints_layer(crs_vect):
    waypoints_layer = QgsVectorLayer("Point?crs=" + str(crs_vect),
                              "waypoints", "memory")
    pr = waypoints_layer.dataProvider()
    if Qgis.QGIS_VERSION_INT >= 33800:
        t_int = QMetaType.Type.Int
        t_dbl = QMetaType.Type.Double
        t_str = QMetaType.Type.QString
    else:
        t_int = QVariant.Int
        t_dbl = QVariant.Double
        t_str = QVariant.String

    pr.addAttributes([
        QgsField("Waypoint Number", t_int),
        QgsField("X [m]", t_dbl),
        QgsField("Y [m]", t_dbl),
        QgsField("Alt. ASL [m]", t_dbl),
        QgsField("Alt. AGL [m]", t_dbl),
        QgsField("Strip", t_str)
    ])
    waypoints_layer.updateFields()
    
    return pr, waypoints_layer

def create_waypoints(projection_centres, crs_vect):
    """Create points where altitude or direction of flight change."""

    pr, waypoints_layer = create_waypoints_layer(crs_vect)

    strips_nr = int(projection_centres.maximumValue(0))
    feats = projection_centres.getFeatures()
    featList = [feat.attributes()[:6] + [feat.geometry()] for feat in feats]
    featList.sort(key=lambda x: x[1])

    waypoint_nr = 1
    for strip_nr in range(1, strips_nr+1):
        strip = [f for f in featList if int(f[0]) == strip_nr]
        start_waypoint = strip[0]
        end_waypoint = strip[-1]

        x_start = start_waypoint[-1].asPoint().x()
        y_start = start_waypoint[-1].asPoint().y()
        x_end = end_waypoint[-1].asPoint().x()
        y_end = end_waypoint[-1].asPoint().y()

        s_nr = f"{strip_nr:04d}"

        feat_pnt = QgsFeature()
        pnt_start = QgsPointXY(x_start, y_start)
        feat_pnt.setGeometry(QgsGeometry.fromPointXY(pnt_start))
        feat_pnt.setAttributes([waypoint_nr] + start_waypoint[2:6] + [s_nr])
        pr.addFeature(feat_pnt)
        waypoint_nr += 1

        feat_pnt = QgsFeature()
        pnt_end = QgsPointXY(x_end, y_end)
        feat_pnt.setGeometry(QgsGeometry.fromPointXY(pnt_end))
        feat_pnt.setAttributes([waypoint_nr] + end_waypoint[2:6] + [s_nr])
        pr.addFeature(feat_pnt)
        waypoint_nr += 1

        waypoints_layer.updateExtents()
    return waypoints_layer

def find_matching_field(layer, patterns):
    """Find matching field name that contains given pattern"""
    def normalize(name):
        return re.sub(r'[^a-z]', '', name.lower())

    norm_patterns = [normalize(p) for p in patterns]

    for field in layer.fields():
        norm_name = normalize(field.name())
        if all(p in norm_name for p in norm_patterns):
            return field.name()
    return None
