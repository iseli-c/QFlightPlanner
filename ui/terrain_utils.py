from math import ceil, fabs
from pyproj import Transformer
from qgis import processing
from ..mathgeo_utils.coordinates import transf_coord
from .dtm_window import DtmWindow, raster_extent_contains


def create_buffer_around_line(path_line, gdal_ds, dtm_layer, buffer_value):
    """Creates a buffer polygon around a corridor line 
    and returns the buffered layer and minimum buffer."""
    gt = gdal_ds.GetGeoTransform()
    px_w, px_h = gt[1], -gt[5]
    ulx, uly = gt[0], gt[3]
    ulx_n, uly_n = ulx + px_w, uly + px_h

    crs_rst = dtm_layer.crs().authid()
    crs_vec = path_line.sourceCrs().authid()
    if crs_rst != crs_vec:
        tf = Transformer.from_crs(crs_rst, crs_vec, always_xy=True)
        ulx, uly = transf_coord(tf, ulx, uly)
        ulx_n, uly_n = transf_coord(tf, ulx_n, uly_n)

    min_buf = max(ceil(fabs(ulx_n - ulx)), ceil(fabs(uly_n - uly)))

    params = {
        "INPUT": path_line,
        "DISTANCE": buffer_value,
        "SEGMENTS": 5,
        "END_CAP_STYLE": 0,
        "JOIN_STYLE": 0,
        "MITER_LIMIT": 2,
        "DISSOLVE": False,
        "OUTPUT": "TEMPORARY_OUTPUT",
    }
    out = processing.run("native:buffer", params)["OUTPUT"]
    return out, min_buf


def aoi_within_dtm(vlayer, dtm_layer):
    """Verify that the whole vector layer lies within the DTM extent and has valid data.

    This is a cheap extent comparison (plus a small windowed nodata sanity check)
    that replaces the former whole-raster polygonization. Raises ValueError with a
    user-facing message; callers running on the GUI thread are responsible for
    presenting it.
    """
    bbox = vlayer.extent()
    if not raster_extent_contains(dtm_layer, bbox, bbox_crs=vlayer.crs()):
        raise ValueError("AoI does not lie entirely\nwithin the extent of the DTM data.")

    try:
        window = DtmWindow.from_layer(dtm_layer, bbox, bbox_crs=vlayer.crs(),
                                      max_pixels=1_000_000)
        window.minmax()
    except ValueError:
        raise ValueError("The DTM contains no valid data\nwithin the Area of Interest.")


def z_at_3d_line(pnt, start_pnt, end_pnt):
    """Return "z" coordinate for point with known x,y
    lying on line in space defined by start and end points.
    """
    x1, y1, z1 = start_pnt
    x2, y2, z2 = end_pnt
    x, y = pnt[:2]

    if x1 != x2:
        t = (x - x1) / (x2 - x1)
    else:
        t = (y - y1) / (y2 - y1)
    z = t * (z2 - z1) + z1
    return z


def simplify_profile(vertices, epsilon):
    """Reduces the number of vertices in the line, keeping its main shape.
    It is based on the Douglas-Peucker simplification algorithm but
    with the vertical distance instead of perpendicular.
    """
    hmax = 0.0
    index = 0
    for i in range(1, len(vertices) - 1):
        z = z_at_3d_line(vertices[i], vertices[0], vertices[-1])
        h = abs(z - vertices[i][2])
        if h > hmax:
            index = i
            hmax = h

    if hmax >= epsilon:
        results = simplify_profile(vertices[:index+1], epsilon)[:-1]\
                  + simplify_profile(vertices[index:], epsilon)
    else:
        results = [vertices[0], vertices[-1]]

    return results


def clipped_raster_minmax(vlayer, dtm_layer, max_pixels=50_000_000):
    """Calculates minimum and maximum elevation values from the DTM within a vector layer.

    Only the window covering the vector layer extent is read (windowed raster access),
    and a mask is built by rasterizing the geometry so that the statistics respect the
    polygon boundary.
    """
    aoi_within_dtm(vlayer, dtm_layer)

    bbox = vlayer.extent()
    window = DtmWindow.from_layer(dtm_layer, bbox, bbox_crs=vlayer.crs(),
                                  max_pixels=max_pixels)

    geometry = None
    for feature in vlayer.getFeatures():
        geom = feature.geometry()
        geometry = geom if geometry is None else geometry.combine(geom)

    mask = window.mask_for_geometry(geometry, src_crs=vlayer.crs()) if geometry else None
    return window.minmax(mask)
