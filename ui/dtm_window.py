import math

import numpy as np
from osgeo import gdal, ogr, osr
from pyproj import Transformer
from qgis.core import (
    QgsCoordinateReferenceSystem,
    QgsCoordinateTransform,
    QgsGeometry,
    QgsProject,
    QgsRectangle,
)

from ..mathgeo_utils.coordinates import crs2pixel


def transform_bbox(bbox, src_crs, dst_crs):
    """Transform a QgsRectangle between two coordinate reference systems."""
    if bbox is None or src_crs is None or dst_crs is None or src_crs == dst_crs:
        return bbox
    transform = QgsCoordinateTransform(src_crs, dst_crs, QgsProject.instance())
    return transform.transformBoundingBox(bbox)


class DtmWindow:
    """A windowed read of a DTM covering a requested bounding box.

    Only the pixels intersecting the requested area are read into memory, which
    keeps operations fast and memory-bounded even for a very large DEM when the
    area of interest covers just a small part of it.
    """

    def __init__(self, array, geotransform, crs, nodata, window, decimated=False):
        self.array = array
        self.geotransform = geotransform
        self.crs = crs
        self.nodata = nodata
        self.window = window
        self.decimated = decimated
        self._transformers = {}

    @classmethod
    def from_layer(cls, dtm_layer, bbox, bbox_crs=None, margin=0.0, max_pixels=None):
        """Read the DTM window covering ``bbox`` (optionally expanded by margin)."""
        dataset = gdal.Open(dtm_layer.source())
        if dataset is None:
            raise ValueError("Cannot open the DTM raster.")

        geotransform = dataset.GetGeoTransform()
        if geotransform is None or gdal.InvGeoTransform(geotransform) is None:
            raise ValueError("The DTM has an invalid geotransform.")

        crs = dtm_layer.crs()
        bbox = transform_bbox(bbox, bbox_crs, crs)
        if margin:
            bbox = QgsRectangle(
                bbox.xMinimum() - margin,
                bbox.yMinimum() - margin,
                bbox.xMaximum() + margin,
                bbox.yMaximum() + margin,
            )

        width, height = dataset.RasterXSize, dataset.RasterYSize
        inv_geo = gdal.InvGeoTransform(geotransform)
        c0, r0 = gdal.ApplyGeoTransform(inv_geo, bbox.xMinimum(), bbox.yMaximum())
        c1, r1 = gdal.ApplyGeoTransform(inv_geo, bbox.xMaximum(), bbox.yMinimum())

        xoff = max(0, int(math.floor(min(c0, c1))))
        yoff = max(0, int(math.floor(min(r0, r1))))
        xend = min(width, int(math.ceil(max(c0, c1))) + 1)
        yend = min(height, int(math.ceil(max(r0, r1))) + 1)
        if xend <= xoff or yend <= yoff:
            raise ValueError("The requested area does not overlap the DTM.")

        xsize, ysize = xend - xoff, yend - yoff
        buf_xsize, buf_ysize = xsize, ysize
        if max_pixels is not None and xsize * ysize > max_pixels:
            factor = int(math.ceil(math.sqrt((xsize * ysize) / max_pixels)))
            buf_xsize = max(1, xsize // factor)
            buf_ysize = max(1, ysize // factor)

        band = dataset.GetRasterBand(1)
        nodata = band.GetNoDataValue()
        array = band.ReadAsArray(xoff, yoff, xsize, ysize, buf_xsize, buf_ysize)
        if array is None:
            raise ValueError("Failed to read the DTM window.")
        array = np.array(array, dtype=float)
        if nodata is not None:
            array = np.ma.masked_equal(array, nodata)
        else:
            array = np.ma.masked_invalid(array)

        decimated = buf_xsize != xsize or buf_ysize != ysize
        window_gt = list(geotransform)
        window_gt[0] = geotransform[0] + xoff * geotransform[1] + yoff * geotransform[2]
        window_gt[3] = geotransform[3] + xoff * geotransform[4] + yoff * geotransform[5]
        if decimated:
            window_gt[1] = geotransform[1] * (xsize / buf_xsize)
            window_gt[4] = geotransform[4] * (ysize / buf_ysize)
            window_gt[2] = geotransform[2] * (xsize / buf_xsize)
            window_gt[5] = geotransform[5] * (ysize / buf_ysize)

        return cls(array, window_gt, crs, nodata, (xoff, yoff, xsize, ysize), decimated)

    def _transformer(self, src_crs):
        key = src_crs.authid() or src_crs.toWkt()
        if key not in self._transformers:
            src = src_crs.authid() or src_crs.toWkt()
            dst = self.crs.authid() or self.crs.toWkt()
            self._transformers[key] = Transformer.from_crs(src, dst, always_xy=True)
        return self._transformers[key]

    def pixel_indices(self, xs, ys, src_crs=None):
        """Return integer (cols, rows, valid) pixel indices for coordinate arrays."""
        xs = np.atleast_1d(np.asarray(xs, dtype=float))
        ys = np.atleast_1d(np.asarray(ys, dtype=float))
        if src_crs is not None and not src_crs == self.crs:
            transformer = self._transformer(src_crs)
            xs, ys = transformer.transform(xs, ys)
            xs = np.asarray(xs, dtype=float)
            ys = np.asarray(ys, dtype=float)

        cols, rows = crs2pixel(self.geotransform, xs, ys)
        cols = np.rint(cols).astype(int)
        rows = np.rint(rows).astype(int)
        ny, nx = self.array.shape
        valid = (cols >= 0) & (cols < nx) & (rows >= 0) & (rows < ny)
        return cols, rows, valid

    def sample(self, xs, ys, src_crs=None):
        """Return terrain heights (float, NaN where invalid) for coordinate arrays."""
        xs = np.atleast_1d(np.asarray(xs, dtype=float))
        ys = np.atleast_1d(np.asarray(ys, dtype=float))
        cols, rows, valid = self.pixel_indices(xs, ys, src_crs)

        result = np.full(xs.shape, np.nan, dtype=float)
        if np.any(valid):
            values = self.array[rows[valid], cols[valid]]
            result[valid] = np.ma.filled(np.ma.asarray(values, dtype=float), np.nan)
        return result

    def minmax(self, mask=None):
        """Return (min, max) of valid values, optionally restricted by a boolean mask."""
        data = self.array
        if mask is not None:
            data = np.ma.masked_array(data, mask=np.logical_not(mask) | np.ma.getmaskarray(data))
        values = data.compressed()
        if values.size == 0:
            raise ValueError("No valid DTM data in the requested area.")
        return float(values.min()), float(values.max())

    def mask_for_geometry(self, geometry, src_crs=None):
        """Rasterize a geometry into a boolean mask aligned with the window.

        Only the sub-window covering the geometry bounding box is rasterized.
        """
        if geometry is None or geometry.isEmpty():
            return None
        if src_crs is not None and not src_crs == self.crs:
            transform = QgsCoordinateTransform(src_crs, self.crs, QgsProject.instance())
            geometry = QgsGeometry(geometry)
            geometry.transform(transform)

        ny, nx = self.array.shape
        gbbox = geometry.boundingBox()
        c_left, r_top = crs2pixel(self.geotransform, gbbox.xMinimum(), gbbox.yMaximum())
        c_right, r_bottom = crs2pixel(self.geotransform, gbbox.xMaximum(), gbbox.yMinimum())

        x0 = max(0, int(math.floor(min(c_left, c_right))))
        y0 = max(0, int(math.floor(min(r_top, r_bottom))))
        x1 = min(nx, int(math.ceil(max(c_left, c_right))) + 1)
        y1 = min(ny, int(math.ceil(max(r_top, r_bottom))) + 1)

        mask = np.zeros((ny, nx), dtype=bool)
        if x1 <= x0 or y1 <= y0:
            return mask

        sub_gt = list(self.geotransform)
        sub_gt[0] = self.geotransform[0] + x0 * self.geotransform[1] + y0 * self.geotransform[2]
        sub_gt[3] = self.geotransform[3] + x0 * self.geotransform[4] + y0 * self.geotransform[5]

        mem_raster = gdal.GetDriverByName('MEM').Create('', x1 - x0, y1 - y0, 1, gdal.GDT_Byte)
        mem_raster.SetGeoTransform(sub_gt)
        mem_raster.SetProjection(self.crs.toWkt())
        mem_raster.GetRasterBand(1).Fill(0)

        srs = osr.SpatialReference()
        srs.ImportFromWkt(self.crs.toWkt())
        mem_vector = ogr.GetDriverByName('Memory').CreateDataSource('mask')
        layer = mem_vector.CreateLayer('mask', srs=srs, geom_type=ogr.wkbUnknown)
        feature = ogr.Feature(layer.GetLayerDefn())
        feature.SetGeometry(ogr.CreateGeometryFromWkt(geometry.asWkt()))
        layer.CreateFeature(feature)

        gdal.RasterizeLayer(mem_raster, [1], layer, burn_values=[1])
        mask[y0:y1, x0:x1] = mem_raster.GetRasterBand(1).ReadAsArray().astype(bool)
        return mask

    def extent(self):
        """Return the window extent in the DTM CRS."""
        gt = self.geotransform
        ny, nx = self.array.shape
        xs = [gt[0], gt[0] + nx * gt[1], gt[0] + ny * gt[2],
              gt[0] + nx * gt[1] + ny * gt[2]]
        ys = [gt[3], gt[3] + nx * gt[4], gt[3] + ny * gt[5],
              gt[3] + nx * gt[4] + ny * gt[5]]
        return QgsRectangle(min(xs), min(ys), max(xs), max(ys))

    def contains(self, bbox):
        """Return True when ``bbox`` (in the DTM CRS) lies within the window extent."""
        return self.extent().contains(bbox)


def raster_extent_contains(dtm_layer, bbox, bbox_crs=None):
    """Return True when the bbox lies entirely within the DTM raster extent."""
    raster_bbox = transform_bbox(bbox, bbox_crs, dtm_layer.crs())
    return dtm_layer.extent().contains(raster_bbox)
