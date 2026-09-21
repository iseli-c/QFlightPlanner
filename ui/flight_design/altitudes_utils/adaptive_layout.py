from dataclasses import dataclass
from math import ceil, cos, radians, sin, sqrt

import numpy as np
from qgis.core import QgsGeometry, QgsPointXY

from ....mathgeo_utils.coordinates import lines_intersection

MIN_FACTOR = 0.25
MIN_DELTA = 0.5
MAX_DELTA = 10.0


@dataclass
class AdaptivePlan:
    """Terrain-adaptive placement of flight lines and photos.

    ``line_offsets`` are the perpendicular offsets ``d_k`` of every flight line
    from the starting boundary of the bounding box, in the same direction the
    uniform layout grows (``C2 = b_ll + sign * d * sqrt(a_ll ** 2 + 1)``).

    ``photo_offsets[k]`` are the along-track offsets ``v_j`` of every photo on
    line ``k``, measured from the AoI along-start in the flight direction.
    """

    line_offsets: list
    photo_offsets: list


def _unit_vectors(alpha):
    """Return the along-track unit vector of the flight direction."""
    rad = radians(alpha)
    return cos(rad), sin(rad)


def _offset_c(a_ll, b_ll, sign, offset):
    """C coefficient of the line parallel to the strips at perpendicular offset."""
    return b_ll + sign * offset * sqrt(a_ll ** 2 + 1.0)


def _anchor(a_ll, b_ll, sign, offset, a_l_, b_l_):
    """Point where the strip line at ``offset`` crosses the along-start boundary."""
    return lines_intersection(a_ll, _offset_c(a_ll, b_ll, sign, offset), a_l_, b_l_)


def _slab(q_lo, q_hi, ux, uy, start, end):
    """Build the rotated rectangle between two parallel strip lines."""
    x_lo, y_lo = q_lo
    x_hi, y_hi = q_hi
    points = [
        QgsPointXY(x_lo + start * ux, y_lo + start * uy),
        QgsPointXY(x_lo + end * ux, y_lo + end * uy),
        QgsPointXY(x_hi + end * ux, y_hi + end * uy),
        QgsPointXY(x_hi + start * ux, y_hi + start * uy),
    ]
    return QgsGeometry.fromPolygonXY([points])


def _factor(z, H, h_agl):
    """Local footprint shrink factor at terrain elevation ``z``."""
    raw = (H - z) / h_agl
    if raw < MIN_FACTOR:
        raise ValueError(
            "Terrain rises too close to the flight altitude to keep the "
            "specified overlap.\n"
            f"Terrain {z:.1f} m ASL, flight altitude {H:.1f} m ASL "
            f"(AGL {h_agl:.1f} m).\n"
            "Increase the GSD / altitude AGL or reduce the flight area."
        )
    return min(1.0, raw)


def corridor_factor(z_max, H, h_agl):
    """Conservative uniform spacing factor from the highest corridor terrain."""
    return _factor(z_max, H, h_agl)


def _pair_band_max(dtm_window, aoi_geom, geom_crs, a_ll, b_ll, sign, d, Lx0, Ly0,
                   By_max, a_l_, b_l_, ux, uy, Dx):
    """Maximum DTM elevation under the footprint-union band of a line pair."""
    q_lo = _anchor(a_ll, b_ll, sign, d - Ly0 / 2.0, a_l_, b_l_)
    q_hi = _anchor(a_ll, b_ll, sign, d + By_max + Ly0 / 2.0, a_l_, b_l_)
    band = _slab(q_lo, q_hi, ux, uy, -Lx0, Dx + Lx0)
    clip = band.intersection(aoi_geom)
    if clip.isEmpty():
        return None
    mask = dtm_window.mask_for_geometry(clip, src_crs=geom_crs)
    if mask is None or not mask.any():
        return None
    try:
        return dtm_window.minmax(mask)[1]
    except ValueError:
        return None


def _pixel_delta(dtm_window):
    """Sampling step along the profile, tied to the DTM pixel size."""
    gt = dtm_window.geotransform
    px = sqrt(gt[1] ** 2 + gt[4] ** 2)
    py = sqrt(gt[2] ** 2 + gt[5] ** 2)
    return min(max(max(px, py), MIN_DELTA), MAX_DELTA)


def plan_adaptive_layout(dtm_window, aoi_geom, geom_crs, alpha,
                         a_ll, b_ll, a_l_, b_l_, Dx, Dy,
                         Lx0, Ly0, p_spec, q_spec, H, h_agl,
                         exceed_pct, m):
    """Greedy terrain-adaptive line placement and per-photo along-track walk.

    Returns an :class:`AdaptivePlan`. Raises ``ValueError`` when terrain rises
    within ``MIN_FACTOR`` of the nominal AGL below the flight altitude.
    """
    Bx_max = Lx0 * (1.0 - p_spec)
    By_max = Ly0 * (1.0 - q_spec)
    if Bx_max <= 0 or By_max <= 0:
        raise ValueError("Overlap settings leave no room for flight lines or photos.")

    sign = 1 if alpha > 90 and alpha <= 270 else -1
    ux, uy = _unit_vectors(alpha)
    d0 = (0.5 - exceed_pct / 100.0) * Ly0

    if Dy <= 2 * d0:
        line_offsets = [Dy / 2.0]
    else:
        line_offsets = []
        last = Dy - d0
        d = d0
        while True:
            line_offsets.append(d)
            z_star = _pair_band_max(
                dtm_window, aoi_geom, geom_crs, a_ll, b_ll, sign, d,
                Lx0, Ly0, By_max, a_l_, b_l_, ux, uy, Dx)
            factor = 1.0 if z_star is None else _factor(z_star, H, h_agl)
            d_next = d + By_max * factor
            if d_next >= last:
                break
            d = d_next
        if last - line_offsets[-1] > 1e-9:
            line_offsets.append(last)

    D_nom = (ceil(Dx / Bx_max) * Bx_max - Dx) / 2.0
    v0 = -D_nom - m * Bx_max
    v_end = Dx + D_nom + m * Bx_max
    delta = _pixel_delta(dtm_window)

    photo_offsets = [
        _walk_line(dtm_window, aoi_geom, geom_crs, a_ll, b_ll, a_l_, b_l_, sign,
                   d, Lx0, Ly0, Bx_max, By_max, H, h_agl, Dx, v0, v_end,
                   ux, uy, delta)
        for d in line_offsets
    ]
    return AdaptivePlan(line_offsets, photo_offsets)


def _walk_line(dtm_window, aoi_geom, geom_crs, a_ll, b_ll, a_l_, b_l_, sign,
               d, Lx0, Ly0, Bx_max, By_max, H, h_agl, Dx, v0, v_end,
               ux, uy, delta):
    """Per-photo greedy walk along one flight line, guaranteeing front overlap."""
    qx, qy = _anchor(a_ll, b_ll, sign, d, a_l_, b_l_)

    sample_count = int(ceil(Dx / delta)) + 1
    along = np.linspace(0.0, Dx, sample_count)
    xs = qx + along * ux
    ys = qy + along * uy
    heights = dtm_window.sample(xs, ys, src_crs=geom_crs)

    inside = np.zeros(along.shape, dtype=bool)
    q_lo = _anchor(a_ll, b_ll, sign, d - Ly0 / 2.0, a_l_, b_l_)
    q_hi = _anchor(a_ll, b_ll, sign, d + Ly0 / 2.0, a_l_, b_l_)
    strip = _slab(q_lo, q_hi, ux, uy, -Lx0, Dx + Lx0)
    clip = strip.intersection(aoi_geom)
    if not clip.isEmpty():
        mask = dtm_window.mask_for_geometry(clip, src_crs=geom_crs)
        if mask is not None and mask.any():
            cols, rows, valid = dtm_window.pixel_indices(xs, ys, src_crs=geom_crs)
            inside[valid] = mask[rows[valid], cols[valid]]
    usable = inside & np.isfinite(heights)

    offsets = []
    v = v0
    while v <= v_end:
        offsets.append(v)
        lo = v - Lx0 / 2.0
        hi = v + Bx_max + Lx0 / 2.0
        selected = usable & (along >= lo) & (along <= hi)
        if not np.any(selected):
            step = Bx_max
        else:
            z_star = float(np.max(heights[selected]))
            step = Bx_max * _factor(z_star, H, h_agl)
        v += step
    return offsets
