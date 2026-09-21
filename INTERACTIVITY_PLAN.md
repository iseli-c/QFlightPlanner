# QFlightPlanner: Responsiveness & Interactivity Implementation Plan

## Context

The user runs the plugin against a **large DEM** (project base layer) with a **small AoI**.
Three problems:

1. **"Update Heights from DTM" freezes the GUI** — it runs synchronously on the UI thread
   and does whole-DEM processing. No progress/busy indication of any kind.
2. **Flight design runs are slow** for the same whole-DEM / O(n²) reasons.
3. **No interactivity**: changing flight direction requires a full "Run Design" re-run;
   the user wants to tweak direction (dial/spinbox/map) and see layers update live on the canvas.

All work happens in this repo: QGIS plugin, Python 3.12, PyQt5/Qt via `qgis.PyQt`, numpy/scipy/pyproj/osgeo.gdal available. No test suite exists — verify manually in QGIS (see Testing section).

---

## Root causes (verified in code)

### A. "Update Heights from DTM" freeze
Entry: `pushButtonGetHeights` → `TerrainSectionHandler.on_btn_get_heights_clicked()` (ui/terrain_section.py:31), synchronous on UI thread.

| Bottleneck | Location | Problem |
|---|---|---|
| `is_poligon_inside_raster()` | ui/terrain_utils.py:54–92 | Runs `gdal:rastercalculator` (`FORMULA='1'`, `EXTENT='ignore'`) over the **entire DEM**, then `gdal:polygonize` over the entire raster, then merges all polygons and does `contains()` tests. Catastrophically slow on a large DEM. |
| `clipped_raster_minmax()` | ui/terrain_utils.py:132–161 | Copies features into a temp memory layer, then `QgsZonalStatistics` over the DTM. Slower than a direct windowed array read. |
| No feedback | ui/terrain_section.py | No wait cursor, no progress bar usage, no threading. |

Note: `is_poligon_inside_raster` is **also called on every Run Design**
(ui/flight_design/altitudes_utils/initialization.py:71–73) — so it freezes design runs too.
`check_raster_values_on_polygon()` (ui/terrain_utils.py:40) is dead code (never called) and has a
swapped `block.value(col, row)` call — remove or fix it.

### B. Whole-DEM reads and O(n²) patterns in design / QC paths
| Bottleneck | Location | Problem |
|---|---|---|
| `prepare_raster_data()` | ui/flight_design/terrain_following/worker.py:102 | `ReadAsArray()` loads the **entire DEM** into memory; only the AoI window is ever needed. |
| `create_flight_profile_waypoints()` | ui/flight_design/terrain_following/worker.py:206–209 | `startEditing()`/`commitChanges()` **per projection centre** — thousands of edit sessions. |
| `run_altitudeStrip()` | ui/flight_design/separate_altitude/worker.py:83–88 | Rebuilds the **full photo feature list inside the per-strip loop** (O(strips×photos)). |
| Per-photo terrain sampling | ui/flight_design/separate_altitude/worker.py:153 and ui/flight_design/altitudes_utils/enrichments.py:26 | One `DTM.dataProvider().sample()` call per point — very slow per call. |
| Per-strip zonal statistics | ui/flight_design/separate_altitude/worker.py:134 | `raster_minmax_in_vector()` re-reads raster per strip. |
| `update_order()` | ui/flight_design/altitudes_utils/projection_centres.py:212–227 | Edit session per feature. |
| `add_photo_feature()` | ui/flight_design/altitudes_utils/projection_centres.py:130–143 | `updateExtents()` per feature (quadratic layer updates). |
| DTM reprojection | ui/flight_design/altitudes_utils/initialization.py:14–26 | `gdal:warpreproject` reprojects the **entire DTM** on the UI thread when CRS differs. |
| Whole-DEM read for Z_min | ui/quality_control/modules/footprints/process_footprints.py:29 | Full `ReadAsArray()` just for a min. |
| `clip_raster()` | ui/quality_control/modules/footprints/utils.py:10 | Re-reads the **entire DEM per photo**, then slices a tiny window. |
| `QApplication.processEvents()` in worker thread | ui/flight_design/terrain_following/worker.py:67 | Useless and unsafe in a QThread worker; signals already handle progress. |

### C. Direction handling today
- `DirectionSectionHandler` (ui/direction_section.py:16) syncs `dial` ↔ `spinBoxDirection`.
- `DirectionMapTool` (ui/direction_section.py:35): two clicks, rubber-band preview, emits azimuth on the second click only.
- No preview of the resulting flight plan; direction change ⇒ full "Run Design" click ⇒ full pipeline (including the polygonize-whole-DEM check above).

---

## Phase 1 — Fix "Update Heights from DTM" (highest priority)

### 1.1 New module `ui/dtm_window.py` — windowed DTM access (shared by later phases)

```python
class DtmWindow:
    """A windowed read of a DTM covering a requested bbox, with fast sampling."""

    @classmethod
    def from_layer(cls, dtm_layer: QgsRasterLayer, bbox_qgs: QgsRectangle,
                  margin: float = 0.0) -> "DtmWindow": ...

    # internals: gdal windowed read:
    #   xoff, yoff, xsize, ysize computed from bbox (transformed to raster CRS) via
    #   inverse geotransform or mathgeo_utils.coordinates.crs2pixel
    #   band.ReadAsArray(xoff, yoff, xsize, ysize)
    #   store: array (masked for nodata), geotransform (shifted to window origin),
    #           full_raster_shape, crs, nodata

    def sample(self, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
        """Vectorized nearest-neighbour terrain heights for coordinate arrays
        given in a (possibly different) CRS (pyproj Transformer, then
        crs2pixel + fancy indexing). Out-of-window values -> nodata."""

    def minmax(self, mask: np.ndarray = None) -> tuple[float, float]: ...
    def extent(self) -> QgsRectangle: ...          # valid-data extent in raster CRS
    def contains(self, bbox) -> bool: ...          # extent check
```

### 1.2 Replace `is_poligon_inside_raster` (ui/terrain_utils.py:54)
New function `aoi_within_dtm(aoi_layer, dtm_layer) -> None (raises ValueError)`:
1. Transform the AoI bounding box to the raster CRS (`QgsCoordinateTransform`, project context).
2. Compare against `dtm_layer.extent()`; raise the same user-facing message
   ("AoI does not lie entirely within the extent of the DTM data.") via `QgsMessBox` + ValueError.
3. Nodata sanity check (replaces polygonize semantics): `DtmWindow.from_layer(dtm, aoi_bbox)`
   subsampled read — if the window is 100% nodata, raise "DTM has no valid data within the AoI".
   (Pixel-perfect nodata coverage checks inside the polygon are not worth the cost here.)

Update both call sites: ui/terrain_utils.py:134 (min/max path) and
ui/flight_design/altitudes_utils/initialization.py:71–73. Delete `gdal:rastercalculator`/`gdal:polygonize` code entirely.

### 1.3 Replace `clipped_raster_minmax` (ui/terrain_utils.py:132)
New implementation using `DtmWindow`:
1. `win = DtmWindow.from_layer(dtm_layer, aoi_bbox)` — full-resolution windowed read
   (AoI is small relative to DEM by assumption).
2. Build an AoI polygon mask over the window by rasterizing the AoI geometry with
   `osgeo.gdal.Rasterize` into an in-memory dataset sharing the window geotransform
   (transform geometry to raster CRS first), **or** keep `QgsZonalStatistics` as a fallback
   if rasterization proves awkward — the dominant win is removing 1.2.
3. `h_min, h_max = win.minmax(mask)` (masked `np.nanmin`/`np.nanmax`).
4. If the window exceeds a pixel budget (e.g. > 100 M px), subsample the read for min/max
   and log a warning that extremes may be approximate.

### 1.4 Processing indication + threading
- Add `WorkerGetHeights(QObject)` (clone the pattern of `WorkerControl`, ui/quality_control/worker.py):
  signals `finished(tuple)`, `error(Exception, str)`, `progress(int)`, plus `killed` flag.
  It runs: buffer creation (corridor case, existing `create_buffer_around_line`) → `aoi_within_dtm` → min/max.
- In `TerrainSectionHandler.on_btn_get_heights_clicked` (ui/terrain_section.py:31):
  - `QApplication.setOverrideCursor(Qt.WaitCursor)` … `restoreOverrideCursor()` in a `finally`.
  - Disable `pushButtonGetHeights` while running; drive `dlg.progressBar`:
    `setFormat("Updating heights from DTM…")`, then `setRange(0, 0)` (busy indicator) → restore format/range at the end.
  - Start worker via `QThread` exactly like `FlightPlannerDialog.startWorker_control` (flight_planner_dialog.py:481).
    Keep it simple: if a previous heights worker is running, ignore the click (button is disabled anyway).
  - On finish: set `doubleSpinBoxMinHeight`/`doubleSpinBoxMaxHeight` (same as today).

### Phase 1 acceptance criteria
- "Update Heights from DTM" on (large DEM + small AoI) completes in **< 1 s** and never blocks the UI for more than a frame; busy cursor + progress bar shown during operation; results identical to before (min/max within the AoI polygon).
- Run Design no longer stalls at initialization (no `gdal:polygonize` anywhere).
- AoI partially/fully outside DEM still produces the existing critical message box.

---

## Phase 2 — Eliminate whole-DEM reads and O(n²) patterns

### 2.1 `WorkerTerrain.prepare_raster_data` (ui/flight_design/terrain_following/worker.py:100)
- Compute the flight bbox once: AoI geometry bbox (or corridor buffer bbox) expanded by the
  photo footprint half-diagonal `d` + exceed margins (`spinBoxExceedExtremeStrips` value × `Bx`).
- Read with `DtmWindow.from_layer(...)` instead of full `ReadAsArray()`.
- Everything downstream (`generate_simplified_profile`, `create_flight_profile_waypoints`)
  works unchanged because it only fancy-indexes `DTM_array` via `crs2pixel` — the windowed
  array + shifted geotransform are drop-in.

### 2.2 `WorkerTerrain.create_flight_profile_waypoints` (worker.py:179–218)
- One `startEditing()` before the per-strip loop, one `commitChanges()` after all strips
  (replace the per-projection-centre edit session at lines 206–209). Use
  `changeAttributeValue` in bulk; alternatively buffer changes and apply via
  `dataProvider().changeAttributeValues(dict)`.
- Delete `QApplication.processEvents()` at worker.py:67 (signals already emit progress).

### 2.3 `WorkerSeparate.run_altitudeStrip` (ui/flight_design/separate_altitude/worker.py:45–163)
- Hoist `photos_list` (lines 83–88) **out** of the strip loop; build once.
- Create one `DtmWindow` over the full flight bbox once before the loop; replace per-strip
  `raster_minmax_in_vector` (line 134) with masked min/max over the strip-geometry bbox slice
  of the window (rasterize strip polygon mask as in 1.3, or bbox-slice approximation if the
  difference proves immaterial — flag this choice in the PR).
- Replace the per-photo `dataProvider().sample()` (line 153) with a single vectorized
  `DtmWindow.sample(xs, ys)` per strip.

### 2.4 `enrich_projection_centres_with_agl` (ui/flight_design/altitudes_utils/enrichments.py:4)
- Build `DtmWindow` over the pc-layer bbox; replace the per-feature `sample()` loop with one
  `sample()` call over stacked coordinate arrays; keep one edit session (already single).

### 2.5 `initialize_design_environment` (ui/flight_design/altitudes_utils/initialization.py)
- Use `aoi_within_dtm` from 1.2 (drops the polygonize stall).
- Replace whole-DEM `gdal:warpreproject` (lines 14–26) with a windowed warp:
  `gdal.Warp('', src, outputBoundsSRS=..., outputBounds=<flight bbox>, dstSRS=ui.epsg_code)`
  into a small MEM/VRT dataset (or `gdal.Translate` with `projWin`). Only when CRS actually differs.
- Remove `QApplication.processEvents()` (line 41).

### 2.6 `projection_centres.py` (ui/flight_design/altitudes_utils/projection_centres.py)
- `update_order` (lines 205–228): single edit session around the loop, or better: collect
  id→(strip, photo) changes and apply once with `changeAttributeValues`.
- `add_photo_feature` (lines 130–143): drop the per-feature `pc_layer.updateExtents()` /
  `photo_layer.updateExtents()`; call `updateExtents()` once per layer after the loops in
  `projection_centres()`.

### 2.7 Quality control (secondary, same class of fix)
- `process_footprints` (ui/quality_control/modules/footprints/process_footprints.py:29):
  derive `Z_min` from the pc-layer bbox `DtmWindow` (or approximate band statistics)
  instead of a full-DEM read.
- `clip_raster` (ui/quality_control/modules/footprints/utils.py:5):
  replace full `ReadAsArray()` (line 10) with a windowed
  `band.ReadAsArray(xoff, yoff, xsize, ysize)` — the window is already computed in the function;
  keep return contract identical.

### Phase 2 acceptance criteria
- Terrain-Following and Separate-Altitude designs on (large DEM + small AoI) complete several×
  faster (target: no individual step > a few seconds; no full-DEM memory spike — verify memory
  stays bounded by AoI size).
- Output layers/attributes byte-identical in schema and values (within float rounding) to current output for a reference AoI.
- Quality Control footprint output unchanged.

---

## Phase 3 — Interactive flight direction

Goal: dragging the dial / typing in the spinbox / drawing on the map updates a lightweight
preview of strips + flight lines on the canvas within ~0.5 s, without a full Run Design.

### 3.1 Design inputs cache — `ui/flight_design/design_inputs.py`
```python
class DesignInputs:
    """Immutable snapshot of everything needed to (re)compute a flight geometry."""
    aoi_geom: QgsGeometry        # reprojected to project CRS, once
    aoi_crs, epsg_code
    dtm_window: DtmWindow        # window covering aoi bbox + max margin (photos + exceed)
    h_min, h_max: float          # from dtm_window
    camera, gsd                  # camera_handler.camera + doubleSpinBoxGSD
    Bx, By, Lx, Ly               # from calculate_flight_parameters
    exceed_strips, multiple_base # spinboxes
    # invalidate on: DTM combo change, AoI/corridor combo change, CRS change,
    # camera change, GSD change, overlap/sidelap change
```
- Build lazily on first preview/design; cache on the dialog (`ui._design_inputs`).
- Wire invalidation: `mMapLayerComboBoxDTM.layerChanged`, `mMapLayerComboBoxAoI/Corridor.layerChanged`,
  `crsSelector.crsChanged`, camera change callback (existing
  `camera_handler.on_camera_changed` hook), GSD/overlap spinbox signals.
- Direction is deliberately **not** part of the cache — it is the interactive variable.

### 3.2 Extract pure-geometry computation — refactor `process_block_mode`
(ui/flight_design/altitudes_utils/process_modes.py:11–37):
- Split into:
  - `compute_design_geometry(inputs, direction) -> (pc_layer, photo_layer)` —
    everything up to and including `projection_centres()` (fast, pure math; reuses
    `bounding_box_at_angle` and `projection_centres` after Phase 2 batching).
  - The final `Run Design` path calls `compute_design_geometry` then continues with the
    existing enrichment/waypoints/styling (unchanged behavior).
- This one function is then shared by preview and final run, guaranteeing "what you preview
  is what you get".

### 3.3 Preview rendering — persistent layers, updated in place
- On first preview: create two in-memory layers, add to a layer-tree group `"_flight_preview"`:
  - `preview_photos` (Polygon, fill `200,200,200,60`, border `0.2`) — style via existing
    `change_layer_style`.
  - `preview_flight_lines` (LineString, from strip end-points; simple red line style).
- On each recompute (in the `finished` handler on the UI thread):
  `dataProvider().truncate()` + `addFeatures(...)` + `layer.triggerRepaint()` — never remove/re-add
  layers (keeps canvas stable, no flicker).
- Preview must set `setCustomProperty("is_flight_preview", True)` or name prefix so
  "Run Design" output groups and cleanup logic ignore it.

### 3.4 Debounced, cancellable preview worker
- In `FlightPlannerDialog`:
  - `self._preview_timer = QTimer(self)`, single-shot, 250 ms.
  - Connect `dial.valueChanged`, `spinBoxDirection.valueChanged` (note: these already sync to
    each other — connect once to `spinBoxDirection.valueChanged` to avoid double fires) and the
    new drag signal (3.5) to `self._preview_timer.start()`.
  - On timeout: build `WorkerPreview` (pattern-clone of `WorkerSeparate`) that runs
    `compute_design_geometry(inputs, direction)` on a QThread and emits
    `finished(pc_layer, photo_layer)`.
  - Supersession: keep `self._preview_worker`/`_preview_thread`; if busy when a new request
    arrives, set `killed = True` on the old worker and start the new one
    (mirror `cancel_worker`, flight_planner_dialog.py:365).
  - Guard: only run when a valid AoI + camera exist; silently skip otherwise.
- Add a checkbox `checkBoxLivePreview` ("Live preview") to `flight_planner_dialog_base.ui`
  next to the direction group; default checked. When unchecked, disconnect the debounce path.

### 3.5 `DirectionMapTool` — live drag feedback (ui/direction_section.py)
- Add `directionDragging = pyqtSignal(int)` emitted from `canvasMoveEvent` while the second point
  is being placed (same azimuth math as `_calculate_direction`).
- Optionally (nice-to-have, do last): a "drag handle" mode — on activation, draw a line through
  the AoI centroid along the current direction; user grabs and rotates it; emit
  `directionDragging` continuously and `directionPicked` on release. Skip if time-boxed.
- `directionDragging` connects to the same debounce timer (3.4).

### Phase 3 acceptance criteria
- Dragging the dial (or drawing on canvas) updates preview photos + flight lines on the
  canvas within ~250 ms debounce + compute time (< ~0.5 s total on the reference AoI),
  with no GUI freezes (compute happens off the UI thread).
- Rapid direction changes never pile up (superseded workers are killed; only latest result renders).
- Turning "Live preview" off restores current behavior exactly.
- "Run Design" output remains identical to pre-change output for the same inputs.

---

## Phase 4 — Polish, verification, rollout

1. Progress/feedback sweep: every long operation (Run Design, Run Control, Get Heights, preview
   warm-up) shows either percentage progress, busy progress bar, or busy cursor, and all run
   buttons disable during their operation.
2. Error paths: each new worker emits `error` → `workerError`-style handler (existing
   `QgsMessBox`/`QgsPrint` conventions).
3. Remove dead code: `check_raster_values_on_polygon` (ui/terrain_utils.py:40), and the
   `gdal:rastercalculator`/`gdal:polygonize` remnants.
4. Keep code style: docstrings on public functions (matching existing convention), no other
   comments; QGIS version guards (`Qgis.QGIS_VERSION_INT`) for QMetaType as done in
   geoprocessing_utils.py:68.

### Testing (manual — no test suite in repo)
Build a reference scenario and record timings before/after each phase:
1. Large DEM (≥ 10k × 10k px) covering far more than the AoI; small AoI polygon; corridor line.
2. For each: Get Heights; One-Altitude design; Separate-Altitude design; Terrain-Following design;
   Quality Control on the produced pc layer; interactive direction dragging.
3. Edge cases: AoI crossing DEM nodata region; AoI partially outside DEM (expect the critical
   message box, not a crash); DTM CRS ≠ project CRS (windowed warp path); direction near 0/90/180/270
   (`bounding_box_at_angle` has a special branch there); CRS-change invalidation of the cache.
4. Timing harness: quick `QgsMessageLog` timing lines (`QgsPrint`) around the previously-hot spots
   are acceptable during development but remove them (or gate behind a DEBUG flag) before finishing.

---

## Suggested implementation order & effort
1. Phase 1 (Get Heights + `DtmWindow` + `aoi_within_dtm`): the user's acute pain; self-contained.
2. Phase 2 (whole-DEM + O(n²) sweep): makes everything, including preview, fast.
3. Phase 3 (interactive preview): biggest new surface; depends on 1–2.
4. Phase 4 (polish).

Each phase is independently shippable. Phases 1–2 are low risk (same outputs, faster);
Phase 3 is additive UI (new checkbox + preview layers) and can be feature-flagged off.
