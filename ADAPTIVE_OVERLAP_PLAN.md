# QFlightPlanner: Terrain-Adaptive Overlap & Spacing Implementation Plan

## Context

All line numbers refer to the current state of the repository. Python 3.12, PyQt via
`qgis.PyQt`, numpy / pyproj / `osgeo.gdal` available. No test suite exists — verification
is manual in QGIS (same convention as `INTERACTIVITY_PLAN.md`).

The plugin's "One Altitude ASL For Entire Flight" mode currently:

- **Uses average terrain height for the flight altitude.**
  `calculate_altitude` computes `altitude_ASL = (min_h + max_h) / 2 + altitude_AGL`
  (ui/flight_design/altitudes_utils/altitude_calculation.py:6). Consequence: over the
  *lowest* DEM elevation in the AoI the achieved GSD is coarser than specified.
- **Has a broken `checkBoxIncreaseOverlap`.** When checked,
  `calculate_flight_parameters` references `ui.p` / `ui.q`
  (ui/flight_design/altitudes_utils/flight_parameters.py:12-13), which are only assigned
  in the *unchecked* branch (lines 6-8) and are never computed anywhere else. Checking
  the box makes the design run fail with an `AttributeError` (silently logged via
  `QgsTraceback`). The same conditional is mirrored in
  ui/flight_design/design_inputs.py:18-23.
- **Lays out a uniform grid.** `projection_centres`
  (ui/flight_design/altitudes_utils/projection_centres.py:144) places `Ny` parallel
  strips at constant spacing `By` and `Nx` photos per strip at constant spacing `Bx`,
  all at a single altitude `H`. Footprints `Lx = pixels_along_track * gsd`,
  `Ly = pixels_across_track * gsd` are nominal (at the specified GSD). Over terrain
  higher than the altitude reference the real footprints shrink, so the specified
  front/side overlap is not maintained.
- **Requires a manual "Update Heights from DTM" button.** `pushButtonGetHeights`
  (flight_planner_dialog_base.ui:591) → `TerrainSectionHandler.on_btn_get_heights_clicked`
  (ui/terrain_section.py:36) → `WorkerGetHeights` on a QThread →
  `doubleSpinBoxMinHeight` / `doubleSpinBoxMaxHeight` populated in `_on_finished`
  (ui/terrain_section.py:86-94). Consumers of the button:
  flight_planner_dialog.py:100 (signal connection), ui/altitude_section.py:143
  (enable/disable), ui/terrain_section.py:60 and 102 (disable/enable during run).

Reusable infrastructure (verified):

- `DtmWindow` (ui/dtm_window.py): windowed DTM reads, vectorized
  `sample(xs, ys, src_crs)`, exact `minmax(mask)`, and `mask_for_geometry(geom, src_crs)`
  rasterization of arbitrary (rotated) polygons. Precedent for per-strip masked
  statistics: ui/flight_design/separate_altitude/worker.py:128-129.
- `clipped_raster_minmax` (ui/terrain_utils.py:98) — masked min/max under a vector layer.
- Live preview: `DesignInputs` (ui/flight_design/design_inputs.py) →
  `compute_design_geometry` (ui/flight_design/altitudes_utils/process_modes.py:19) →
  `projection_centres`; runs on a QThread with 250 ms debounce
  (flight_planner_dialog.py:106-120, 273-303). Block-only by design.
- The checkbox is only enabled for the "One Altitude ASL For Entire Flight" altitude type
  (ui/altitude_section.py:142) — correct gating, keep it.

---

## Requirements

1. **GSD at the lowest elevation.** When a DEM/DTM is used, the flight altitude must be
   `H = z_min + AGL`, so the specified GSD is achieved at the *lowest* DEM elevation in
   the AoI (and bettered everywhere else). Applies in one-altitude mode, whether the
   overlap checkbox is checked or not.
2. **Per-line side overlap (checkbox checked).** Flight line spacing is adjusted per
   flight line so the specified side overlap is guaranteed at *all* elevations.
3. **Per-photo front overlap (checkbox checked).** Forward (along-track) photo spacing is
   adjusted photo-by-photo along each line so the specified front overlap is guaranteed
   at all elevations, avoiding unnecessary extra photos over low terrain.
4. **Corridor mode**: a conservative *global* uniform adjustment (single `Bx`/`By`
   derived from the maximum terrain elevation under the corridor buffer). No refactor of
   the corridor numbering pre-pass.
5. **Preview**: a new checkbox controls whether the live preview applies the adaptive
   layout (it can be disabled if it impacts preview speed).
6. **Automatic terrain heights.** Min/max terrain heights are computed automatically
   whenever the relevant inputs change (DTM, AoI, corridor line, buffer, block/corridor
   tab). The "Update Heights from DTM" button is removed.
7. **Safety.** If terrain rises so close to the flight altitude that the footprint factor
   collapses, the design must error out with a clear message — the overlap spec is never
   silently violated.

### Decisions locked with the product owner

| Decision | Choice |
|---|---|
| Corridor behaviour when checked | Conservative global uniform adjustment (no numbering refactor) |
| Forward overlap granularity | Per photo (explicitly requested, to minimize photo count over low terrain) |
| Preview | Optional via a new checkbox, default off (speed concern) |
| Terrain too close to flight altitude | Error out, never warn-and-clamp |

---

## Math model (single altitude `H` for the whole flight)

- `z_min` = `doubleSpinBoxMinHeight` (auto-refreshed per Requirement 6);
  `h_AGL` = AGL from GSD (`= gsd / sensor_size * focal_length`);
  **`H = z_min + h_AGL`**.
- Nominal footprints at the specified GSD: `Lx0 = pixels_along_track * gsd`,
  `Ly0 = pixels_across_track * gsd`.
- At terrain elevation `z` the local footprint shrinks by the **factor**

  ```
  factor(z) = (H - z) / h_AGL        # == 1.0 at z == z_min, smaller on higher terrain
  ```

  clamped to `[MIN_FACTOR, 1.0]`, `MIN_FACTOR = 0.25`.
- Maximum (nominal) spacings: `Bx_max = Lx0 * (1 - p_spec)`,
  `By_max = Ly0 * (1 - q_spec)`, where `p_spec` / `q_spec` are the front overlap and
  side overlap spinbox fractions.
- Adaptive spacing rules (conservative, footprint-union model):
  - **Pair spacing** between line `k` and `k+1`:
    `By_k = By_max * factor(z*_pair)` where `z*_pair` is the maximum DTM elevation under
    the pair's footprint-union band: across-offset range
    `[d_k - Ly0/2, d_k + By_max + Ly0/2]`, the full along-extent of the AoI, clipped to
    the AoI.
  - **Per-photo step** from photo `j` at along-offset `v_j`:
    `step_j = Bx_max * factor(z*_step)` where `z*_step` is the maximum elevation on the
    strip's center-line profile over `[v_j - Lx0/2, v_j + Bx_max + Lx0/2]`
    (in-AoI samples only).
- If any required `factor < MIN_FACTOR` → abort the design with `QgsMessBox`
  (terrain rises within 25 % of the nominal AGL below the flight altitude → the user
  must increase the GSD/AGL or reduce the AoI).
- Guarantee: overlap ≥ spec at every elevation, because each spacing is derived from the
  *highest* terrain in its constraint region; over the lowest terrain the spacing equals
  nominal (fewest photos — the stated goal of per-photo adaptation).

---

## Algorithm — greedy line placement + per-photo walk

### Line placement (replaces uniform `Ny` / `By_o`)

1. `d_0 = (0.5 - x/100) * Ly0` — same first-line edge offset as the current code
   (`calculate_offsets`, projection_centres.py:37; `x` = `spinBoxExceedExtremeStrips`).
2. Special case `Dy <= 2*d_0` → single line at `Dy/2` (matches the current `Ny == 1`
   branch, projection_centres.py:34-35).
3. Greedy loop, given `d_k`: mask the pair band (rotated rectangle with the ±`Ly0/2`
   margins above) intersected with the AoI → `DtmWindow.minmax` → `By_k`;
   `d_{k+1} = d_k + By_k`. If `d_{k+1} >= Dy - d_0` → clamp to `Dy - d_0` and stop
   (clamping only moves the last line closer, i.e. increases overlap — safe).
4. Empty band ∩ AoI (concave AoI) → `By_k = By_max` (nominal fallback).

With flat terrain at `z_min` this yields full `By_max` steps — never more lines than
necessary, and at least as few as today (today's uniform `By_o <= By_max` redistributes
lines to exactly span `Dy_o`; the greedy walk with the clamped last line is the
spec-meeting equivalent).

### Per-photo walk (per line `k`)

1. Sample the strip's center line densely: one vectorized `DtmWindow.sample` call at
   spacing `delta` = DTM pixel size (clamp to `[0.5 m, 10 m]`); discard out-of-AoI
   samples using the strip's `mask_for_geometry` lookup (vectorized array indexing).
2. First photo at `v_0 = -D_nom - m * Bx_nom`, where
   `D_nom = (ceil(Dx / Bx_nom) * Bx_nom - Dx) / 2` and `m` = `spinBoxMultipleBase`
   — exact parity with the uniform grid start (the uniform grid spans
   `[-D - m*Bx, Dx + D + m*Bx]`).
3. Walk `v_{j+1} = v_j + step_j` (step from the profile slice max per the math model,
   capped at `Bx_max`) until `v >= Dx + D_nom + m * Bx_nom`.
4. On flat terrain the walk reproduces the uniform layout exactly.

### Conventions that MUST be preserved (from `calculate_offsets`)

- Across-offset sign: `sign = 1 if 90 < alpha <= 270 else -1`.
- Along-offset sign: `sign2 = -1 if 0 <= alpha <= 180 else 1`.
- Line equation `a*x - y + C = 0`; a parallel line at perpendicular distance `d`:
  `C2(d) = C1 + sign * d * sqrt(a**2 + 1)`.
- Perpendicular unit vector `(cos(alpha - 90°), sin(alpha - 90°))`;
  along unit vector `(cos(alpha), sin(alpha))`.
- Photo-column centering uses the `C22 = b_l_ + sign2 * D * sqrt(a2**2 + 1)` shift with
  the per-strip `D` (see Phase 4).

---

## Phase 1 — Automatic terrain heights (remove the button)

### 1.1 UI (flight_planner_dialog_base.ui)
- Delete the `pushButtonGetHeights` widget (grid item row 10, column 0; lines 590-596).

### 1.2 Dialog wiring (flight_planner_dialog.py)
- Remove the connection at line 100
  (`pushButtonGetHeights.clicked.connect(...)`).
- Call `self.terrain_handler.refresh_heights()` (new method, see 1.3) from:
  - `on_mMapLayerComboBoxDTM_layerChanged` (line 188) — including the
    `lyr is None` case, where `terrain_handler.set_dtm(None, None)` must also be called
    (today the handler keeps a stale DTM when the combo is cleared);
  - `on_mMapLayerComboBoxAoI_layerChanged` (line 199);
  - `on_mMapLayerComboBoxCorridor_layerChanged` (line 211);
  - `on_tabWidgetBlockCorridor_currentChanged` (line 221) — the sampled geometry
    changes with the tab;
  - a new connection `self.doubleSpinBoxBuffer.valueChanged` → refresh (the buffer
    defines the corridor sampling area).
- Startup note: `_init_default_layers` (line 166) triggers these handlers before any
  layers are chosen — `refresh_heights` must silently no-op on incomplete inputs.

### 1.3 Handler (ui/terrain_section.py)
- Rename `on_btn_get_heights_clicked` (line 36) to public `refresh_heights()`
  (drop the `@pyqtSlot()` decorator).
- Incomplete inputs (no DTM; block without AoI; corridor without line) → **silent
  no-op** (no message boxes — this fires on every combo change; real failures still go
  through `_on_error`).
- Replace the drop-new-requests guard (line 51-52,
  `if self.thread is not None and self.thread.isRunning(): return`) with a pending
  pattern: if a worker is running, set `self._pending = True` and return; `_on_finished`
  re-runs `refresh_heights()` once when the flag is set (mirrors
  `FlightPlannerDialog._preview_pending`, flight_planner_dialog.py:273-303). The last
  input state must always be the one computed.
- Keep the progress-bar feedback ("Updating heights from DTM..." format) and wait
  cursor as-is.
- Remove `_on_enabled` (line 101) and its `worker.enabled` connection (line 77) — the
  button no longer exists.
- Keep `_on_finished` behaviour: set `doubleSpinBoxMinHeight` /
  `doubleSpinBoxMaxHeight`, and for corridor set
  `doubleSpinBoxBuffer.setMinimum(min_buf / 2)` (lines 86-94).

### 1.4 Enabler (ui/altitude_section.py)
- Remove `self.dialog.pushButtonGetHeights.setEnabled(enable)` (line 143).

### 1.5 Worker (ui/get_heights_worker.py)
- Unchanged (all state is constructor-injected). Optionally drop the now-unused
  `enabled` signal.

### Semantics
- Heights refresh on every relevant input change **regardless of altitude type** —
  values are always fresh when the user switches to one-altitude mode.
- The spinboxes stay editable (manual override possible), but the next input change
  overwrites them — `z_min` / `z_max` used downstream are therefore always current.

---

## Phase 2 — Min-based flight altitude

- ui/flight_design/altitudes_utils/altitude_calculation.py:
  `altitude_ASL = min_h + altitude_AGL` (drop the average; keep the
  `(altitude_ASL, altitude_AGL)` return signature). `max_h` becomes unused in this
  function — remove the read or keep it only if still needed for the progress bar.
- ui/flight_design/design_inputs.py::_altitude_asl (line 28): identical change
  (preview mirror).

---

## Phase 3 — Remove the broken `ui.p` / `ui.q` stub

- ui/flight_design/altitudes_utils/flight_parameters.py: set `ui.p`, `ui.q` from the
  spinboxes **unconditionally** (delete the `if not ...isChecked()` conditional,
  lines 6-8). `Bx`, `By` returned are always the nominal values.
- ui/flight_design/design_inputs.py::_flight_parameters (lines 18-23): simplify to
  match (remove the `getattr` fallbacks).
- Docstrings: state that `ui.p` / `ui.q` are the *specified* overlap fractions; the
  terrain adaptation (Phase 4) consumes them, it does not mutate them.

---

## Phase 4 — Adaptive block layout (core)

### 4.1 New module `ui/flight_design/altitudes_utils/adaptive_layout.py`

```python
@dataclass
class AdaptivePlan:
    line_offsets: list[float]      # perpendicular offsets d_k of every line
    photo_offsets: list[list[float]]  # per line: along-offsets v_j of every photo

def plan_adaptive_layout(dtm_window, aoi_geom, geom_crs, alpha,
                         a_ll, b_ll, a_l_, b_l_, Dx, Dy,
                         Lx0, Ly0, p_spec, q_spec, H, h_AGL,
                         exceed_pct, m) -> AdaptivePlan:
    """Greedy terrain-adaptive line placement + per-photo walk (see Algorithm)."""
```

Helpers (all private to the module):
- rotated-rectangle polygon builder (anchor + along/perpendicular unit vectors);
- pair-band max: `dtm_window.mask_for_geometry(band ∩ aoi, src_crs=geom_crs)` +
  `dtm_window.minmax(mask)`; empty intersection → nominal fallback (`By_max`);
- strip profile: one vectorized `dtm_window.sample(...)` along the center line at
  `delta` = DTM pixel size (clamp `[0.5 m, 10 m]`), out-of-AoI samples removed via the
  strip mask array lookup;
- per-step slice max over `[v_j - Lx0/2, v_j + Bx_max + Lx0/2]` (numpy slice of the
  profile);
- `factor` computation with the `[MIN_FACTOR, 1.0]` clamp; on `factor < MIN_FACTOR`
  raise `ValueError` with a user-facing message (the caller shows `QgsMessBox` first,
  mirroring `initialize_design_environment`'s pattern, initialization.py:98-102).

### 4.2 `projection_centres.py`
- `projection_centres(..., adaptive_plan=None)` (new keyword parameter at the end).
- `adaptive_plan is None` → **existing code path byte-identical**.
- With a plan:
  - line positions from `plan.line_offsets` (advance by `(d_{k+1} - d_k)` along the
    perpendicular unit vector);
  - per line, photo positions from `plan.photo_offsets[k]` (start point from the
    `C2(d_k)` / `C22` line-intersection construction with the strip-specific `D_k`);
  - generalize the strip polygon (`create_strip_geometry`, lines 93-105) to build from
    the actual first/last photo positions (same corner trig, per-photo coordinates);
  - keep the serpentine `kappa` (line 157), the
    `central_line.distance(geom_strip) <= m * Bx_nom` trim filter (line 166, use the
    nominal `Bx`), and `update_order` renumbering unchanged — all work off the placed
    features, so variable per-strip photo counts need no changes there;
  - verify the `first_p` / `first_s` capture logic (lines 176-180) still holds when a
    strip places zero photos (pre-existing edge case, preserve current behaviour).

### 4.3 `process_modes.py`
- `compute_design_geometry(..., adaptive_plan=None)` — pass-through parameter.
- `process_block_mode(ui, Bx, By, len_along, len_across, altitude_ASL, altitude_AGL)`
  (new `altitude_AGL` parameter):
  - if `ui.checkBoxIncreaseOverlap.isChecked()`:
    - build `DtmWindow.from_layer(ui.DTM, aoi_bbox + margin, bbox_crs=ui.crs_vct)` where
      `margin` reuses the `_design_margin` logic (initialization.py:18 — footprint
      diagonal × (2 + exceed));
    - `plan = plan_adaptive_layout(...)` with `H = altitude_ASL`,
      `h_AGL = altitude_AGL`, `p_spec = ui.p`, `q_spec = ui.q`;
    - on `ValueError` → `QgsMessBox` with the message, reset progress bar, return `None`
      (let `run_design_one_altitude` abort cleanly);
    - pass the plan into `compute_design_geometry`.
  - Runs synchronously (≈ `Ny` mask rasterizations + `Ny` vectorized samples — well
    under 2 s on the reference AoI; acceptable, one-altitude design is already
    synchronous).
- `one_altitude/run_design.py`: pass `altitude_AGL` (currently discarded at line 19)
  through to `process_block_mode` / `process_corridor_mode`; handle a `None` return
  from Phase 4.3/5 (aborted design).

---

## Phase 5 — Corridor: conservative global adjustment

- `process_corridor_mode(ui, ..., altitude_ASL, altitude_AGL)`:
  - if `ui.checkBoxIncreaseOverlap.isChecked()`:
    - compute `z*_buf` = maximum DTM elevation under the merged corridor segment
      buffers (one `DtmWindow` over the corridor bbox +
      `mask_for_geometry(combined buffer geometry)`, or `clipped_raster_minmax` on the
      merged buffer layer);
    - `factor = clamp((H - z*_buf) / h_AGL, MIN_FACTOR, 1.0)`; on `factor < MIN_FACTOR`
      → `QgsMessBox` + abort (same as Phase 4);
    - otherwise use the uniform scalars `Bx_adj = Bx * factor`,
      `By_adj = By * factor` everywhere in the existing path — the numbering pre-pass
      (`corridor_flight_numbering`, process_modes.py:145) and `projection_centres` are
      called with the adjusted scalars. **No structural change**; numbering stays valid
      because counts remain uniform per segment.

---

## Phase 6 — Preview option

### 6.1 UI (flight_planner_dialog_base.ui)
- Add `checkBoxPreviewAdaptive` next to `checkBoxLivePreview`:
  text "Adaptive spacing in preview", tooltip "Apply terrain-adaptive line/photo
  spacing in the live preview (slower). Only used when the adaptive overlap option is
  checked." Default **unchecked**.
- Update the `checkBoxIncreaseOverlap` label (line 696) to reflect the new behaviour,
  e.g. "Adapt line/photo spacing to local terrain (maintain minimum front/side
  overlap)".

### 6.2 Dialog (flight_planner_dialog.py)
- `checkBoxPreviewAdaptive.toggled` → `schedule_preview()`;
- keep it enabled/disabled in sync with `checkBoxIncreaseOverlap`
  (its `toggled` → `checkBoxPreviewAdaptive.setEnabled`).

### 6.3 Preview path (ui/flight_design/design_inputs.py)
- New `DesignInputs` fields: `adaptive: bool`
  (= `checkBoxIncreaseOverlap.isChecked() and checkBoxPreviewAdaptive.isChecked()`),
  `dtm_layer`, `p_spec`, `q_spec`, `h_AGL`.
- In `compute(direction)`: when `adaptive`, build a `DtmWindow`
  (`DtmWindow.from_layer(ui.DTM, ...)` handles a DTM in any CRS via `bbox_crs` /
  `src_crs` — the preview does **not** run the design-time DTM warp), run the same
  `plan_adaptive_layout`, and call `compute_design_geometry(..., adaptive_plan=...)`
  so preview == design output.
- Missing DTM or sampling errors in the preview → fall back to the nominal layout and
  `QgsPrint` a warning (never a modal box from a worker thread).

---

## Phase 7 — Polish

- `QgsMessBox` text for the `MIN_FACTOR` violation: name the offending elevation, the
  flight altitude, and the remedy (increase GSD/AGL or reduce the AoI).
- Repo conventions: docstrings only (no comments), `QgsMessBox` / `QgsPrint` /
  `QgsTraceback` error patterns, no new threads.
- README: one-paragraph note that terrain heights are auto-computed and that the
  adaptive checkbox guarantees the specified overlaps at all elevations.

---

## Acceptance criteria & manual verification (no test suite)

Reference scenario: large DEM (≥ 10k × 10k px), small AoI polygon, corridor line,
camera preset; record behaviour before/after each phase.

1. **Automatic heights**: selecting/changing DTM, AoI, corridor line, buffer, or
   block/corridor tab refreshes min/max spinboxes within a second, without clicking
   anything; no message boxes when inputs are incomplete; rapid input changes never
   leave stale values (pending-rerun picks up the last state); the corridor buffer
   minimum side effect (`setMinimum(min_buf/2)`) still works; no `pushButtonGetHeights`
   anywhere in the UI or code.
2. **Unchecked box**: layout identical to today except `Alt. ASL [m]` = min spinbox +
   AGL (verify in the projection-centres attribute table).
3. **Flat AoI, checked**: layout ≈ nominal (max spacings); overlaps ≈ spec exactly.
4. **Uneven AoI, checked**: Quality Control overlap map shows overlap ≥ spec everywhere;
   photo/line count < global-conservative count; per-photo intervals visibly tighter
   over hills and wider over valleys; `Alt. AGL [m]` varies per photo.
5. **Corridor, checked**: uniform conservative spacing; QC overlap ≥ spec.
6. **Preview toggle**: adaptive preview matches Run Design output for the same
   direction; disabling it restores the fast nominal preview.
7. **Safety**: AoI with relief exceeding 75 % of `h_AGL` → Run Design aborts with the
   clear message box (not a traceback-only failure); the preview silently falls back
   to nominal.
8. **Edge cases**: concave AoI (empty band/strip intersections → nominal fallback, no
   crash); `alpha` near 0/90/180/270 (`bounding_box_at_angle` special branch);
   DTM CRS ≠ project CRS (windowed warp path at design time; `src_crs` handling in the
   preview); `Dy <= 2*d_0` single-line case; a strip that intersects no AoI area.

---

## Out of scope

- Separate-altitude and terrain-following modes (unchanged).
- Fixpoint "walking" refinement of pair spacing (the band-max approach is conservative
  and within about one line of optimal).
- Moving the adaptive planner into a background worker (synchronous is acceptable at
  current cost).

## Suggested implementation order

1. Phase 1 (automatic heights) — self-contained UX change, unblocks fresh `z_min` for
   everything else.
2. Phases 2-3 (min altitude, stub removal) — trivial, immediately verifiable.
3. Phase 4 (adaptive block layout) — the core; largest new surface.
4. Phase 5 (corridor) — small once Phase 4 exists.
5. Phase 6 (preview option) — additive UI.
6. Phase 7 (polish).

Each phase is independently shippable.
