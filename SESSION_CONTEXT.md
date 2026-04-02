# Session Context — 2026-03-30 / 2026-03-31

## What This Session Accomplished

### Phase 3: Gabor Directional Filter Bank (DONE)
- Created `gabor_filter_bank.m`: 8-orientation × 2-scale Gabor bank computing directional contrast.
- Integrated as multiplicative weight on `clean_detection` in `sharpness_detection_with_mirage_suppression.m`.
- Weight = `0.2 + 0.8 * min(dc / 2.0, 1.0)`. Isotropic blobs suppressed to 20% floor; linear trail passes at full strength.
- Debug maps: `maps/gabor_dc/`.

### Phase 6: 2D DWT Codec Artifact Suppression (DONE)
- Created `dwt2_enhance.m`: 2-level manual Haar DWT, codegen-safe.
- **Initial version** zeroed LL2 entirely — too aggressive, removed the trail along with the artifact.
- **Fixed version** uses LL2 median subtraction (removes uniform codec artifact, preserves trail peaks) and THRESH_SCALE=0.5 (softer VisuShrink).
- Processing order: 4-channel sum → DWT denoise → Gabor directional weight.
- Debug maps: `maps/dwt_approx/`.

### Anisotropy-Weighted Trail Extraction (DONE — not yet verified)
- Removed `find_densest_cluster` blob detector from Mode 1. It locked onto the bright isotropic blob instead of the thin linear trail.
- New Mode 1: accumulate detection maps → intensity cap at 80th percentile → structure tensor anisotropy weighting → PCA on weighted map.
- Anisotropy = (λ1 − λ2) / (λ1 + λ2) from smoothed gradient structure tensor. Thin lines → weight ≈ 1; blobs → weight ≈ 0.
- Added `compute_anisotropy`, `percentile_cap`, `box_smooth_local` local functions.
- Debug maps: `maps/pca_pre_weight/`, `maps/pca_aniso/`, `maps/pca_post_weight/`.
- **Status**: Code written, NOT YET RUN. Needs verification.

---

## Known Issues (Unsolved)

### 1. Trail detection gap over snow-covered regions
The trail breaks into a discontinuity where the ground is snow-covered (low-texture). The Laplacian, temporal diff, and intensity channels all fail in these regions. The Gabor phase channel (weight 0.50) was added to bridge these gaps but may not fully resolve them. Documented in CLAUDE.md under "Known Artifacts" item #3.

### 2. Trajectory over-extrapolation
The quadratic fit extends well past the visible trail (~280px) across the entire frame. In run 2026-03-30_22-11-10, path X ranged [1, 1748] and path Y [39, 841] — nearly full-frame. Needs quality gates (Phase 4): fit RMSE check, trajectory direction sanity, confidence normalization.

### 3. Impact detection not triggering
Splash signal too weak relative to baseline. Max splash_energy = 2370 vs baseline max = 2370 — no margin. May need to lower the threshold, change the ROI, or use a different detection approach. Impact ROI is configured at (1451, 589) with radius 100.

### 4. Mode 2 trajectory coefficients frozen after onset
Once the quadratic fit is established in Mode 1, Mode 2 follows the prediction but never updates the coefficients. The comment in the code says "Mode 2 trajectory refinement DISABLED." If the initial fit is wrong (e.g., dominated by blob), the trajectory never self-corrects. The anisotropy-weighted PCA should improve the initial fit, but periodic re-fitting in Mode 2 would add robustness.

### 5. Persistent phase channel pattern (visual clutter)
The Gabor phase channel shows a static bright arc in the upper-left (treeline/vegetation boundary) in every frame. This is a z-score bias from high-variance textured regions. Not harmful to tracking (spatially separated from trail) but clutters detection maps.

### 6. H.264/H.265 codec macroblock artifacts (partially solved)
Rectangular bright regions in detection maps during bullet onset (frames 184-190). The DWT's LL2 median subtraction addresses this, but effectiveness needs verification with the latest code changes.

### 7. Anisotropy-weighted PCA — verified, DOES NOT suppress blob
Run 2026-03-31_16-59-21 (structure tensor): elongation=2.43, PCA passed but trail axis dominated by blob. Trail extrapolates full frame (X=[234,1810], Y=[1,782]).
Run 2026-04-01_19-45-16 (Gabor DC replacement): elongation=1.989, PCA FAILED (below 2.0 threshold). Zero tracking, zero path points. Gabor DC weight ≈ 1.0 everywhere — no discrimination.

**Key insight: local orientation operators cannot solve this problem.**
Both the structure tensor (pixel-level gradient) and Gabor directional contrast (multi-scale orientation energy) are LOCAL operators. They answer "does this pixel have a preferred orientation?" but cannot answer "does this pixel belong to a long continuous linear structure?" In the accumulated diff map, vegetation edges, codec block boundaries, and noise texture are ALL locally directional. The trail's uniqueness is its GLOBAL linear coherence (~280px), not its local orientation. Any pixel-level weighting scheme (structure tensor, Gabor DC, Harris corner suppression, etc.) will fail on this input because the accumulated diff map has widespread directional signal from non-trail sources.

**What works upstream but not here**: The Gabor DC works in the detection pipeline because the per-frame detection map has the trail as an isolated feature against a clean background. The accumulated diff map over 5 frames has signal everywhere (vegetation movement, codec drift, phase noise), destroying the contrast that the Gabor bank needs.

**Next approach**: Morphological filtering with elongated structuring elements. This tests GLOBAL linear extent (does a line structuring element fit this feature?) rather than local orientation. Only structures that match the element's length and angle survive — short edges, blobs, and noise are removed by the opening operation.

---

## Future Plans

### Phase 4: Confidence Tuning & Fit Quality Gate (TODO)
- Add fit RMSE quality gate to `trajectory_integrated_detection.m` (reject if RMSE > 200px).
- Add trajectory direction sanity check.
- Tune `LOST_CONF_THRESH`, `FLIGHT_GRACE`, `LOST_TIMEOUT` in `flight_state_machine.m` based on measured confidence values.
- Normalize confidence to [0, 1] range.

### Phase 5: Kalman Filter (TODO)
- New file `kalman_trajectory.m`: 6-state constant-acceleration model [x, y, vx, vy, ax, ay].
- Initialize from quadratic fit coefficients.
- Replace polynomial evaluation in Mode 2 with Kalman predict/update.
- Provides uncertainty estimates and smooth prediction through gaps.

### Mode 2 Trajectory Refinement (TODO)
- Periodically re-accumulate detection maps and re-run anisotropy-weighted PCA.
- Only update fit if new PCA axis is consistent with current trajectory (angle within threshold).
- Prevents the feedback loop where a bad initial fit leads to searching in the wrong area.

### Gabor Floor Tuning (TODO)
- Current GABOR_FLOOR = 0.2 may still let too much blob signal through.
- Consider reducing to 0.05 or 0.01 now that the centroid extraction no longer relies on blob brightness.

### Impact Detection Fix (TODO)
- Lower splash threshold or change detection approach.
- Consider frame differencing in the impact ROI instead of absolute energy.

---

## File Inventory

| File | Role | Status |
|------|------|--------|
| `main_pipeline.m` | Top-level frame loop | Modified, working |
| `temporal_median_background.m` | Background estimation | Unchanged |
| `sharpness_detection_with_mirage_suppression.m` | 4-channel detection + DWT + Gabor | Modified, working |
| `gabor_filter_bank.m` | 8-orient × 2-scale directional contrast | New, working |
| `dwt2_enhance.m` | 2-level Haar DWT denoising | New, fixed (LL2 median sub) |
| `flight_state_machine.m` | 4-state machine | Unchanged |
| `impact_detection.m` | Impact splash detector | Unchanged, not triggering |
| `trajectory_integrated_detection.m` | Centroid extraction + quadratic fit | Major rewrite (anisotropy PCA), NEEDS TESTING |
| `path_history_buffer.m` | Path accumulation | Unchanged |
| `video_overlay.m` | Frame annotation | Modified (blob/centroid markers) |
| `save_log.m` | CSV/MAT log export | Modified |
| `analyze_video.m` | Diagnostic figures | Modified |
| `select_impact_roi.m` | Interactive ROI selection | Unchanged |
| `rgb_debug.m` | RGB debug utility | Unchanged |

## Key Run Results

| Run | Key Observation |
|-----|-----------------|
| `2026-03-30_22-11-10` | PCA working, n_det=3, trajectory over-extrapolates full frame |
| `2026-03-30_22-28-43` | Gabor filter bank added; codec rectangular artifact visible in td_cleaned |
| `2026-03-30_22-56-06` | DWT too aggressive (LL2_ATTEN=0), detection maps nearly black |
| `2026-03-30_23-05-19` | DWT fixed (LL2 median sub), trail visible but blob dominates PCA. Thin linear trail visible BELOW blob in frames 189-190 — this is the actual bullet path |

## Detection Pipeline Processing Order
```
grayscale → background subtraction → Laplacian deficit
                                   → temporal diff
                                   → intensity diff
                                   → Gabor phase change
         → 4-channel weighted sum (0.1/0.1/0.3/0.5)
         → 2D Haar DWT denoise (LL2 median sub + VisuShrink 0.5×)
         → Gabor directional filter bank weight (floor=0.2, sat=2.0)
         → clean_detection output
```

## Trajectory Pipeline Processing Order
```
Mode 0: Wait for FLIGHT state, save baseline detection
Mode 1: Accumulate diff_map for 5 frames
       → intensity cap at 80th percentile
       → structure tensor anisotropy weighting
       → PCA on weighted map → 3 trail axis points
       → quadratic fit → Mode 2
Mode 2: Follow quadratic prediction, integrate evidence
       (coefficients currently frozen — no refinement)
```
