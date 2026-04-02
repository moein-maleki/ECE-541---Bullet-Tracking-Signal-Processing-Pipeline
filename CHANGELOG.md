# Changelog

Running log of changes made to the schlieren trail tracking pipeline.

---

## 2026-03-31

### Session 8e — Mexican Hat radial filter for blob suppression
- **trajectory_integrated_detection.m**:
  - Added `mexican_hat_filter` local function: 2D Mexican Hat (LoG) convolution tuned to the trail's characteristic radius. Profile: `w(r) = (1 - (r/r_opt)^2) * exp(-r^2/(2*r_opt^2))`. Features matching r_opt get positive response; wider features (blob) get strong negative response. Zero-mean kernel ensures uniform regions cancel. 2x downsampled. Negative output clipped to zero.
  - Added `estimate_trail_radius` local function: finds peak pixel in the masked map, measures 8-direction radial profile, returns the HWHM (half-width at half-maximum) as the adaptive radius. Clamped to [5, 20] pixels.
  - Applied after spatial mask, before PCA. Pipeline is now: intensity cap → morph line opening → spatial mask → Mexican Hat → PCA.
  - New parameters: `MHAT_R_DEFAULT=10`, `MHAT_R_MIN=5`, `MHAT_R_MAX=20`, `MHAT_DENSITY_PCTILE=0.50`.
  - Debug map `pca_post_weight` now shows the Mexican Hat output (the actual PCA input).

### Session 8d — Spatial mask for PCA
- **trajectory_integrated_detection.m**:
  - Added data-driven spatial mask after morphological filtering. Computes per-column signal sum, finds the peak column, masks to within `PCA_SPATIAL_MARGIN=400` pixels of it. This excludes treeline vegetation edges at the left side of the frame that survive the morph filter (>80px long) and contaminate PCA.
  - Debug map `pca_post_weight` now shows the masked output (morph + spatial mask) rather than raw morph output.
  - New parameter: `PCA_SPATIAL_MARGIN = 400`.

### Session 8c — Ideal path debug overlay
- **define_ideal_path.m** (NEW): Interactive tool — opens video frame 190, user clicks along the trail from near-field to far-field, saves `ideal_path.mat` with `ideal_x`, `ideal_y` arrays.
- **video_overlay.m**: Loads `ideal_path.mat` on first call (persistent). Draws magenta line + dots on every frame showing the user-defined ideal path.
- **main_pipeline.m**: Saves `debug_path_compare.png` to the run folder — captures one annotated frame 5 frames into active tracking for path comparison.

### Session 8b — Parameter tuning + FAR marker + scale grid
- **trajectory_integrated_detection.m**:
  - Increased `MORPH_LINE_LEN` from 40 to 80. At 40px, long vegetation edges (~50-100px) survived the morph opening and contaminated PCA. At 80px (40px half-res SE), only features >80px survive — the trail (~280px) passes easily; vegetation edges and blob (~50px) are killed.
  - Disabled `GRAVITY_PRIOR` (set to 0). The trail is nearly straight; forcing ay=0.4 added artificial downward curvature that pulled the far-field endpoint to the wrong location.
- **video_overlay.m**:
  - Added FAR-field endpoint marker: red circle (r=12) + "FAR" label at the trajectory evaluated at t=TRAIL_DURATION. Shows where Mode 2 prediction clamps.
  - Replaced 100px corner cross with full-frame 100px grid (yellow, 15% alpha blend).

### Session 8 — Morphological line filter replaces local orientation weighting
- **trajectory_integrated_detection.m**:
  - Replaced structure tensor anisotropy AND Gabor DC weighting with morphological line opening (`morph_line_open` local function). Both local orientation operators failed: structure tensor (elongation=2.43) and Gabor DC (elongation=1.989, PCA FAILED) cannot distinguish trail from vegetation edges/codec artifacts because all are locally directional. The trail's uniqueness is its **global linear extent** (~280px), which only a shape-fitting operator can test.
  - `morph_line_open`: erode+dilate with line SE at `MORPH_N_ANGLES=12` angles, keep per-pixel max. 2x downsampled for performance. `MORPH_LINE_LEN=40` pixels (20px at half-res). Features shorter than the line length are removed regardless of local orientation.
  - New parameters: `MORPH_LINE_LEN=40`, `MORPH_N_ANGLES=12`. Removed `ANISO_SMOOTH_K`, `PCA_GABOR_DC_SAT`.
  - Debug map `pca_aniso` now shows the morphological filter output.
  - `compute_anisotropy` kept as dead code for reference.
  - **Removed entry-point extrapolation** (was lines 386-441). This backward-extrapolated the quadratic to the frame edge, adding a virtual anchor point far outside the trail. With bx≈-53, this extended the trajectory ~35 pseudo-frames backward across the full frame from a ~280px trail. The PCA endpoints already define the trail bounds.
  - **Clamped Mode 2 prediction time** to `[0, TRAIL_DURATION]`. Previously `t = frame_count - t_ref` grew unbounded; now the prediction holds at the far-field endpoint after TRAIL_DURATION frames instead of extrapolating off-screen.
  - **Added fit RMSE computation** after the 3-point quadratic fit (informational; with 3 points and 3 coefficients, RMSE is near-zero).

---

## 2026-03-30 (continued)

### Session 7 — Anisotropy-weighted trail extraction
- **trajectory_integrated_detection.m**:
  - **Removed blob-first centroid detection** (`find_densest_cluster` no longer used for Mode 1 initialization). The blob detector favored bright isotropic features over the thin linear trail.
  - **New Mode 1 approach**: accumulate detection maps, then apply intensity capping (80th percentile) + structure tensor anisotropy weighting before PCA. The anisotropy weight = (λ1 - λ2) / (λ1 + λ2) from the smoothed gradient structure tensor. Thin linear features (trail) get weight ≈ 1; isotropic blobs get weight ≈ 0.
  - PCA now extracts all 3 centroids directly from the anisotropy-weighted map — no separate blob detection step.
  - Added local functions: `compute_anisotropy` (structure tensor eigenvalue ratio), `percentile_cap` (histogram-based), `box_smooth_local` (separable cumsum filter).
  - Reduced `TRAIL_THRESH_FRAC` from 0.25 to 0.15 and `MIN_TRAIL_PIXELS` from 200 to 100 to accommodate the thinner weighted trail.

### Session 6b — DWT tuning fix
- **dwt2_enhance.m**: LL2 attenuation (=0) was too aggressive — removed the trail along with the codec artifact. Replaced with **LL2 median subtraction**: subtracts the median of LL2 (captures uniform codec artifact) while preserving local peaks (trail). Also reduced VisuShrink threshold to 0.5× (THRESH_SCALE=0.5) to retain more trail detail in soft-thresholded bands. Debug output now shows the removed baseline (median portion) rather than the full original LL2.

### Session 6 — Phase 6: 2D DWT Codec Artifact Suppression
- **dwt2_enhance.m** (NEW):
  - 2-level manual Haar DWT decomposition (codegen-safe, no wavelet toolbox).
  - LL2 approximation band attenuated to 0 — removes low-frequency rectangular artifacts from H.264/H.265 macroblock quantization shifts during bullet onset.
  - VisuShrink soft thresholding on all detail bands: noise sigma from MAD of HH1, threshold = sigma * sqrt(2*log(N)), histogram-based median for codegen.
  - Input padded to multiple of 4 for clean 2-level decomposition.
  - Returns enhanced map + `debug_approx` (LL2-only reconstruction showing what was removed).
- **sharpness_detection_with_mirage_suppression.m**:
  - DWT applied BEFORE Gabor directional weighting (provides cleaner input to filter bank).
  - Added `debug_dwt_approx` output.
  - Processing order is now: 4-channel sum → DWT denoise → Gabor directional weight.
- **main_pipeline.m**:
  - Captures `debug_dwt_approx`, saves to `maps/dwt_approx/` during FLIGHT frames.
- **CLAUDE.md**:
  - Documented H.264/H.265 codec macroblock artifact phenomenon under "Known Artifacts".
  - Updated "Current Status / Known Issues" to reflect PCA working, trajectory over-extrapolation.

### Session 5 — Phase 3: Gabor Directional Filter Bank
- **gabor_filter_bank.m** (NEW):
  - 8-orientation × 2-scale (sigma=3,6) Gabor filter bank.
  - Computes directional contrast: `(max_orient_energy - mean_orient_energy) / (mean_orient_energy + eps)`.
  - High values = linear/directional features (trail); low values = isotropic (blobs, noise).
  - 2x downsampling for performance (block average down, nearest-neighbor up).
  - Codegen-safe: fixed kernel size (37×37), no dynamic allocation.
- **sharpness_detection_with_mirage_suppression.m**:
  - Added Gabor directional filter bank weighting after 4-channel combination.
  - Multiplicative weight: `floor + (1-floor) * min(dc/saturation, 1)` with `GABOR_FLOOR=0.2`, `GABOR_DC_SAT=2.0`.
  - Enhances linear trail features, suppresses isotropic blob/noise by up to 80%.
  - Added `debug_gabor_dc` output (directional contrast map).
- **main_pipeline.m**:
  - Captures `debug_gabor_dc` from detection function.
  - Saves Gabor DC maps to `maps/gabor_dc/` during FLIGHT frames.

---

## 2026-03-30

### Session 1 (12:00 - 13:00)
- **main_pipeline.m**: Added `diary` call to capture MATLAB console output to `console.md` in each run folder.

### Session 2 (13:00 - 15:30) — Phase 1: Trajectory fitting overhaul
- **trajectory_integrated_detection.m**:
  - Added PCA trail axis extraction (`find_trail_axis` function) using intensity-weighted covariance and closed-form 2x2 eigendecomposition.
  - Rewrote Mode 1: hybrid approach — immediate first centroid via center-surround blob detector (`find_densest_cluster`), then PCA for centroids #2 and #3.
  - Disabled onset cooldown (`ONSET_COOLDOWN_MAX = 0`) — no muzzle blast in this video; the bright blob at onset IS the trail entry.
  - Added baseline subtraction: stores pre-flight detection frame (`prev_detection` from Mode 0) and accumulates `max(clean_detection - baseline, 0)` for PCA.
  - Fixed PCA orientation: trail entry (t=0) = FARTHER from impact ROI, trail end (t=max) = CLOSER to impact.
  - Fixed baseline capture: uses previous frame's detection (frame 185), not current frame (186) which already contains the trail.
  - Centroid merge in fallback path now FREEZES positions (no running-average drift).
  - Added vertical weight prior for first centroid search (suppresses top-of-frame vegetation, boosts lower half where trail is expected). Applied only to temporary copy, not to PCA accumulation.
  - Added `MIN_BLOB_CONTRAST` threshold to reject weak noise before real anomaly forms.
  - Added new debug outputs: `debug_pca_elongation`, `debug_accum_count`.
- **main_pipeline.m**:
  - Captures new trajectory debug outputs and stores in log.
  - Tracks first centroid (blob) position and full centroid log for overlay.
- **save_log.m**: Added `pca_elongation` and `accum_count` CSV columns.
- **analyze_video.m**:
  - Marks PCA fire frame on n_detections plot.
  - Shows PCA centroid as cyan diamond on scatter plot.
  - Updated detection map montage to read from `maps/detection/` subfolder.

### Session 3 (15:00 - 17:30) — Detection pipeline debugging
- **sharpness_detection_with_mirage_suppression.m**:
  - Added debug outputs: `debug_raw_deficit` (before texture gating), `debug_tex_weight` (combined gate), `debug_pre_rowmed` (z-score before row median), `debug_tex_weight_lo`, `debug_tex_weight_hi`, `debug_post_rowmed` (z-score after row median), `debug_spatial_cleaned`, `debug_td_zscore`, `debug_td_cleaned`, `debug_intensity_cleaned`.
  - Disabled high-texture gate (`tex_weight_hi = ones(H,W)`) — it was either never triggering (with absolute-max normalization) or over-suppressing mid-texture trail regions (with 95th-percentile normalization). The z-score noise baseline already handles vegetation suppression.
  - User disabled low-texture gate too (`tex_weight_lo = ones(H,W)`) for experimentation.
  - Added documentation for `box_smooth` function (separable box filter via cumulative sums).
  - User changed detection weights from `0.25/0.45/0.30` to `0.1/0.1/0.8` (heavily favoring intensity channel).
- **video_overlay.m**:
  - Added yellow circle "BLOB" marker at first centroid position.
  - Added cyan X markers for all logged centroids (permanent, accumulate over frames).
  - Added green detection map overlay (boosts green channel proportional to `clean_detection` intensity).
  - Added `centroid_log_x/y/len` and `clean_detection` inputs.
  - Fixed `LOST_TIMEOUT_VIS` from 20 to 8 to match actual `LOST_TIMEOUT` in state machine.
- **main_pipeline.m**:
  - Organized debug maps into subfolders under `maps/`: `detection/`, `raw_deficit/`, `tex_weight/`, `tex_lo/`, `tex_hi/`, `pre_rowmed/`, `post_rowmed/`, `spatial_cleaned/`, `td_zscore/`, `td_cleaned/`, `int_cleaned/`.

### Comment/documentation audit
- Removed all incorrect "muzzle blast" references across `sharpness_detection_with_mirage_suppression.m`, `trajectory_integrated_detection.m`, `impact_detection.m`.
- Updated function headers in all modified files to document new parameters and outputs.
- Updated `flight_state_machine.m` header to document all input parameters.
- Updated `CLAUDE.md` with architecture docs, pipeline order, design constraints.

### Session 4 (18:00 - 20:00) — Detection pipeline tuning + Gabor phase channel
- **sharpness_detection_with_mirage_suppression.m**:
  - Disabled both texture gates (tex_hi and tex_lo set to ones by user) for experimentation.
  - User changed detection weights to intensity-heavy (0.1/0.1/0.8), then to intensity-only.
  - Added **Gabor phase change channel** (4th detection channel): uses cross-spectral analysis with Gabor filter pairs (cosine + sine) at two orientations (0° and 90°) to detect refractive phase shifts in low-texture regions (snow, uniform ground) where Laplacian and intensity channels fail.
  - Phase channel includes: Gabor convolution via `conv2`, cached background response, Welford noise baseline learning, z-scoring, row median subtraction.
  - Reverted clean_detection to 4-channel weighted sum: `0.20*spatial + 0.35*temporal + 0.25*intensity + 0.20*phase`.
  - Added `debug_phase_cleaned` output.
- **main_pipeline.m**: Added `maps/phase_cleaned/` subfolder and saving for phase channel debug maps.
