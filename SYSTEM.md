# SYSTEM.md — Schlieren Trail Detection Pipeline

Pure MATLAB pipeline that detects and tracks a near-sonic bullet's flight path using its schlieren (refractive distortion) trail in 1080p/30fps video. All functions carry `%#codegen` tags but run as regular MATLAB scripts.

**Input**: `../longer.mp4` (1920x1080, 30fps, ~254 frames).
**Output**: `runs/<timestamp>/` containing `output.avi`, logs, detection maps, and diagnostic figures.

---

## Pipeline Overview

Each frame passes through six sequential stages:

```
Frame (RGB)
  |
  v
1. Background Estimation          temporal_median_background.m
  |
  v
2. Sharpness Detection            sharpness_detection_with_mirage_suppression.m
  |  (4-channel detection -> DWT denoise -> Gabor directional weight)
  |  Output: clean_detection (HxW double), frame_energy_scalar
  |
  v
3. State Machine                  flight_state_machine.m
  |  (WAITING -> FLIGHT -> LOST -> IMPACT)
  |  Uses PREVIOUS frame's trajectory outputs to avoid algebraic loop
  |
  v
4. Impact Detection               impact_detection.m
  |  (independent parallel detector for bullet impact splash)
  |
  v
5. Trajectory Integration         trajectory_integrated_detection.m
  |  (Mode 0 -> Mode 1: PCA trail extraction -> Mode 2: trajectory tracking)
  |  Output: path_x, path_y, trajectory coefficients, confidence
  |
  v
6. Path Buffer + Video Overlay    path_history_buffer.m, video_overlay.m
```

All persistent state lives inside each function. No global variables.

---

## Stage 1: Background Estimation

**File**: `temporal_median_background.m`

Computes the pixel-wise temporal median of the first 60 frames. This produces a static background image representing the scene without the bullet. The `background_ready` flag stays false until all 60 frames are accumulated and the median is computed.

**Why temporal median**: The median is robust to occasional moving objects (e.g., wind-blown vegetation) that would corrupt a simple mean. 60 frames (~2 seconds) provides sufficient samples for a stable estimate.

---

## Stage 2: Sharpness Detection

**File**: `sharpness_detection_with_mirage_suppression.m`

Detects the schlieren trail through four independent channels, each exploiting a different physical signature of the refractive distortion. All channels produce z-scored maps (standard deviations above a learned noise baseline), making them scale-comparable.

### 2.1 Detection Channels

| Channel | Weight | What it detects | How |
|---------|--------|----------------|-----|
| Spatial (Laplacian deficit) | 0.10 | Loss of texture sharpness where refraction blurs the background | Local Laplacian energy of current frame minus background Laplacian; deficit = background is sharper |
| Temporal diff | 0.10 | Frame-to-frame changes in sharpness | Difference of consecutive Laplacian maps (3-frame sliding window) |
| Intensity diff | 0.30 | Brightness shift from refraction bending light paths | Absolute difference between current and background intensity |
| Gabor phase | 0.50 | Refractive phase rotation of local spatial frequencies | Cross-spectral Gabor analysis (cos+sin pairs at 0 and 90 deg, sigma=4, f0=0.1 cyc/px). Detects phase shifts even in low-texture regions (snow, uniform ground) where the other three channels fail |

**Why these weights**: The phase channel carries the heaviest weight (0.50) because it is the only channel that works in low-texture regions where the trail crosses snow-covered ground. The Laplacian and temporal channels fail there because there is no background texture to lose sharpness from. Intensity contributes 0.30 as a reliable mid-texture detector. Spatial and temporal are at 0.10 each — they provide useful signal in textured areas but are noisy and less reliable overall.

### 2.2 Noise Baseline and Z-Scoring

Each channel maintains a per-pixel running mean and variance (Welford's algorithm) over the first 20 post-background frames (frames 61-80). The noise floor captures the normal variation when no bullet is present. Each channel's output is z-scored: `(value - mean) / sqrt(variance)`, then clipped to positive values. Row-median subtraction removes row-correlated sensor/codec artifacts.

### 2.3 DWT Denoising

**File**: `dwt2_enhance.m`

Applied to the 4-channel weighted sum before Gabor directional weighting. 2-level manual Haar DWT (codegen-safe, no wavelet toolbox):

- **LL2 median subtraction**: The H.264/H.265 codec re-quantizes macroblocks when the bullet fires (frames 184-190), creating rectangular bright regions in the detection map. These map to the LL2 approximation band (low-frequency, spatially smooth). Subtracting the LL2 median removes this uniform elevation while preserving local peaks (trail features that exceed the background).
- **VisuShrink soft thresholding** on all 6 detail bands: Noise sigma estimated from MAD of the finest diagonal band (HH1). Threshold = 0.5x the full VisuShrink threshold — softer than standard to retain trail detail in the detail bands.

**Why DWT before Gabor**: The Gabor directional filter bank responds to any oriented structure, including codec macroblock edges. Removing these artifacts first ensures the filter bank enhances only real scene features.

### 2.4 Gabor Directional Filter Bank

**File**: `gabor_filter_bank.m`

8 orientations (0 to 157.5 deg, 22.5 deg steps) x 2 scales (sigma=3, sigma=6). For each orientation, the bank computes the total energy (Re^2 + Im^2) across both scales. The directional contrast is:

```
DC = (max_orient_energy - mean_orient_energy) / (mean_orient_energy + eps)
```

This is applied as a multiplicative weight on `clean_detection`:

```
weight = 0.2 + 0.8 * min(DC / 2.0, 1.0)
```

- Linear/directional features (trail): DC >= 2.0, weight = 1.0 (full pass)
- Isotropic features (blobs, noise): DC ~ 0, weight = 0.2 (80% suppression)

The 0.2 floor prevents complete zeroing of potentially useful signal. 2x downsampling (block average) is used for performance; result is upsampled by nearest-neighbor.

---

## Stage 3: State Machine

**File**: `flight_state_machine.m`

Four states: `WAITING(0) -> FLIGHT(1) -> LOST(2) -> IMPACT(3)`.

### Onset Detection

After the noise floor is ready (frame 80), the state machine calibrates an onset threshold over 60 frames (80-140). It accumulates `frame_energy_scalar` values to compute mean and std, then sets the threshold at `mean + 8*std`. FLIGHT is declared when frame energy exceeds this threshold for 3 consecutive frames (debounce).

### State Transitions

| From | To | Condition |
|------|----|-----------|
| WAITING | FLIGHT | frame_energy > onset_threshold for 3 frames |
| FLIGHT | LOST | confidence < 0.02 for 5 frames AND flight_frame_count > 15 (grace period) |
| LOST | FLIGHT | confidence > 0.05 for 2 frames (reacquisition) |
| LOST | IMPACT | lost for 8 consecutive frames OR prediction exits frame |

The 15-frame grace period at FLIGHT onset prevents premature LOST transitions during PCA accumulation and initial fit establishment.

**Why previous-frame inputs**: The state machine receives the previous frame's trajectory outputs (tracking_active, confidence, path_x, path_y) to avoid an algebraic loop where the state machine and trajectory block depend on each other's current-frame output.

---

## Stage 4: Impact Detection

**File**: `impact_detection.m`

Independent parallel detector for bullet impact (dirt/debris splash). Runs alongside the trajectory integrator but does not depend on it.

**ROI mode** (configured at impact_x=1451, impact_y=589, radius=100): Counts pixels in the ROI where background-diff exceeds 25. Learns a baseline count during calm frames. Triggers when count spikes by 2x over baseline. Splash must persist for 2 frames to lock.

**Status**: Not currently triggering — the splash signal is too weak relative to baseline in this video.

---

## Stage 5: Trajectory Integration

**File**: `trajectory_integrated_detection.m`

The core tracking block. Operates in three modes:

### Mode 0: Waiting

Active while state machine is in WAITING. Stores the current detection map as `prev_detection` for temporal differencing. On transition to FLIGHT, captures the previous frame's detection as the pre-flight baseline for background subtraction.

### Mode 1: Near-Field Detection (PCA Trail Axis Extraction)

Accumulates baseline-subtracted detection maps for 5 frames after FLIGHT onset. On frame 5 of accumulation (typically frame 188):

1. **Intensity cap**: Cap the accumulated map at the 80th percentile. This flattens the bright blob (near-field trail entry) to the same level as the dimmer trail body, preventing PCA from being pulled toward the blob.

2. **Morphological line opening**: Apply morphological opening (erode + dilate) with a line structuring element at 12 angles (0 to 165 deg, 15 deg steps). Line length = 80px full-res (operated at 2x downsampled for performance, so 21-element SE at half-res). The per-pixel max across all angles is kept.

   This removes features shorter than ~80px: blobs (~30-50px diameter, circular — no 80px line fits inside regardless of angle), codec block edges, and noise speckles. Only elongated features matching the line length survive. The trail (~280px long) survives easily at the correct angle.

   **Why morphological filtering**: Local orientation operators (structure tensor, Gabor directional contrast) fail because the accumulated diff map contains widespread directional signal from vegetation edges, codec artifacts, and noise — all locally directional. The trail's distinguishing property is its global linear extent (~280px), not its local orientation. Morphological opening with a line SE tests global shape fit: a feature must accommodate the entire line length to survive.

3. **Spatial mask**: Find the column with the highest total signal in the morph-filtered map. Mask to within ±400px of that column. This is a data-driven exclusion of distant contamination (e.g., treeline vegetation at the left of frame that has edges long enough to survive the morph filter) without hard-coding a frame region.

4. **Mexican Hat radial filter**: Convolve with a 2D Mexican Hat (Laplacian of Gaussian) kernel tuned to the trail's characteristic width. The profile is `w(r) = (1 - (r/r_opt)^2) * exp(-r^2 / (2*r_opt^2))`. Features matching `r_opt` in width get positive response; features wider than `r_opt` (the blob, ~30-50px) get strong negative response and are suppressed below zero. The kernel is zero-mean, so uniform regions cancel.

   The adaptive radius `r_opt` is estimated from the peak region of the masked map: find the brightest pixel, measure the radial intensity profile in 8 directions, compute the half-width at half-maximum (HWHM). Clamped to [5, 20] pixels (default 10).

   **Why Mexican Hat after morph**: The morph filter tests length ("must be long enough"), the spatial mask tests location ("must be near the signal"), and the Mexican Hat tests width ("must be thin enough"). Each removes a different class of contamination. The blob survives the morph filter (it's long enough when slightly elongated) and the spatial mask (it's in the right region), but fails the Mexican Hat (it's too wide).

   2x downsampled for performance. Negative output clipped to zero before PCA.

5. **PCA**: Threshold the filtered map at 15% of max. Compute intensity-weighted centroid and 2x2 covariance matrix. Closed-form eigendecomposition gives the principal axis. Check elongation (lambda1/lambda2 >= 2.0). Project bright pixels onto the axis, find 5th/95th percentile endpoints via histogram. Return 3 points: near-field endpoint, centroid, far-field endpoint.

6. **Quadratic fit**: Fit x(t) and y(t) as quadratics through the 3 PCA points. Pseudo-time: t=0 (near-field), t=10 (centroid), t=20 (far-field). Gravity prior disabled (ay left as-is; trail is nearly straight). Transition to Mode 2.

### Mode 2: Trajectory-Guided Tracking

Evaluates the frozen quadratic coefficients at each frame:

```
t = clamp(frame_count - t_ref, 0, TRAIL_DURATION)
pred_x = ax*t^2 + bx*t + cx
pred_y = ay*t^2 + by*t + cy
```

The time is clamped to `[0, TRAIL_DURATION]` so the prediction stays within the PCA data range. After TRAIL_DURATION frames, the prediction holds at the far-field endpoint rather than extrapolating into empty space.

**Trajectory-aligned integration**: Maintains a 5-frame evidence buffer. For each buffered frame, samples a 30px-wide perpendicular strip along the trajectory and accumulates into an integrated map. Confidence is the mean integrated-map value in a 15px radius around the predicted position.

**Centroid refinement** in Mode 2 is currently disabled. The initial Mode 1 fit is the only fit that runs. The coefficients are frozen for the duration of tracking.

### Trajectory Freeze

Once the state machine reaches IMPACT (state 3), trajectory coefficients are locked and `tracking_active` is set to false, stopping path accumulation. If impact detection provides a location, a one-shot refit incorporates it as the terminal point.

---

## Stage 6: Output

### Path History Buffer

**File**: `path_history_buffer.m`

Fixed-size ring buffer (max 120 entries). Appends (x, y, confidence, state) each frame when `tracking_active` is true. Provides the full path history for video overlay rendering.

### Video Overlay

**File**: `video_overlay.m`

Renders onto each output frame:
- **Detection map overlay**: Green channel intensity proportional to `clean_detection`
- **Path history**: Green connecting lines between consecutive path points, white dot markers
- **Centroid log**: Cyan X markers at all historical centroid positions
- **Blob marker**: Yellow circle at the first detected centroid position
- **Trajectory curve**: Cyan dashed dots projected forward from current position
- **FAR endpoint marker**: Red circle (r=12) + "FAR" label at the trajectory's far-field endpoint (t=TRAIL_DURATION)
- **Impact marker**: Magenta X at impact location (when detected)
- **State info**: Text showing frame number, state duration, confidence
- **Confidence bar**: Horizontal bar proportional to confidence
- **Scale grid**: Yellow lines at 100px intervals across the full frame (15% alpha blend)
- **Ideal path overlay**: Magenta line + dots from `ideal_path.mat` (if exists), loaded once on first call via persistent state

---

## Post-Processing

### Log Export

**File**: `save_log.m`

Writes `pipeline_log.mat` (full MATLAB workspace) and `pipeline_log.csv` with per-frame columns: frame, state, frame_energy, onset_threshold, confidence, path_x/y, detection_max/mean, flags, trajectory coefficients, centroid debug fields (n_detections, mode, merged, rejected, change_max, pca_elongation, accum_count).

### Diagnostic Analysis

**File**: `analyze_video.m`

Generates five diagnostic figures saved to `pics/`:
- `system_diagnostic.png`: Energy vs onset threshold, state machine timeline, confidence, predicted position, detection statistics
- `trajectory_coefficients.png`: ax/bx/cx/ay/by/cy over time
- `flight_path.png`: Detected path in image coordinates with FLIGHT markers and raw centroids
- `detection_maps.png`: Montage of 20 detection maps around flight onset
- `centroid_diagnostics.png`: Centroid buffer count, change map peaks, centroid positions (color-coded by frame), merge/reject events

---

## Signal Flow Summary

```
grayscale frame
    |
    +---> Laplacian deficit  ---> z-score ---> row-med-sub ---> spatial_cleaned  (x0.10)
    +---> temporal diff      ---> z-score ---> row-med-sub ---> td_cleaned       (x0.10)
    +---> intensity diff     ---> z-score ---> row-med-sub ---> intensity_cleaned (x0.30)
    +---> Gabor phase change ---> z-score ---> row-med-sub ---> phase_cleaned    (x0.50)
    |
    v
4-channel weighted sum
    |
    v
2D Haar DWT denoise (LL2 median sub + VisuShrink 0.5x)
    |
    v
Gabor directional filter bank weight (floor=0.2, sat=2.0)
    |
    v
clean_detection  -------> state machine (frame_energy_scalar)
    |                  |
    |                  +-> impact detection
    v
trajectory integration
    |
    +---> Mode 1: accumulate 5 frames
    |       |-> intensity cap (80th pctile)
    |       |-> morphological line opening (80px, 12 angles)
    |       |-> spatial mask (peak column ± 400px)
    |       |-> Mexican Hat radial filter (adaptive r_opt)
    |       |-> PCA -> 3 trail axis points
    |       |-> quadratic fit -> Mode 2
    |
    +---> Mode 2: evaluate clamped quadratic
    |       |-> trajectory-aligned evidence integration
    |       |-> confidence estimation
    v
path buffer -> video overlay -> output.avi
```

---

## Key Parameters

| Parameter | Value | Location | Purpose |
|-----------|-------|----------|---------|
| Background frames | 60 | temporal_median_background | Frames for median background |
| Noise baseline frames | 20 | sharpness_detection | Frames for z-score baseline |
| Onset calibration frames | 60 | flight_state_machine | Frames to learn energy threshold |
| Onset K-sigma | 8 | flight_state_machine | Onset sensitivity |
| Channel weights | 0.1/0.1/0.3/0.5 | sharpness_detection | Spatial/temporal/intensity/phase |
| DWT thresh scale | 0.5 | dwt2_enhance | Softer VisuShrink |
| Gabor DC floor | 0.2 | sharpness_detection | Min weight for isotropic features |
| Gabor DC saturation | 2.0 | sharpness_detection | DC value for full weight |
| PCA accum frames | 5 | trajectory_integrated | Frames to accumulate for PCA |
| Morph line length | 80 | trajectory_integrated | SE length for morphological filter |
| Morph angles | 12 | trajectory_integrated | Angles tested (15 deg steps) |
| Spatial margin | 400 | trajectory_integrated | X-range around peak column for mask |
| MHat r_default | 10 | trajectory_integrated | Default Mexican Hat radius (px) |
| MHat r_min/max | 5/20 | trajectory_integrated | Adaptive radius bounds |
| MHat density pctile | 0.50 | trajectory_integrated | HWHM threshold for radius estimate |
| Intensity cap pctile | 0.80 | trajectory_integrated | Percentile cap before morph |
| Trail elongation min | 2.0 | trajectory_integrated | PCA eigenvalue ratio threshold |
| Trail duration | 20 | trajectory_integrated | Pseudo-time span for trajectory |
| Gravity prior | 0 | trajectory_integrated | Disabled — trail is straight |
| LOST timeout | 8 | flight_state_machine | Frames before LOST -> IMPACT |
| Flight grace | 15 | flight_state_machine | No LOST during first N flight frames |
| Impact ROI | (1451, 589) r=100 | main_pipeline | Configured impact search area |

---

## File Inventory

| File | Role |
|------|------|
| `main_pipeline.m` | Top-level frame loop, I/O, debug map saving |
| `temporal_median_background.m` | Background estimation (60-frame median) |
| `sharpness_detection_with_mirage_suppression.m` | 4-channel detection + DWT + Gabor weighting |
| `gabor_filter_bank.m` | 8-orient x 2-scale directional contrast |
| `dwt2_enhance.m` | 2-level Haar DWT denoising |
| `flight_state_machine.m` | 4-state machine with adaptive onset |
| `impact_detection.m` | Impact splash detector (ROI-based) |
| `trajectory_integrated_detection.m` | PCA trail extraction + quadratic fit + tracking |
| `path_history_buffer.m` | Fixed-size path accumulator |
| `video_overlay.m` | Frame annotation and rendering |
| `save_log.m` | CSV/MAT log export |
| `analyze_video.m` | Diagnostic figure generation |
| `select_impact_roi.m` | Interactive ROI selection utility |
| `define_ideal_path.m` | Interactive ideal path definition, saves ideal_path.mat |
| `rgb_debug.m` | RGB debug utility |

---

## Debug Map Outputs

Saved to `runs/<timestamp>/maps/<subfolder>/` during FLIGHT frames:

| Subfolder | Content |
|-----------|---------|
| `detection/` | Final `clean_detection` map |
| `raw_deficit/` | Laplacian deficit before texture gating |
| `tex_weight/` | Combined texture gate mask |
| `tex_lo/`, `tex_hi/` | Low/high texture gate components |
| `pre_rowmed/`, `post_rowmed/` | Z-score before/after row median subtraction |
| `spatial_cleaned/` | Spatial channel after noise margin |
| `td_zscore/`, `td_cleaned/` | Temporal diff z-score and cleaned |
| `int_cleaned/` | Intensity channel after noise margin |
| `phase_cleaned/` | Gabor phase channel after noise margin |
| `gabor_dc/` | Gabor filter bank directional contrast |
| `dwt_approx/` | DWT LL2-only reconstruction (removed content) |
| `pca_pre_weight/` | Intensity-capped accumulated map (morph input) |
| `pca_aniso/` | Morphological line filter output |
| `pca_post_weight/` | Final PCA input (same as morph output) |
