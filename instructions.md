# DSP-Based Schlieren Trail Tracking: Complete Session Report

---

## 1. THE PROBLEM

A pre-recorded video, shot from a static tripod-mounted camera at 1080p/30fps, captures a landscape scene. Somewhere during the video, a near-sonic object (a bullet) is fired across the field of view. The object itself is small and difficult to see. However, the air disturbance behind the object creates a schlieren-like trail — a momentary blurring/refractive distortion of the background that is visible to the human eye.

The task: detect, track, and draw the flight path of this object using only the schlieren trail as the primary observable signal. The entire pipeline must be implemented in MATLAB's Simulink environment.

---

## 2. THE REQUIREMENTS

### 2.1 Core Functional Requirements

- Detect when the bullet is fired (onset detection)
- Track the schlieren trail across the full frame as the bullet travels ~800 meters downrange
- Estimate and draw the flight path overlaid on the original video
- Manage the tracking lifecycle through a state machine: WAITING → FLIGHT → LOST → IMPACT

### 2.2 DSP & Engineering Requirements

- The solution must be built with explicit DSP reasoning — not black-box toolbox blocks
- Every filter choice (type, order, cutoff frequencies) must be justified from signal properties
- Phase implications, delay, gain, and windowing effects must be documented at each stage
- Filter parameters should be derived from the video data itself, not hardcoded from assumptions
- Spatial, temporal, and frequency-domain analysis must be utilized
- STFT, 2D Wavelet Transform, Gabor filter banks, and spatiotemporal thresholding were requested
- Physics-guided prediction (quadratic ballistic trajectory) should fill gaps where the trail becomes invisible
- Stateflow should manage the tracking lifecycle

### 2.3 Video Characteristics (from Q&A)

| Parameter | Value |
|-----------|-------|
| Resolution | 1920×1080 (HD) |
| Frame rate | 30 fps |
| Camera | Static on tripod, minor vibrations |
| Range to object | ~800 meters |
| Object speed | Near-sonic (~Mach 0.8–1.2) |
| Trail angular width | ~45 degrees, rotating with trajectory |
| Flight duration | ~20–30 frames visible |
| Trail vs. object visibility | Trail is MORE visible than the object |
| Background | Static landscape, with mirage/heat shimmer effects |
| Toolboxes available | Computer Vision, Image Processing, DSP System, Signal Processing |

---

## 3. THE ORIGINAL PIPELINE DESIGN (Theoretical)

The initial design was a 10-stage pipeline, designed top-down from DSP first principles before seeing any actual video data:

### Stage 0: Video Ingestion & Frame Buffering
- `From Multimedia File` → RGB to Grayscale (explicit BT.709 formula, not a toolbox block) → uint8 to double → Circular frame buffer (16–32 frames)

### Stage 1: Static Background Estimation & Mirage Suppression
- Temporal median over K=60–120 pre-flight frames (robust to outliers, unlike mean)
- Background subtraction: R(x,y,t) = I(x,y,t) – B(x,y)
- Temporal bandpass filter: 4th-order Butterworth IIR, passband 4–12 Hz, to reject mirage (0.5–3 Hz) and sensor noise (near Nyquist)
- Phase correlation-based tripod vibration compensation

### Stage 2: Oriented Spatial Filter Bank
- 24-channel Gabor filter bank (8 orientations × 3 spatial frequency scales)
- Orientation-contrast map to detect directional disturbances

### Stage 3: Temporal-Frequency Analysis via STFT
- Per-pixel STFT with Hann window (N=16, hop=2)
- Trail-band energy extraction
- Energy onset detection for flight start

### Stage 4: 2D Discrete Wavelet Transform
- db4 wavelet, 2-level decomposition
- Spatiotemporal soft thresholding (VisuShrink)

### Stage 5: Spatiotemporal Fusion & ROI
- Multiplicative fusion of STFT energy, Gabor orientation contrast, and wavelet energy
- Otsu thresholding for trail segmentation

### Stage 6: Stateflow State Machine
- States: WAITING → FLIGHT → LOST → IMPACT

### Stage 7: Physics-Guided Kalman Filter
- 6-state ballistic model: [px, py, vx, vy, ax, ay]
- Adaptive measurement noise R scaled by detection confidence

### Stage 8: Video Overlay
- Path drawing, crosshair, predicted trajectory, status indicators

### Stages 9–10: Architecture summary and implementation sequence

This design was comprehensive but built entirely on assumptions about the signal. The implementation phase revealed that several assumptions were wrong.

---

## 4. WHAT WE ACTUALLY BUILT (Implementation Journey)

### 4.3 Sharpness Deficit (Structural Residual): The Correct Signal Representation

**The breakthrough:** Instead of subtracting pixel values, we compare local spatial structure. The schlieren trail blurs the background, reducing local high-frequency spatial energy. We detect this as a "sharpness deficit."

**Implementation:**
1. Compute the Laplacian (isotropic second derivative, a spatial highpass filter) of both the current frame and the background
2. Square and spatially smooth (11×11 boxcar) to get local Laplacian energy
3. Compute the ratio: `deficit = (energy_bg – energy_curr) / (energy_bg + ε)`

This ratio-based metric has a critical property: it naturally rejects row-correlated sensor artifacts (rolling shutter banding) because any systematic row bias appears in both numerator and denominator and cancels.

**Texture gating** was added to suppress unreliable detections in low-texture regions (sky, uniform areas) where the denominator is near zero and the ratio is noisy.

### 4.4 Gradient Coherence: Tried and Rejected

**Why it was tried:** The sharpness deficit worked well for near-field trail detection (strong blur, measurable energy loss) but failed at far-field (subtle sub-pixel displacements that do not reduce Laplacian energy — they just relocate it). The gradient coherence metric was introduced to detect pixel displacement by measuring how well the local gradient vector fields of the current frame and background align.

**Implementation:** Sobel gradients of both frames → normalized dot product of gradient fields, spatially averaged → coherence loss = 1 – coherence.

**Why it was rejected:** The Sobel Gy kernel spans 3 rows asymmetrically. The CMOS rolling shutter creates different row-wise exposure during flight (muzzle blast vibration changes sensor readout timing). This caused systematic gradient disagreement along entire horizontal bands, producing strong rectangular artifacts in the detection map. These artifacts were amplified by the z-score normalization (the noise floor was learned during WAITING when the row pattern was different). The gradient coherence metric was fundamentally vulnerable to rolling shutter in a way the Laplacian ratio was not, because the Sobel kernel's antisymmetric structure does not cancel additive row bias the way the symmetric Laplacian does.

**Insight:** The far-field detection problem cannot be solved by a more sensitive single-frame metric. The signal is below the single-frame noise floor regardless of what spatial operator is used. The solution is temporal: accumulate evidence over multiple frames along a predicted trajectory (see Section 4.6).

### 4.5 Mirage Noise Floor Estimation & Subtraction

**Problem:** Even with the sharpness deficit, the detection map showed pervasive mirage-induced speckle. The trail signal, especially at far range, was buried in it.

**Key insight:** In the raw pixel domain, mirage is a structural distortion (like schlieren) and cannot be subtracted. But in the detection map domain (sharpness deficit), mirage produces an approximately additive, stationary noise floor. This noise floor can be estimated and subtracted.

**Implementation (Welch's online algorithm):**
- During the first 20 frames after background_ready, accumulate per-pixel mean and variance of the sharpness deficit map
- After 20 frames, the noise floor is characterized: each pixel has its own mean (expected mirage level) and standard deviation
- During FLIGHT: compute z-score = (current_deficit – noise_mean) / noise_std
- Threshold: only pixels exceeding NOISE_MARGIN standard deviations above their local mirage floor are retained

This is a per-pixel Neyman-Pearson detector. Each pixel has its own adaptive threshold based on its learned null distribution (mirage only).

**Temporal differencing** was added as a complementary channel: |deficit(t) – deficit(t-1)| highlights regions where sharpness loss changed between frames. The trail moves fast (appears/disappears in 1–3 frames), mirage moves slowly (0.5–3 Hz). The first-order temporal difference is a highpass filter (H(z) = 1 – z⁻¹) that suppresses slow mirage and passes fast trail transients.

**Pre-filtering:** A 3-frame temporal average (boxcar FIR lowpass, cutoff ~10 Hz) was applied to the raw frames before computing the deficit, reducing sensor noise by ~2.4 dB while preserving both mirage and trail.

### 4.6 Trajectory-Guided Integration: The Far-Field Solution

**Problem:** Beyond a certain range (~400–600m), the schlieren distortion becomes sub-pixel. No single-frame metric can detect it. The trail is invisible in any individual detection map.

**The human perception insight:** The user observed that the human eye can track the trail to far range because the brain fits a ballistic arc from the initial visible portion and then "looks where it expects the trail to be," accumulating visual evidence over time along the predicted path. This is exactly a matched filter in the spatiotemporal domain.

**Implementation (two-phase approach):**

Phase 1 (near-field, Mode 1): Collect centroid measurements from the first 6 frames where the trail is strongly visible. Fit a quadratic trajectory: x(t) = ax·t² + bx·t + cx, y(t) = ay·t² + by·t + cy. This captures the ballistic (constant-acceleration) nature of the flight path.

Phase 2 (far-field, Mode 2): Project the trajectory forward. At each frame, sample the detection map along the predicted path. Accumulate evidence over the last 5 frames. Even sub-threshold single-frame signals become detectable when integrated coherently along the correct path. SNR improvement: √N = √5 = 2.24 = 3.5 dB.

**Online trajectory refinement:** When a detection is found near the predicted position, the measurement is incorporated and the quadratic is refitted using a sliding window of 6 most recent detections. This corrects for drag deceleration, wind, and initial fit errors.

### 4.7 State Machine

**Implementation:** A MATLAB Function block with persistent state variables, implementing:
- WAITING: dormant, accumulating noise floor statistics
- FLIGHT: trail detected, actively tracking
- LOST: trail temporarily invisible, prediction-only (with timeout)
- IMPACT: terminal state

**Adaptive onset detection:** The onset threshold is not hardcoded. After the noise floor is ready, the state machine accumulates frame_energy_scalar statistics for 60 additional frames to learn the baseline mean and standard deviation. The onset threshold is set to mean + 8·sigma. This prevents false triggers from mirage fluctuations.

### 4.8 Video Overlay

**Implementation:** A MATLAB Function block that draws on the original RGB frames:
- Historical path points (color-coded: green=FLIGHT, orange=LOST)
- Connecting lines between consecutive points
- Red crosshair at current predicted position
- Cyan dashed predicted future trajectory (from quadratic coefficients)
- Status indicator bar (color-coded by state)
- Confidence bar
- Binary frame counter
- Timeout countdown bar (LOST state only)

---

## 6. CURRENT SYSTEM STATE

### 6.1 Active Blocks

| Block | Function Name | Role |
|-------|--------------|------|
| Background | `temporal_median_background_v2` | Computes temporal median over 60 frames, outputs `background_ready` flag |
| Detection | `sharpness_detection_with_mirage_suppression_v2` | Laplacian deficit + mirage noise floor subtraction + temporal differencing |
| Trajectory | `trajectory_integrated_detection_v3` | Near-field centroid collection → quadratic fit → far-field trajectory-guided integration |
| State Machine | `flight_state_machine_v2` | Adaptive onset detection + WAITING/FLIGHT/LOST/IMPACT transitions |
| Path Buffer | `path_history_buffer_v1` | Accumulates trajectory points for overlay drawing |
| Overlay | `video_overlay_v2` | Draws path, crosshair, predicted trajectory, status bars on RGB video |
| Frame Counter | `frame_counter_v1` | Standalone persistent counter |

### 6.2 Removed / Obsolete Blocks

| Block | Reason for Removal |
|-------|--------------------|
| `temporal_bandpass` (Butterworth IIR) | Destroyed trail signal; mirage/trail spectral overlap made temporal filtering impossible |
| `adaptive_onset_and_filter` | Onset detection moved to state machine; temporal bandpass removed; ROI moved to trajectory integrator |
| `structural_residual_v2` (gradient coherence) | Rolling shutter vulnerability; amplified row-correlated sensor artifacts |
| Background subtraction (R = I – B) | Wrong model for schlieren (refractive distortion is not additive) |

---

## 7. KEY INSIGHTS AND LESSONS LEARNED

### 7.1 Schlieren is Not Additive
The single most important insight of the project. Every standard video processing technique (background subtraction, temporal filtering, change detection) assumes the signal is additive on top of a static background. Schlieren is a structural distortion — it warps the background, not adds to it. This required a fundamentally different detection approach: comparing local spatial structure (Laplacian energy ratio) rather than pixel values.

### 7.2 Assumptions Without Measurement Are Not DSP Engineering
The initial filter parameters (4 Hz lower cutoff, Butterworth passband) were derived from physics reasoning about object speed and mirage frequency. The actual video showed that these assumptions were wrong. The trail's temporal content extended well below 4 Hz. The correct approach is always: measure the signal first, then design the filter. This led to the data-driven calibration philosophy that now governs the entire pipeline.

### 7.3 The Far-Field Problem is Information-Theoretic
No spatial operator applied to a single frame can extract signal that is below the noise floor. The solution is not a better metric — it is temporal integration along a predicted trajectory. This is the matched filter principle: if you know the shape of the signal (the ballistic arc), correlate the data with that shape to accumulate SNR as √N.

### 7.4 Detection-Domain Processing vs. Signal-Domain Processing
Mirage cannot be subtracted in the pixel domain (it is a structural distortion, like schlieren). But in the detection domain (the sharpness deficit map), mirage produces an approximately additive, stationary noise floor that CAN be estimated and subtracted. The insight is that the correct domain for noise subtraction depends on the noise's statistical properties in that domain, not on intuition about the physical process.

### 7.5 Rolling Shutter is a Silent Killer
The CMOS rolling shutter created row-correlated artifacts that were invisible in the raw video but devastating in derivative-based detection maps. The Laplacian (symmetric kernel) survived because its ratio formulation cancels smooth row bias. The Sobel gradient (antisymmetric kernel) amplified it. Kernel symmetry determines susceptibility to row-correlated sensor artifacts.

### 7.6 System Synchronization Matters More Than Algorithm Quality
The most sophisticated detection algorithm is useless if the noise floor is learned from garbage data (because the background block had not finished computing yet). The `background_ready` gating flag — a trivial boolean — fixed a cascade of failures that no amount of algorithm tuning could have addressed.

### 7.7 Simulink-Specific Pitfalls
- MATLAB Coder infers types aggressively. A `complex` type from `zp2sos` propagates silently through the entire datapath.
- Downstream blocks back-propagate size constraints. A stale [2×2×3] cache in a video writer block caused errors that appeared to originate in the MATLAB Function block.
- Persistent variables and output ports are independent. Outputs must be explicitly assigned from persistent variables on every execution path.
- `buttord` should not be trusted for near-Nyquist transition bands. Always inspect the computed order before using it.

---

## 8. REMAINING WORK

1. **Validate centroid detection at the actual bullet flight frames (~190+).** The adaptive percentile-based centroid finder has not yet been tested on the real trail data.

2. **Tune confidence thresholds** (LOST_CONF_THRESH, REACQ_CONF_THRESH) based on measured confidence values during actual flight.

3. **Implement the Gabor oriented filter bank** (Stage 2 of the original design) to discriminate trail from mirage based on directional structure. This was designed but never implemented due to the focus on getting the detection foundation working first.

4. **Implement the 2D DWT** (Stage 4) for multiscale spatial analysis. Also designed but not yet implemented.

5. **Implement the Kalman filter** (Stage 7) to replace or supplement the least-squares quadratic fit. The Kalman filter provides uncertainty estimates and naturally handles measurement gaps.

6. **Verify the complete pipeline end-to-end** with correct video output (the RGB wiring and scaling issue needs final confirmation).
