# ECE 541 — Schlieren Bullet Trail Detection

A pure MATLAB DSP pipeline that detects and tracks a near-sonic bullet's flight path using its schlieren (refractive distortion) trail in 1080p/30fps video.

## The Problem

When a bullet travels at near-sonic speeds, it compresses the air ahead of it and creates a trailing pressure wave. This pressure gradient causes refractive index variations in the air — a schlieren effect — that subtly distorts the background scene as viewed by a camera. The distortion is faint: a few pixels of spatial shift, slight blurring of background texture, and minor intensity changes along the bullet's path.

The trail appears almost instantaneously (the bullet crosses the full 1920px frame in under 1ms) and persists as a static spatial structure for roughly 25 frames (~0.8 seconds) before dissipating. It is not a moving object — it is a fading atmospheric imprint.

The challenge is detecting this trail against a cluttered outdoor scene (vegetation, snow, dirt berms) captured with a consumer action camera, compressed with H.264/H.265 lossy encoding that introduces its own spatial artifacts.

## The Pipeline

Each frame passes through six sequential stages:

```
RGB Frame
  -> Background Estimation (60-frame temporal median)
  -> Sharpness Detection (4-channel: Laplacian deficit, temporal diff,
                          intensity diff, Gabor phase change)
     -> 2D Haar DWT denoising (codec artifact suppression)
     -> Gabor directional filter bank (linear feature enhancement)
  -> State Machine (WAITING -> FLIGHT -> LOST -> IMPACT)
  -> Impact Detection (ROI-based splash detector)
  -> Trajectory Integration
     -> Mode 1: Accumulate 5 frames of baseline-subtracted detection
        -> Intensity cap (80th percentile)
        -> Morphological line opening (80px SE, 12 angles)
        -> Spatial mask (peak column +/- 400px)
        -> Mexican Hat radial filter (adaptive width-based suppression)
        -> PCA trail axis extraction -> 3-point quadratic fit
     -> Mode 2: Evaluate clamped quadratic trajectory
  -> Video Overlay + Path History
```

### Detection Channels

The detection block fuses four independent measurements, each exploiting a different physical signature of the refractive distortion:

| Channel | Weight | What it detects |
|---------|--------|-----------------|
| Laplacian deficit | 0.10 | Loss of texture sharpness where refraction blurs the background |
| Temporal diff | 0.10 | Frame-to-frame sharpness changes at trail onset |
| Intensity diff | 0.30 | Brightness shift from refraction bending light paths |
| Gabor phase | 0.50 | Phase rotation of local spatial frequencies — works in low-texture regions (snow) where the other channels fail |

All channels are z-scored against a learned noise baseline, making them scale-comparable across frames.

### Trail Extraction

The core challenge is extracting the thin diagonal trail (~3-8px wide, ~280px long) from an accumulated detection map contaminated by vegetation edges, codec artifacts, and a bright blob at the trail entry point. The pipeline applies four filters in sequence, each removing a different class of contamination:

1. **Morphological line opening** — removes features shorter than 80px (tests global linear extent)
2. **Spatial mask** — data-driven column mask around the peak signal (removes distant vegetation)
3. **Mexican Hat radial filter** — Laplacian of Gaussian tuned to trail width (suppresses features wider than the trail)
4. **PCA** — extracts the principal axis from the surviving signal, produces 3 seed points for a quadratic trajectory fit

### DSP Components

- **2D Haar DWT** (2-level, manual implementation) — LL2 median subtraction removes codec macroblock artifacts; VisuShrink soft thresholding denoises detail bands
- **Gabor filter bank** (8 orientations x 2 scales) — directional contrast weighting enhances linear features over isotropic blobs
- **Gabor phase channel** — cross-spectral analysis detects refractive phase shifts in low-texture regions
- **Morphological operations** — codegen-safe manual erosion/dilation with line structuring elements
- **Mexican Hat wavelet** — 2D LoG convolution for scale-selective feature extraction

## Current Status

The pipeline successfully:
- Detects the trail onset (FLIGHT declared at frame 183-186)
- Extracts a PCA trail axis from the accumulated detection map
- Fits a quadratic trajectory and tracks for ~67 frames
- Confines the detected path to the correct region of the frame (~340px X range)

The pipeline struggles with:
- **Blob dominance in PCA** — the bright trail entry blob (~30-50px) has more intensity than the thin trail body, pulling the PCA centroid away from the true trail axis
- **Impact detection** — splash signal too weak relative to baseline
- **Mode 2 trajectory refinement** — coefficients are frozen after initial fit

## Running

```matlab
cd D:\documents\UalbertaCourses\ECE_541\project\claude
main_pipeline   % runs everything, outputs to runs/<timestamp>/
```

Input video: `../longer.mp4` (1920x1080, 30fps, ~254 frames). Not included in the repo.

Each run produces: `output.avi`, `pipeline_log.mat`, `pipeline_log.csv`, `maps/` (detection map PNGs per stage), `pics/` (diagnostic figures), and `debug_path_compare.png`.

### Debug Tools

```matlab
define_ideal_path      % click on video frame to mark ground-truth trail
% then run main_pipeline — ideal path appears as magenta overlay on every frame
```

## Files

| File | Purpose |
|------|---------|
| `main_pipeline.m` | Top-level frame loop |
| `temporal_median_background.m` | 60-frame median background |
| `sharpness_detection_with_mirage_suppression.m` | 4-channel detection + DWT + Gabor |
| `gabor_filter_bank.m` | 8x2 directional contrast |
| `dwt2_enhance.m` | 2-level Haar DWT denoising |
| `flight_state_machine.m` | 4-state tracker (WAITING/FLIGHT/LOST/IMPACT) |
| `impact_detection.m` | ROI-based impact splash detector |
| `trajectory_integrated_detection.m` | PCA + quadratic fit + Mode 2 tracking |
| `path_history_buffer.m` | Fixed-size path accumulator |
| `video_overlay.m` | Frame annotation and rendering |
| `save_log.m` | CSV/MAT log export |
| `analyze_video.m` | Diagnostic figure generation |
| `define_ideal_path.m` | Interactive ground-truth path tool |
| `select_impact_roi.m` | Interactive impact ROI selection |

## Documentation

- `SYSTEM.md` — Complete technical description of the current system (always up-to-date)
- `CHANGELOG.md` — Chronological log of all changes
- `SESSION_CONTEXT.md` — Session state, known issues, and future plans
- `CLAUDE.md` — Project instructions for Claude Code
