# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project
ECE 541 DSP project: detect and track a near-sonic bullet's flight path using schlieren (refractive distortion) trail in 1080p/30fps video. Pure MATLAB pipeline (no Simulink blocks). All functions have `%#codegen` tags but run as regular MATLAB scripts.

## Running the Pipeline
```matlab
% From D:\documents\UalbertaCourses\ECE_541\project\claude\
main_pipeline   % runs everything, outputs to runs/<timestamp>/
```
Each run creates a timestamped folder under `runs/` containing: `output.avi`, `pipeline_log.mat`, `pipeline_log.csv`, `maps/` (detection map PNGs), and `pics/` (diagnostic figures).

Input video: `../longer.mp4` (1920x1080, 30fps, ~254 frames).

## Architecture

### Pipeline Order (main_pipeline.m)
Each frame passes through these stages sequentially:
1. **Background estimation** (`temporal_median_background.m`) — temporal median of first 60 frames
2. **Sharpness detection** (`sharpness_detection_with_mirage_suppression.m`) — 4-channel detection (Laplacian deficit, temporal diff, intensity diff, Gabor phase), z-score noise floor, then 2D Haar DWT denoising (`dwt2_enhance.m`, suppresses codec macroblock artifacts), then Gabor directional filter bank (`gabor_filter_bank.m`) weights the map to enhance linear trail features. Outputs `clean_detection` map (HxW double) and scalar `frame_energy`
3. **State machine** (`flight_state_machine.m`) — 4 states: WAITING(0) -> FLIGHT(1) -> LOST(2) -> IMPACT(3). Uses PREVIOUS frame's trajectory outputs (breaks algebraic loop). Adaptive onset threshold calibrated over 60 post-noise-floor frames
4. **Impact detection** (`impact_detection.m`) — independent parallel detector for bullet impact splash. ROI-based pixel count or full-frame energy
5. **Trajectory integration** (`trajectory_integrated_detection.m`) — Mode 0 (waiting), Mode 1 (near-field centroid collection + quadratic fit), Mode 2 (trajectory-guided integration). This is the core tracking block
6. **Path buffer** (`path_history_buffer.m`) — accumulates (x,y) points when `tracking_active=true`
7. **Video overlay** (`video_overlay.m`) — renders trajectory, state info, confidence bar onto output frames

### Data Flow
- Detection map: `clean_detection` (1080x1920 double, z-score based). Weights: 0.10*spatial + 0.10*temporal + 0.30*intensity + 0.50*phase, then Gabor directional filter bank multiplicative weighting
- State machine receives previous frame's trajectory outputs to avoid algebraic loop
- Trajectory block outputs quadratic coefficients: x(t) = ax*t^2 + bx*t + cx, y(t) = ay*t^2 + by*t + cy, where t = frame_number - t_ref
- All persistent state lives inside each function (no global variables)

### Post-Processing
- `save_log.m` — writes pipeline_log.mat and pipeline_log.csv
- `analyze_video.m` — generates diagnostic figures (system_diagnostic, trajectory_coefficients, flight_path, detection_maps, centroid_diagnostics)

## Key Signal Facts (from run 2026-03-30_12-11-38)
- Background ready: frame 60. Noise floor ready: frame 80
- Onset calibration completes at frame 140 (threshold = 7026.55)
- Bullet fires ~frame 184-185. FLIGHT declared frame 186
- Trail visible in detection maps from ~frame 186-210 (~25 frames)
- The trail appears as a semi-linear anomaly on the RIGHT side of the frame
- There is NO muzzle blast artifact in this video
- The bright blob at bottom-right (~x=1600, y=580) is the NEAR-FIELD trail entry point, NOT a separate artifact. Its apparent isolation from the rest of the trail is caused by camera focus discontinuity between frames 186-188. It IS part of the trail
- Trail extends from blob region (~1600,580) toward (~1750,350) — about 280px diagonal
- LOST at frame 220, IMPACT at frame 228
- Impact ROI configured at (1451, 589) with radius 100

## Known Artifacts
1. **H.264/H.265 codec macroblock artifacts** (frames 184-190): When the bullet fires, the abrupt scene change causes the video encoder to re-quantize coding tree units (16×16 macroblocks in H.264, up to 64×64 CTUs in H.265). Different rows of CTUs get different quality allocations, creating a rectangular region of altered pixel values in the decoded frame. This manifests as a bright horizontal band in the temporal diff channel (`td_cleaned`) and a broad rectangular elevation in the combined detection map. The artifact persists ~6 frames as the codec stabilizes. Suppressed by the 2D DWT enhancement (`dwt2_enhance.m`): the artifact maps to the LL2 approximation band (low-frequency, spatially smooth), while the trail maps to detail bands (high-frequency, directional). Attenuating LL2 removes the artifact.
2. **Persistent phase channel pattern**: The Gabor phase channel (`phase_cleaned`) shows a static bright arc in the upper-left (treeline/vegetation boundary) that is identical across all frames. This is a persistent bias from the phase z-score not fully normalizing high-variance textured regions. Not harmful to tracking (it's spatially separated from the trail) but contributes to visual clutter in detection maps.
3. **Trail discontinuity over snow-covered regions**: Comparing the detection map at frame 190 with the original RGB frame reveals that the trail breaks into a gap at the same location where the ground is covered in snow. In snow/uniform-texture regions, the Laplacian deficit, temporal diff, and intensity diff channels all fail — there is no background texture to lose sharpness from, no structured intensity to shift. The trail's refractive distortion is still physically present but invisible to texture-dependent detectors. The Gabor phase channel was added specifically to address this: it detects phase rotation of local spatial frequencies, which occurs even in low-texture regions where the schlieren bending of light paths alters the phase of faint residual structure (snow grain, subtle gradients). The phase channel's 0.50 weight reflects its critical role in bridging the trail across these texture-poor gaps.

## Current Status / Known Issues
1. **PCA trail axis extraction**: Implemented and working (see `spicy-noodling-horizon.md`). n_detections reaches 3 at frame 188, quadratic fit fires, Mode 2 active for ~27 frames.
2. **Trajectory over-extrapolation**: The quadratic fit extends past the visible trail (~280px) across the full frame. Needs quality gates (Phase 4).
3. **Impact detection**: Not triggering (splash signal too weak relative to baseline).

## Design Constraints
- The trail is a STATIC spatial structure that appears almost instantly (bullet crosses frame in <1ms at near-sonic speed). Do not design tracking algorithms that assume the detection "moves" frame-to-frame like a ball.
- Do not add "muzzle blast suppression" — there is no blast in this video.
- The bright blob at bottom-right is the trail entry point, not noise to be filtered.
