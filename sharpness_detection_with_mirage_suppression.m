function [clean_detection, frame_energy_scalar, noise_floor_ready, ...
          debug_raw_deficit, debug_tex_weight, debug_pre_rowmed, ...
          debug_tex_weight_lo, debug_tex_weight_hi, ...
          debug_post_rowmed, debug_spatial_cleaned, ...
          debug_td_zscore, debug_td_cleaned, ...
          debug_intensity_cleaned, ...
          debug_phase_cleaned, ...
          debug_gabor_dc, ...
          debug_dwt_approx] = ...
    sharpness_detection_with_mirage_suppression(current_frame, ...
                                                    background_frame, ...
                                                    background_ready)
%#codegen
% SHARPNESS_DETECTION_WITH_MIRAGE_SUPPRESSION
%
% Detects schlieren trail via four channels: Laplacian sharpness deficit,
% temporal differencing, intensity difference, and Gabor phase change.
% Each channel is z-scored against a learned noise baseline and
% row-median subtracted to remove row-correlated sensor artifacts.
% The phase channel uses cross-spectral Gabor analysis to detect
% refractive distortion in low-texture regions (snow, uniform ground).
%
% Inputs:
%   current_frame   : HxW double, grayscale frame
%   background_frame: HxW double, temporal median background
%   background_ready: logical, whether background is valid
%
% Outputs:
%   clean_detection : HxW double, combined detection map (z-score based)
%   frame_energy_scalar: double, mean positive z-score of spatial channel
%   noise_floor_ready: logical, noise baseline calibrated (after 20 frames)
%   debug_raw_deficit: HxW, Laplacian deficit BEFORE texture gating
%   debug_tex_weight : HxW, texture gate mask (0=suppressed, 1=pass)
%   debug_pre_rowmed : HxW, z-score BEFORE row median subtraction
%   debug_tex_weight_lo: HxW, low-texture gate (suppresses sky/uniform)
%   debug_tex_weight_hi: HxW, high-texture gate (disabled — all ones)
%   debug_post_rowmed  : HxW, z-score AFTER row median subtraction
%   debug_spatial_cleaned: HxW, spatial channel after noise margin
%   debug_td_zscore    : HxW, temporal diff z-score (after row median)
%   debug_td_cleaned   : HxW, temporal channel after noise margin
%   debug_intensity_cleaned: HxW, intensity channel after noise margin
%   debug_phase_cleaned: HxW, Gabor phase channel after noise margin
%   debug_gabor_dc     : HxW, Gabor filter bank directional contrast
%   debug_dwt_approx   : HxW, DWT LL2-only reconstruction (low-freq
%                         content removed — shows codec block artifacts)
%
% Does NOTHING until background_ready is true. Noise floor estimation
% begins only after background_ready = true.

% ===================== PARAMETERS =====================
MIN_NOISE_FRAMES = 20;
TEMPORAL_AVG_LEN = 3;
K_SPATIAL        = 5;
NOISE_MARGIN     = 2.0;
% TEXTURE_UPPER not used — high-texture gate disabled (see below).

% Gabor phase channel parameters
GABOR_SIGMA  = 4;     % Gaussian envelope std dev (pixels)
GABOR_F0     = 0.1;   % center frequency (cycles/pixel, wavelength=10px)
GABOR_HW     = 12;    % kernel half-width (3*sigma)

% Gabor directional filter bank weighting (Phase 3)
% Applied after 4-channel combination to enhance linear trail features
% and suppress isotropic blobs/noise.
GABOR_DC_SAT = 2.0;   % directional contrast saturation (DC >= this → weight=1)
GABOR_FLOOR  = 0.2;   % minimum weight for isotropic regions (prevents zeroing)

[H, W] = size(current_frame);
win_area = (2*K_SPATIAL+1)^2;

% ===================== PERSISTENT STATE =====================
persistent frame_buf buf_ptr buf_filled
persistent noise_mean noise_var noise_count noise_ready
persistent prev_deficit
persistent intensity_noise_mean intensity_noise_var
persistent le_bg_cached bg_le_computed le_floor_cached
persistent phase_noise_mean phase_noise_var
persistent phase_bg_h_cos phase_bg_h_sin phase_bg_v_cos phase_bg_v_sin phase_bg_computed

% ===================== INITIALIZATION =====================
if isempty(frame_buf)
    frame_buf   = zeros(H, W, TEMPORAL_AVG_LEN);
    buf_ptr     = int32(1);
    buf_filled  = false;

    noise_mean  = zeros(H, W);
    noise_var   = zeros(H, W);
    noise_count = int32(0);
    noise_ready = false;

    prev_deficit = zeros(H, W);

    intensity_noise_mean = zeros(H, W);
    intensity_noise_var  = zeros(H, W);

    le_bg_cached   = zeros(H, W);
    bg_le_computed = false;
    le_floor_cached = 0;

    phase_noise_mean = zeros(H, W);
    phase_noise_var  = zeros(H, W);
    phase_bg_h_cos   = zeros(H, W);
    phase_bg_h_sin   = zeros(H, W);
    phase_bg_v_cos   = zeros(H, W);
    phase_bg_v_sin   = zeros(H, W);
    phase_bg_computed = false;
end

% =====================================================================
% GATE: DO NOTHING UNTIL BACKGROUND IS VALID
% =====================================================================
% Before background_ready, output zeros. Do not accumulate noise
% statistics. Do not update the temporal average buffer.
% This is the critical fix: the entire block is dormant until the
% median background exists.

debug_raw_deficit    = zeros(H, W);
debug_tex_weight     = zeros(H, W);
debug_pre_rowmed     = zeros(H, W);
debug_tex_weight_lo  = zeros(H, W);
debug_tex_weight_hi  = zeros(H, W);
debug_post_rowmed    = zeros(H, W);
debug_spatial_cleaned = zeros(H, W);
debug_td_zscore      = zeros(H, W);
debug_td_cleaned     = zeros(H, W);
debug_intensity_cleaned = zeros(H, W);
debug_phase_cleaned     = zeros(H, W);
debug_gabor_dc          = zeros(H, W);
debug_dwt_approx        = zeros(H, W);

if ~background_ready
    clean_detection     = zeros(H, W);
    frame_energy_scalar = 0;
    noise_floor_ready   = false;
    return;
end

% =====================================================================
% PRE-FILTER: 3-FRAME TEMPORAL AVERAGE
% =====================================================================
frame_buf(:,:,buf_ptr) = current_frame;
buf_ptr = int32(mod(double(buf_ptr), TEMPORAL_AVG_LEN) + 1);
if ~buf_filled && buf_ptr == int32(1)
    buf_filled = true;
end

if buf_filled
    avg_frame = mean(frame_buf, 3);
else
    avg_frame = current_frame;
end

% =====================================================================
% LAPLACIAN SHARPNESS DEFICIT
% =====================================================================
% Background Laplacian energy is constant once background stabilises.
% Cache it to avoid recomputing every frame (saves ~5 large matrix ops).
if ~bg_le_computed
    lap_bg = zeros(H, W);
    for r = 2:H-1
        for c = 2:W-1
            lap_bg(r,c) = background_frame(r-1,c) + background_frame(r+1,c) + ...
                          background_frame(r,c-1) + background_frame(r,c+1) - ...
                          4*background_frame(r,c);
        end
    end
    le_bg_cached = box_smooth(lap_bg.^2, H, W, K_SPATIAL) / win_area;

    % Compute a meaningful noise floor for the relative deficit.
    % This replaces the 1e-10 epsilon and naturally suppresses sky
    % (le_bg ≈ 0 → deficit ≈ 0) while preserving sensitivity in
    % out-of-focus nearfield (le_bg small but > le_floor).
    le_floor_cached = mean(le_bg_cached(:)) * 0.05;
    if le_floor_cached < 1
        le_floor_cached = 1;
    end

    bg_le_computed = true;
end
le_bg = le_bg_cached;

% Current frame Laplacian energy (recomputed every frame)
lap_curr = zeros(H, W);
for r = 2:H-1
    for c = 2:W-1
        lap_curr(r,c) = avg_frame(r-1,c) + avg_frame(r+1,c) + ...
                        avg_frame(r,c-1) + avg_frame(r,c+1) - ...
                        4*avg_frame(r,c);
    end
end
le_curr = box_smooth(lap_curr.^2, H, W, K_SPATIAL) / win_area;

% Relative sharpness deficit: normalized by local background sharpness.
% le_floor_cached prevents noise amplification in truly flat regions (sky)
% while preserving sensitivity in out-of-focus nearfield where the trail
% enters but le_bg is small.
raw_deficit = max((le_bg - le_curr) ./ (le_bg + le_floor_cached), 0);

% =====================================================================
% TEXTURE GATING  (low-texture suppression only)
% =====================================================================
% Suppress sky/uniform regions where blur is undetectable (no
% structure to lose). High-texture gating is disabled — the z-score
% noise floor already handles vegetation false positives.
tex_strength = le_bg;
tex_max = max(tex_strength(:)) + 1e-10;
tex_norm = tex_strength / tex_max;
% Low-texture gate: suppress sky/uniform regions where blur is
% undetectable (no structure to lose sharpness from).
% Threshold at 0.002 of max so out-of-focus nearfield passes through.
tex_weight_lo = min(tex_norm / 0.002, 1.0);
% High-texture gate: DISABLED. The z-score noise floor (Stage 6)
% already handles vegetation false positives — high-texture areas
% have high noise_std, so only truly anomalous deficits (the trail)
% produce significant z-scores. Applying tex_hi suppresses the
% mid-texture regions where the trail actually lives.
tex_weight_hi = ones(H, W);

tex_weight_lo = ones(H, W); % added by me 

tex_weight = tex_weight_lo .* tex_weight_hi;
debug_raw_deficit    = raw_deficit;   % BEFORE texture gating
debug_tex_weight     = tex_weight;    % combined gate
debug_tex_weight_lo  = tex_weight_lo; % low-texture gate (suppresses sky/uniform)
debug_tex_weight_hi  = tex_weight_hi; % high-texture gate (suppresses dense vegetation)
raw_deficit = raw_deficit .* tex_weight;

% =====================================================================
% INTENSITY DIFFERENCE CHANNEL
% =====================================================================
% Captures refractive brightness shifts missed by Laplacian deficit,
% especially in out-of-focus nearfield regions where both background
% and trail are blurry (Laplacian energy near zero for both).
% The schlieren trail bends light paths, causing local intensity
% shifts detectable as |current - background|.
intensity_diff = abs(avg_frame - background_frame);
% Apply full texture gating (both low and high).
% Low-texture gate suppresses sky where intensity noise variance is
% near-zero and z-scores would explode.  The nearfield out-of-focus
% regions pass through because tex_norm ≈ 0.002 > low-texture threshold.
intensity_diff = intensity_diff .* tex_weight;

% =====================================================================
% GABOR PHASE CHANGE CHANNEL
% =====================================================================
% Detects refractive distortion via local phase shift in the spatial
% frequency domain. The schlieren trail bends light paths, rotating
% the phase of local spatial patterns — detectable even in low-texture
% regions (snow, uniform ground) where Laplacian and intensity
% channels fail due to lack of background structure.
%
% Uses Gabor filter pairs (cosine + sine at a fixed frequency) at two
% orientations (horizontal and vertical). Phase change between current
% and background frame is computed via cross-spectral analysis.
% Smoothed with box_smooth to suppress single-pixel noise.

% Build 1D Gabor kernel pair (cosine and sine modulated Gaussian)
gabor_cos_k = zeros(1, 2*GABOR_HW+1);
gabor_sin_k = zeros(1, 2*GABOR_HW+1);
for k = -GABOR_HW:GABOR_HW
    idx = k + GABOR_HW + 1;
    g = exp(-k^2 / (2*GABOR_SIGMA^2));
    gabor_cos_k(idx) = g * cos(2*pi*GABOR_F0*k);
    gabor_sin_k(idx) = g * sin(2*pi*GABOR_F0*k);
end

% Cache background Gabor responses (computed once)
if ~phase_bg_computed
    phase_bg_h_cos = conv2(background_frame, gabor_cos_k, 'same');
    phase_bg_h_sin = conv2(background_frame, gabor_sin_k, 'same');
    phase_bg_v_cos = conv2(background_frame, gabor_cos_k', 'same');
    phase_bg_v_sin = conv2(background_frame, gabor_sin_k', 'same');
    phase_bg_computed = true;
end

% Current frame Gabor responses (horizontal and vertical orientations)
cur_h_cos = conv2(avg_frame, gabor_cos_k, 'same');
cur_h_sin = conv2(avg_frame, gabor_sin_k, 'same');
cur_v_cos = conv2(avg_frame, gabor_cos_k', 'same');
cur_v_sin = conv2(avg_frame, gabor_sin_k', 'same');

% Cross-spectral phase difference per orientation:
%   cross = resp_curr * conj(resp_bg)
%   phase_change = |angle(cross)| = |atan2(Im(cross), Re(cross))|
% Horizontal orientation
cross_re_h = cur_h_cos .* phase_bg_h_cos + cur_h_sin .* phase_bg_h_sin;
cross_im_h = cur_h_sin .* phase_bg_h_cos - cur_h_cos .* phase_bg_h_sin;
phase_change_h = abs(atan2(cross_im_h, cross_re_h));

% Vertical orientation
cross_re_v = cur_v_cos .* phase_bg_v_cos + cur_v_sin .* phase_bg_v_sin;
cross_im_v = cur_v_sin .* phase_bg_v_cos - cur_v_cos .* phase_bg_v_sin;
phase_change_v = abs(atan2(cross_im_v, cross_re_v));

% Combine orientations and smooth to suppress speckle
phase_diff = phase_change_h + phase_change_v;
phase_diff = box_smooth(phase_diff, H, W, K_SPATIAL) / win_area;

% =====================================================================
% MIRAGE NOISE FLOOR: ESTIMATION & SUBTRACTION
% Welford's online algorithm for computing running mean and variance
% in a single pass, applied per-pixel across frames
% =====================================================================
noise_count = noise_count + int32(1);
n = double(noise_count);

if ~noise_ready
    if n == 1
        noise_mean = raw_deficit;
        noise_var  = zeros(H, W);
        intensity_noise_mean = intensity_diff;
        intensity_noise_var  = zeros(H, W);
        phase_noise_mean = phase_diff;
        phase_noise_var  = zeros(H, W);
    else
        delta  = raw_deficit - noise_mean;
        noise_mean = noise_mean + delta / n;
        delta2 = raw_deficit - noise_mean;
        noise_var  = noise_var + delta .* delta2;

        delta_i  = intensity_diff - intensity_noise_mean;
        intensity_noise_mean = intensity_noise_mean + delta_i / n;
        delta2_i = intensity_diff - intensity_noise_mean;
        intensity_noise_var  = intensity_noise_var + delta_i .* delta2_i;

        delta_p  = phase_diff - phase_noise_mean;
        phase_noise_mean = phase_noise_mean + delta_p / n;
        delta2_p = phase_diff - phase_noise_mean;
        phase_noise_var  = phase_noise_var + delta_p .* delta2_p;
    end

    if noise_count >= int32(MIN_NOISE_FRAMES)
        noise_var = noise_var / (n - 1);
        intensity_noise_var = intensity_noise_var / (n - 1);
        phase_noise_var = phase_noise_var / (n - 1);
        noise_ready = true;
    end

    clean_detection     = raw_deficit;
    frame_energy_scalar = mean(raw_deficit(:));
    noise_floor_ready   = false;
    prev_deficit        = raw_deficit;
    % debug maps already set above (raw_deficit, tex_weight)
    return;
end

% --- Noise floor ready: apply suppression ---
noise_std = sqrt(max(noise_var + 1e-12, 0));
z_score = (raw_deficit - noise_mean) ./ (noise_std + 1e-10);
debug_pre_rowmed = max(z_score, 0);  % z-score BEFORE row median

% Row median subtraction: CMOS rolling shutter and post-shot camera
% vibration produce row-correlated elevation in the z-score map
% (affecting entire rows uniformly).  The row median captures this
% row-wise bias; subtracting it preserves localized features (the
% trail) while removing the row-wide component.
z_score = z_score - median(z_score, 2);
debug_post_rowmed = max(z_score, 0);  % z-score AFTER row median subtraction

spatial_cleaned = max(z_score - NOISE_MARGIN, 0);
debug_spatial_cleaned = spatial_cleaned;

temporal_diff = abs(raw_deficit - prev_deficit);
prev_deficit = raw_deficit;

td_zscore = temporal_diff ./ (noise_std + 1e-10);
td_zscore = td_zscore - median(td_zscore, 2);  % row median on temporal channel too
debug_td_zscore = max(td_zscore, 0);

td_cleaned = max(td_zscore - NOISE_MARGIN, 0);
debug_td_cleaned = td_cleaned;

% --- Intensity difference z-score ---
intensity_noise_std = sqrt(max(intensity_noise_var + 1e-12, 0));
% Minimum noise floor prevents z-score explosion in regions where
% noise variance converged to near-zero during calm baseline period.
int_noise_floor = max(mean(intensity_noise_std(:)) * 0.1, 0.5);
intensity_noise_std = max(intensity_noise_std, int_noise_floor);
intensity_z = (intensity_diff - intensity_noise_mean) ./ intensity_noise_std;
intensity_z = intensity_z - median(intensity_z, 2);  % row median
intensity_cleaned = max(intensity_z - NOISE_MARGIN, 0);
debug_intensity_cleaned = intensity_cleaned;

% --- Gabor phase z-score ---
phase_noise_std = sqrt(max(phase_noise_var + 1e-12, 0));
phase_floor = max(mean(phase_noise_std(:)) * 0.1, 1e-4);
phase_noise_std = max(phase_noise_std, phase_floor);
phase_z = (phase_diff - phase_noise_mean) ./ phase_noise_std;
phase_z = phase_z - median(phase_z, 2);  % row median
phase_cleaned = max(phase_z - NOISE_MARGIN, 0);
debug_phase_cleaned = phase_cleaned;

% Combine: 4-channel weighted sum.
%   spatial  — Laplacian sharpness deficit (textured regions)
%   temporal — frame-to-frame deficit change (onset detection)
%   intensity— brightness shift from refraction (mid-texture regions)
%   phase    — Gabor phase shift (works in low-texture regions: snow, ground)
clean_detection = 0.1 * spatial_cleaned ...
                + 0.1 * td_cleaned ...
                + 0.3 * intensity_cleaned ...
                + 0.5 * phase_cleaned;

% --- 2D DWT denoising (Phase 6) ---
% Removes low-frequency rectangular artifacts from H.264/H.265 codec
% macroblock quantization shifts (attenuates LL2 approximation band)
% and denoises detail bands via VisuShrink soft thresholding.
% Applied before Gabor weighting so the filter bank gets a cleaner input.
[clean_detection, debug_dwt_approx] = dwt2_enhance(clean_detection, H, W);

% --- Gabor directional filter bank weighting (Phase 3) ---
% Enhances linear trail features, suppresses isotropic blobs/noise.
% Applied to the combined detection map so all channels benefit.
gabor_dc = gabor_filter_bank(clean_detection, H, W);
debug_gabor_dc = gabor_dc;
gabor_weight = GABOR_FLOOR + (1.0 - GABOR_FLOOR) * min(gabor_dc / GABOR_DC_SAT, 1.0);
clean_detection = clean_detection .* gabor_weight;

% NOTE: no per-frame normalization here.  The raw z-score–based values
% must be scale-comparable across frames so that the temporal change map
% in trajectory_integrated_detection works correctly.  Visualization
% normalization happens downstream (main_pipeline map saving).

frame_energy_scalar = mean(max(z_score(:), 0));
noise_floor_ready = true;

end


% ================================================================
% BOX_SMOOTH — separable box filter via cumulative sums
% ================================================================
% Computes the sum of each pixel's (2K+1) x (2K+1) neighborhood.
% Uses two-pass cumulative sum (horizontal then vertical) for O(H*W)
% complexity regardless of K. Edges are handled by clamping the
% window to frame bounds (partial windows at borders).
%
% Inputs:
%   in  : HxW double, input image
%   H, W: frame dimensions
%   K   : half-width of the box (full window is 2K+1 pixels)
%
% Output:
%   out : HxW double, local sum within (2K+1)x(2K+1) window per pixel
%
% Note: returns the SUM, not the mean. Caller divides by window area
% (2K+1)^2 if a mean is needed. Used by the Laplacian energy
% computation to spatially smooth the squared Laplacian.
function out = box_smooth(in, H, W, K)
    % Pass 1: horizontal cumulative sum, then extract box sums per column
    cs_h = cumsum(in, 2);
    smooth_h = zeros(H, W);
    for c = 1:W
        c_lo = max(1, c - K);
        c_hi = min(W, c + K);
        if c_lo > 1
            smooth_h(:,c) = cs_h(:,c_hi) - cs_h(:,c_lo-1);
        else
            smooth_h(:,c) = cs_h(:,c_hi);
        end
    end
    % Pass 2: vertical cumulative sum on the horizontal result
    cs_v = cumsum(smooth_h, 1);
    out = zeros(H, W);
    for r = 1:H
        r_lo = max(1, r - K);
        r_hi = min(H, r + K);
        if r_lo > 1
            out(r,:) = cs_v(r_hi,:) - cs_v(r_lo-1,:);
        else
            out(r,:) = cs_v(r_hi,:);
        end
    end
end