function [enhanced, debug_approx] = dwt2_enhance(input_map, H, W)
%#codegen
% DWT2_ENHANCE  2-level 2D Haar wavelet denoising for detection maps
%
% Suppresses low-frequency rectangular artifacts from H.264/H.265 video
% codec macroblock quantization shifts, and denoises detail bands via
% VisuShrink soft thresholding.
%
% Algorithm:
%   1. 2-level forward Haar DWT (manual, no toolbox):
%      Level 1: HxW -> LL1, LH1, HL1, HH1 at H/2 x W/2
%      Level 2: LL1  -> LL2, LH2, HL2, HH2 at H/4 x W/4
%   2. LL2 median subtraction: the codec artifact is a spatially uniform
%      elevation in LL2; subtracting the median removes it while
%      preserving local peaks (trail features that exceed the background)
%   3. Soft-threshold detail bands (scaled VisuShrink):
%      - Noise sigma estimated from MAD of finest diagonal band (HH1)
%      - Threshold T = THRESH_SCALE * sigma * sqrt(2 * log(N))
%      - THRESH_SCALE=0.5 (softer than full VisuShrink to preserve trail)
%      - Soft threshold: sign(x) * max(|x| - T, 0)
%   4. 2-level inverse Haar DWT to reconstruct enhanced map
%
% Inputs:
%   input_map : HxW double, detection map
%   H, W      : frame dimensions
%
% Outputs:
%   enhanced     : HxW double, denoised detection map (non-negative)
%   debug_approx : HxW double, LL2-only reconstruction (what was removed)

% ===================== PARAMETERS =====================
THRESH_SCALE = 0.5;  % fraction of VisuShrink threshold (1.0 = full, <1 = softer)

% ===================== PAD TO EVEN (both levels need even dims) =====================
% Level 1 needs H,W even. Level 2 needs H/2, W/2 even → H,W divisible by 4.
H4 = H + mod(4 - mod(H, 4), 4);  % round up to multiple of 4
W4 = W + mod(4 - mod(W, 4), 4);
if H4 == H + 4; H4 = H; end  % already multiple of 4
if W4 == W + 4; W4 = W; end
padded = zeros(H4, W4);
padded(1:H, 1:W) = input_map;

% ===================== LEVEL 1 FORWARD =====================
H1 = H4 / 2;
W1 = W4 / 2;
LL1 = zeros(H1, W1);
LH1 = zeros(H1, W1);
HL1 = zeros(H1, W1);
HH1 = zeros(H1, W1);

for r = 1:H1
    r2a = 2*r - 1;
    r2b = 2*r;
    for c = 1:W1
        c2a = 2*c - 1;
        c2b = 2*c;
        a = padded(r2a, c2a);   % top-left
        b = padded(r2a, c2b);   % top-right
        cc = padded(r2b, c2a);  % bottom-left
        d = padded(r2b, c2b);   % bottom-right
        LL1(r,c) = (a + b + cc + d) * 0.5;
        LH1(r,c) = (a + b - cc - d) * 0.5;   % horizontal detail
        HL1(r,c) = (a - b + cc - d) * 0.5;   % vertical detail
        HH1(r,c) = (a - b - cc + d) * 0.5;   % diagonal detail
    end
end

% ===================== LEVEL 2 FORWARD =====================
H2 = H1 / 2;
W2 = W1 / 2;
LL2 = zeros(H2, W2);
LH2 = zeros(H2, W2);
HL2 = zeros(H2, W2);
HH2 = zeros(H2, W2);

for r = 1:H2
    r2a = 2*r - 1;
    r2b = 2*r;
    for c = 1:W2
        c2a = 2*c - 1;
        c2b = 2*c;
        a = LL1(r2a, c2a);
        b = LL1(r2a, c2b);
        cc = LL1(r2b, c2a);
        d = LL1(r2b, c2b);
        LL2(r,c) = (a + b + cc + d) * 0.5;
        LH2(r,c) = (a + b - cc - d) * 0.5;
        HL2(r,c) = (a - b + cc - d) * 0.5;
        HH2(r,c) = (a - b - cc + d) * 0.5;
    end
end

% ===================== NOISE ESTIMATION (MAD of HH1) =====================
% Robust noise estimator: sigma = median(|HH1|) / 0.6745
% Histogram-based median for codegen compatibility (no sort).
n_hh1 = H1 * W1;
max_abs = 0;
for r = 1:H1
    for c = 1:W1
        v = abs(HH1(r,c));
        if v > max_abs
            max_abs = v;
        end
    end
end

if max_abs < 1e-10
    % No signal — return input unchanged
    enhanced = input_map;
    debug_approx = zeros(H, W);
    return;
end

N_BINS_MED = 200;
bin_w = max_abs / double(N_BINS_MED);
hist_counts = zeros(N_BINS_MED, 1);
for r = 1:H1
    for c = 1:W1
        b = min(N_BINS_MED, max(1, ceil(abs(HH1(r,c)) / bin_w)));
        hist_counts(b) = hist_counts(b) + 1;
    end
end

cum = 0;
median_bin = 1;
half_count = double(n_hh1) / 2.0;
for b = 1:N_BINS_MED
    cum = cum + hist_counts(b);
    if cum >= half_count
        median_bin = b;
        break;
    end
end

mad_median = (double(median_bin) - 0.5) * bin_w;
sigma_noise = mad_median / 0.6745;

% VisuShrink universal threshold, scaled down for less aggressive denoising.
% Full VisuShrink (THRESH_SCALE=1.0) with N~2M gives T~5.4*sigma, which
% over-smooths the trail. 0.5x retains more trail detail.
n_total = double(H4 * W4);
threshold = THRESH_SCALE * sigma_noise * sqrt(2.0 * log(n_total));

% ===================== SAVE ORIGINAL LL2 FOR DEBUG =====================
LL2_orig = LL2;

% ===================== LL2 MEDIAN SUBTRACTION =====================
% The codec artifact is a spatially uniform elevation across LL2.
% The trail creates local peaks that exceed the background level.
% Subtracting the median removes the uniform elevation (artifact)
% while preserving local peaks (trail).
% Histogram-based median for codegen compatibility.
n_ll2 = H2 * W2;
ll2_max = 0;
for r = 1:H2
    for c = 1:W2
        if LL2(r,c) > ll2_max
            ll2_max = LL2(r,c);
        end
    end
end

if ll2_max > 1e-10
    N_BINS_LL2 = 200;
    bin_w_ll2 = ll2_max / double(N_BINS_LL2);
    hist_ll2 = zeros(N_BINS_LL2, 1);
    for r = 1:H2
        for c = 1:W2
            b = min(N_BINS_LL2, max(1, ceil(LL2(r,c) / bin_w_ll2)));
            hist_ll2(b) = hist_ll2(b) + 1;
        end
    end
    cum_ll2 = 0;
    med_bin_ll2 = 1;
    half_ll2 = double(n_ll2) / 2.0;
    for b = 1:N_BINS_LL2
        cum_ll2 = cum_ll2 + hist_ll2(b);
        if cum_ll2 >= half_ll2
            med_bin_ll2 = b;
            break;
        end
    end
    ll2_median = (double(med_bin_ll2) - 0.5) * bin_w_ll2;

    for r = 1:H2
        for c = 1:W2
            v = LL2(r,c) - ll2_median;
            if v > 0
                LL2(r,c) = v;
            else
                LL2(r,c) = 0;
            end
        end
    end
end

% ===================== SOFT THRESHOLD DETAIL BANDS =====================
% Level 2 details
for r = 1:H2
    for c = 1:W2
        v = LH2(r,c);
        if v > threshold
            LH2(r,c) = v - threshold;
        elseif v < -threshold
            LH2(r,c) = v + threshold;
        else
            LH2(r,c) = 0;
        end

        v = HL2(r,c);
        if v > threshold
            HL2(r,c) = v - threshold;
        elseif v < -threshold
            HL2(r,c) = v + threshold;
        else
            HL2(r,c) = 0;
        end

        v = HH2(r,c);
        if v > threshold
            HH2(r,c) = v - threshold;
        elseif v < -threshold
            HH2(r,c) = v + threshold;
        else
            HH2(r,c) = 0;
        end
    end
end

% Level 1 details
for r = 1:H1
    for c = 1:W1
        v = LH1(r,c);
        if v > threshold
            LH1(r,c) = v - threshold;
        elseif v < -threshold
            LH1(r,c) = v + threshold;
        else
            LH1(r,c) = 0;
        end

        v = HL1(r,c);
        if v > threshold
            HL1(r,c) = v - threshold;
        elseif v < -threshold
            HL1(r,c) = v + threshold;
        else
            HL1(r,c) = 0;
        end

        v = HH1(r,c);
        if v > threshold
            HH1(r,c) = v - threshold;
        elseif v < -threshold
            HH1(r,c) = v + threshold;
        else
            HH1(r,c) = 0;
        end
    end
end

% ===================== LEVEL 2 INVERSE =====================
LL1_recon = zeros(H1, W1);
for r = 1:H2
    r2a = 2*r - 1;
    r2b = 2*r;
    for c = 1:W2
        c2a = 2*c - 1;
        c2b = 2*c;
        ll = LL2(r,c);
        lh = LH2(r,c);
        hl = HL2(r,c);
        hh = HH2(r,c);
        LL1_recon(r2a, c2a) = (ll + lh + hl + hh) * 0.5;
        LL1_recon(r2a, c2b) = (ll + lh - hl - hh) * 0.5;
        LL1_recon(r2b, c2a) = (ll - lh + hl - hh) * 0.5;
        LL1_recon(r2b, c2b) = (ll - lh - hl + hh) * 0.5;
    end
end

% ===================== LEVEL 1 INVERSE =====================
recon = zeros(H4, W4);
for r = 1:H1
    r2a = 2*r - 1;
    r2b = 2*r;
    for c = 1:W1
        c2a = 2*c - 1;
        c2b = 2*c;
        ll = LL1_recon(r,c);
        lh = LH1(r,c);
        hl = HL1(r,c);
        hh = HH1(r,c);
        recon(r2a, c2a) = (ll + lh + hl + hh) * 0.5;
        recon(r2a, c2b) = (ll + lh - hl - hh) * 0.5;
        recon(r2b, c2a) = (ll - lh + hl - hh) * 0.5;
        recon(r2b, c2b) = (ll - lh - hl + hh) * 0.5;
    end
end

% ===================== CROP AND CLAMP =====================
enhanced = zeros(H, W);
for r = 1:H
    for c = 1:W
        v = recon(r, c);
        if v > 0
            enhanced(r, c) = v;
        end
    end
end

% ===================== DEBUG: LL2 MEDIAN BASELINE RECONSTRUCTION =========
% Shows the uniform low-frequency baseline that was subtracted from LL2
% (the codec artifact). Reconstructs from the median portion of LL2_orig
% (i.e., LL2_orig - LL2_after_median_subtraction) with all details zeroed.
% LL2_removed = what was subtracted (original minus what remains)
LL2_removed = zeros(H2, W2);
for r = 1:H2
    for c = 1:W2
        LL2_removed(r,c) = LL2_orig(r,c) - LL2(r,c);
    end
end

LL1_approx = zeros(H1, W1);
for r = 1:H2
    r2a = 2*r - 1;
    r2b = 2*r;
    for c = 1:W2
        c2a = 2*c - 1;
        c2b = 2*c;
        v = LL2_removed(r,c) * 0.5;
        LL1_approx(r2a, c2a) = v;
        LL1_approx(r2a, c2b) = v;
        LL1_approx(r2b, c2a) = v;
        LL1_approx(r2b, c2b) = v;
    end
end

approx_full = zeros(H4, W4);
for r = 1:H1
    r2a = 2*r - 1;
    r2b = 2*r;
    for c = 1:W1
        c2a = 2*c - 1;
        c2b = 2*c;
        v = LL1_approx(r,c) * 0.5;
        approx_full(r2a, c2a) = v;
        approx_full(r2a, c2b) = v;
        approx_full(r2b, c2a) = v;
        approx_full(r2b, c2b) = v;
    end
end

debug_approx = approx_full(1:H, 1:W);

end
