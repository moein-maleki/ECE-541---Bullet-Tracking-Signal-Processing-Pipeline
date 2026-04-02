function directional_contrast = gabor_filter_bank(input_map, H, W)
%#codegen
% GABOR_FILTER_BANK  8-orientation x 2-scale oriented energy filter bank
%
% Computes directional contrast: how much the maximum-orientation Gabor
% energy exceeds the mean across orientations. High values indicate
% pixels belonging to linear/elongated structures (trail); low values
% indicate isotropic features (blobs, noise).
%
% Algorithm:
%   1. Downsample input 2x (block average) for performance
%   2. For each of 8 orientations (0 to 157.5 deg, 22.5 deg steps):
%      a. For each of 2 scales (sigma=3, sigma=6):
%         - Build 2D Gabor kernel pair (cosine + sine modulated Gaussian)
%         - Convolve with input to get real and imaginary responses
%         - Accumulate energy = Re^2 + Im^2
%      b. Track per-pixel max and sum of orientation energies
%   3. Directional contrast = (max - mean) / (mean + eps)
%   4. Upsample result to full resolution
%
% Inputs:
%   input_map : HxW double, detection map (or any 2D signal)
%   H, W      : frame dimensions
%
% Output:
%   directional_contrast : HxW double, >= 0. Higher = more directional.
%     Typical range: 0 (isotropic) to ~7 (all energy in one orientation).

N_ORIENT = 8;        % orientations: 0 to 157.5 deg in 22.5 deg steps
SIGMAS_1 = 3;        % fine scale
SIGMAS_2 = 6;        % coarse scale
GAMMA    = 0.5;      % spatial aspect ratio (elongates filter along orientation)
MAX_HW   = 18;       % ceil(3 * max sigma) — fixed kernel half-width
KSZ      = 2*MAX_HW + 1;  % 37x37 kernel

% --- Downsample 2x (2x2 block average) ---
H2 = floor(H / 2);
W2 = floor(W / 2);
small = zeros(H2, W2);
for r = 1:H2
    r1 = 2*r - 1;
    r2 = min(H, 2*r);
    for c = 1:W2
        c1 = 2*c - 1;
        c2 = min(W, 2*c);
        small(r,c) = (input_map(r1,c1) + input_map(r1,c2) ...
                    + input_map(r2,c1) + input_map(r2,c2)) * 0.25;
    end
end

max_energy = zeros(H2, W2);
sum_energy = zeros(H2, W2);

for oi = 1:N_ORIENT
    theta = double(oi - 1) * pi / double(N_ORIENT);
    ct = cos(theta);
    st = sin(theta);

    energy = zeros(H2, W2);

    % --- Scale 1: sigma = 3, f0 = 1/6 ---
    f0_1 = 1.0 / (2.0 * SIGMAS_1);
    k_cos = zeros(KSZ, KSZ);
    k_sin = zeros(KSZ, KSZ);
    for kr = -MAX_HW:MAX_HW
        for kc = -MAX_HW:MAX_HW
            xr = double(kc) * ct + double(kr) * st;
            yr = -double(kc) * st + double(kr) * ct;
            g = exp(-(xr*xr + GAMMA*GAMMA * yr*yr) / (2*SIGMAS_1*SIGMAS_1));
            k_cos(kr+MAX_HW+1, kc+MAX_HW+1) = g * cos(2*pi*f0_1*xr);
            k_sin(kr+MAX_HW+1, kc+MAX_HW+1) = g * sin(2*pi*f0_1*xr);
        end
    end
    rc = conv2(small, k_cos, 'same');
    rs = conv2(small, k_sin, 'same');
    energy = energy + rc .* rc + rs .* rs;

    % --- Scale 2: sigma = 6, f0 = 1/12 ---
    f0_2 = 1.0 / (2.0 * SIGMAS_2);
    k_cos2 = zeros(KSZ, KSZ);
    k_sin2 = zeros(KSZ, KSZ);
    for kr = -MAX_HW:MAX_HW
        for kc = -MAX_HW:MAX_HW
            xr = double(kc) * ct + double(kr) * st;
            yr = -double(kc) * st + double(kr) * ct;
            g = exp(-(xr*xr + GAMMA*GAMMA * yr*yr) / (2*SIGMAS_2*SIGMAS_2));
            k_cos2(kr+MAX_HW+1, kc+MAX_HW+1) = g * cos(2*pi*f0_2*xr);
            k_sin2(kr+MAX_HW+1, kc+MAX_HW+1) = g * sin(2*pi*f0_2*xr);
        end
    end
    rc2 = conv2(small, k_cos2, 'same');
    rs2 = conv2(small, k_sin2, 'same');
    energy = energy + rc2 .* rc2 + rs2 .* rs2;

    % --- Update running max and sum across orientations ---
    for r = 1:H2
        for c = 1:W2
            if energy(r,c) > max_energy(r,c)
                max_energy(r,c) = energy(r,c);
            end
        end
    end
    sum_energy = sum_energy + energy;
end

mean_energy = sum_energy / double(N_ORIENT);

% Directional contrast: anisotropy ratio
dc_small = (max_energy - mean_energy) ./ (mean_energy + 1e-10);

% --- Upsample to full resolution (nearest neighbor) ---
directional_contrast = zeros(H, W);
for r = 1:H
    sr = min(H2, max(1, ceil(double(r) / 2)));
    for c = 1:W
        sc = min(W2, max(1, ceil(double(c) / 2)));
        directional_contrast(r,c) = dc_small(sr, sc);
    end
end

end
