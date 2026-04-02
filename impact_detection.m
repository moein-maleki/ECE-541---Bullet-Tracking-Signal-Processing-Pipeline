function [impact_x, impact_y, impact_detected, impact_conf, debug_splash_energy] = ...
    impact_detection(current_gray, background_gray, background_ready, ...
                     current_state, noise_floor_ready, ...
                     roi_center_x, roi_center_y, roi_radius)
%#codegen
% IMPACT_DETECTION
%
% Detects bullet impact location by looking for a sudden, persistent
% disturbance — dirt and debris kicked up by the bullet hitting the ground.
%
% Two modes:
%   ROI mode (roi_center_x > 0):
%     Uses BACKGROUND-DIFF PIXEL COUNT as detection signal.
%     More reliable in small areas because count-based detection is
%     robust to vegetation noise (a few noisy pixels don't dominate).
%     Learns baseline count during calm frames, triggers when count
%     spikes by 2x.
%
%   Full-frame mode (roi_center_x == 0):
%     Uses temporal energy sum over bottom 55% of frame.
%     Learns baseline, triggers when energy exceeds 1.5x baseline.
%
% Once impact is detected, the location is LOCKED and returned
% unchanged on all subsequent frames.
%
% ROI selection:
%   Run select_impact_roi('../longer.mp4') to interactively choose the
%   impact area, then set IMPACT_ROI_X/Y/RADIUS in main_pipeline.m.

% ===================== PARAMETERS =====================
DIFF_THRESHOLD     = 25;     % min gray-level change (bg diff) to count
TEMPORAL_THRESHOLD = 15;     % min gray-level change (frame-to-frame)
MIN_SPLASH_PIXELS  = 30;     % minimum disturbed pixels for valid splash
PERSISTENCE_FRAMES = 2;      % splash must persist for N frames to lock
MIN_FLIGHT_FRAMES  = 8;      % skip first N frames after FLIGHT (detection ramp-up)
TEMPORAL_ONSET_RATIO = 1.5;  % full-frame mode: energy ratio
BG_COUNT_ONSET_RATIO = 2.0;  % ROI mode: pixel count ratio

[H, W] = size(current_gray);

% ===================== PERSISTENT STATE =====================
persistent impact_locked loc_x loc_y
persistent prev_gray_stored has_prev
persistent baseline_sum baseline_count baseline_val baseline_ready
persistent splash_counter
persistent flight_frame_count_impact

if isempty(impact_locked)
    impact_locked   = false;
    loc_x           = 0;
    loc_y           = 0;
    prev_gray_stored = zeros(H, W);
    has_prev         = false;
    baseline_sum     = 0;
    baseline_count   = int32(0);
    baseline_val     = 0;
    baseline_ready   = false;
    splash_counter   = int32(0);
    flight_frame_count_impact = int32(0);
end

% ===================== DEFAULT OUTPUTS =====================
impact_x       = loc_x;
impact_y       = loc_y;
impact_detected = impact_locked;
impact_conf    = double(impact_locked);
debug_splash_energy = 0;

if impact_locked
    return;
end

if ~background_ready
    prev_gray_stored = current_gray;
    has_prev = true;
    return;
end

% ===================== SEARCH REGION =====================
if roi_center_x > 0
    r_lo = max(1, round(roi_center_y) - round(roi_radius));
    r_hi = min(H, round(roi_center_y) + round(roi_radius));
    c_lo = max(1, round(roi_center_x) - round(roi_radius));
    c_hi = min(W, round(roi_center_x) + round(roi_radius));
else
    GROUND_FRAC = 0.55;
    r_lo = max(1, round(H * (1 - GROUND_FRAC)));
    r_hi = H;
    c_lo = 1;
    c_hi = W;
end

% --- Frame-to-background difference (spatial) ---
n_bright_bg = 0;
sum_energy_bg = 0;
sum_wx_bg = 0;
sum_wy_bg = 0;

for r = r_lo:r_hi
    for c = c_lo:c_hi
        d = abs(current_gray(r,c) - background_gray(r,c));
        if d > DIFF_THRESHOLD
            n_bright_bg = n_bright_bg + 1;
            sum_energy_bg = sum_energy_bg + d;
            sum_wx_bg = sum_wx_bg + c * d;
            sum_wy_bg = sum_wy_bg + r * d;
        end
    end
end

% --- Frame-to-frame difference (temporal onset) ---
temporal_energy = 0;
n_bright_temp = 0;
sum_wx_temp = 0;
sum_wy_temp = 0;

if has_prev
    for r = r_lo:r_hi
        for c = c_lo:c_hi
            d = abs(current_gray(r,c) - prev_gray_stored(r,c));
            if d > TEMPORAL_THRESHOLD
                n_bright_temp = n_bright_temp + 1;
                temporal_energy = temporal_energy + d;
                sum_wx_temp = sum_wx_temp + c * d;
                sum_wy_temp = sum_wy_temp + r * d;
            end
        end
    end
end

prev_gray_stored = current_gray;
has_prev = true;

% ===================== DETECTION SIGNAL =====================
% ROI mode: use background-diff pixel COUNT (robust to distributed noise).
% Full-frame mode: use temporal energy SUM (sensitive to large-area onset).
if roi_center_x > 0
    splash_signal = double(n_bright_bg);
else
    splash_signal = temporal_energy;
end
debug_splash_energy = splash_signal;

% ===================== BASELINE LEARNING (before flight) =====================
if current_state == int32(0)
    if noise_floor_ready
        baseline_count = baseline_count + int32(1);
        baseline_sum = baseline_sum + splash_signal;
        if baseline_count >= int32(20)
            baseline_val = baseline_sum / double(baseline_count);
            baseline_ready = true;
        end
    end
    splash_counter = int32(0);
    flight_frame_count_impact = int32(0);
    return;
end

% ===================== POST-FLIGHT: DETECT SPLASH =====================
if current_state >= int32(1)
    flight_frame_count_impact = flight_frame_count_impact + int32(1);
end

if flight_frame_count_impact <= int32(MIN_FLIGHT_FRAMES)
    splash_counter = int32(0);
    return;
end

if ~baseline_ready
    return;
end

% Trigger logic depends on mode
if roi_center_x > 0
    % ROI mode: background-diff pixel count must spike above baseline.
    % Baseline captures normal vegetation/noise count; splash adds many
    % more pixels.  Minimum threshold of 30 prevents triggering on
    % near-empty ROIs.
    threshold = max(baseline_val * BG_COUNT_ONSET_RATIO, 30);
    splash_triggered = splash_signal > threshold;
else
    % Full-frame mode: temporal energy sum must spike.
    threshold = max(baseline_val * TEMPORAL_ONSET_RATIO, 1000);
    splash_triggered = splash_signal > threshold && n_bright_temp >= MIN_SPLASH_PIXELS;
end

if splash_triggered
    splash_counter = splash_counter + int32(1);
else
    splash_counter = max(splash_counter - int32(1), int32(0));
end

% Lock impact once persistent splash confirmed
if splash_counter >= int32(PERSISTENCE_FRAMES)
    % Use background-diff centroid for impact location (most stable)
    if n_bright_bg >= MIN_SPLASH_PIXELS && sum_energy_bg > 0
        loc_x = sum_wx_bg / sum_energy_bg;
        loc_y = sum_wy_bg / sum_energy_bg;
    elseif temporal_energy > 0
        loc_x = sum_wx_temp / temporal_energy;
        loc_y = sum_wy_temp / temporal_energy;
    else
        % Fall back to ROI center
        loc_x = double(roi_center_x);
        loc_y = double(roi_center_y);
    end
    impact_locked = true;
    impact_x = loc_x;
    impact_y = loc_y;
    impact_detected = true;
    impact_conf = 1.0;
end

end
