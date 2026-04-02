function [current_state, processing_active, state_duration, onset_threshold_out] = ...
    flight_state_machine(frame_energy_scalar, noise_floor_ready, ...
                            tracking_active, confidence, path_x, path_y)
%#codegen
% FLIGHT_STATE_MACHINE
%
% Adaptive onset threshold: after noise_floor_ready, accumulates
% frame_energy_scalar for ONSET_CAL_FRAMES frames to learn its
% baseline mean and std. Onset threshold = mean + K_ONSET * std.
%
% Inputs:
%   frame_energy_scalar : double, mean z-score energy of current frame
%   noise_floor_ready   : logical, noise baseline calibrated
%   tracking_active     : logical, from trajectory integrator (prev frame)
%   confidence          : double, tracking confidence (prev frame)
%   path_x, path_y      : double, predicted position (prev frame)
%
% States (int32):
%   0 = WAITING  : background accumulation, no object detected
%   1 = FLIGHT   : trail detected, actively tracking
%   2 = LOST     : trail temporarily invisible, prediction-only
%   3 = IMPACT   : terminal state

% ===================== PARAMETERS =====================
ONSET_CAL_FRAMES = 60;     % frames to calibrate onset baseline
K_ONSET          = 8;      % sigma multiplier for onset threshold
ONSET_DEBOUNCE   = 3;

LOST_CONF_THRESH = 0.02;
LOST_DEBOUNCE    = 5;
FLIGHT_GRACE     = 15;     % don't go LOST during first N frames of flight
                           % (allows cooldown + N_INIT centroid collection)

REACQ_CONF_THRESH= 0.05;
REACQ_DEBOUNCE   = 2;

LOST_TIMEOUT     = 8;      % stricter: bullet is fast, if lost for 8 frames it's gone

FRAME_MARGIN     = 20;
FRAME_W          = 1920;
FRAME_H          = 1080;

% ===================== PERSISTENT STATE =====================
persistent state
persistent frames_in_state
persistent onset_counter
persistent lost_counter
persistent reacq_counter
persistent flight_frame_count

% --- Onset calibration ---
persistent onset_cal_count        % frames accumulated for onset baseline
persistent onset_cal_ready        % baseline calibrated
persistent onset_energy_sum       % running sum of frame_energy_scalar
persistent onset_energy_sq_sum    % running sum of squared values
persistent onset_threshold        % adaptive threshold

% ===================== INITIALIZATION =====================
if isempty(state)
    state              = int32(0);
    frames_in_state    = int32(0);
    onset_counter      = int32(0);
    lost_counter       = int32(0);
    reacq_counter      = int32(0);
    flight_frame_count = int32(0);

    onset_cal_count    = int32(0);
    onset_cal_ready    = false;
    onset_energy_sum   = 0;
    onset_energy_sq_sum= 0;
    onset_threshold    = 1e10;     % impossibly high until calibrated
end
onset_threshold_out = onset_threshold;
frames_in_state = frames_in_state + int32(1);

% ===================== STATE: WAITING (0) =====================
if state == int32(0)

    processing_active = false;

    if noise_floor_ready
        if ~onset_cal_ready
            % --- ONSET CALIBRATION PHASE ---
            % Accumulate frame_energy_scalar statistics to learn
            % the baseline level during calm background.
            onset_cal_count = onset_cal_count + int32(1);
            onset_energy_sum = onset_energy_sum + frame_energy_scalar;
            onset_energy_sq_sum = onset_energy_sq_sum + ...
                                  frame_energy_scalar * frame_energy_scalar;

            if onset_cal_count >= int32(ONSET_CAL_FRAMES)
                n = double(onset_cal_count);
                mu = onset_energy_sum / n;
                var_val = onset_energy_sq_sum / n - mu * mu;
                sigma = sqrt(max(var_val, 1e-12));

                onset_threshold = mu + K_ONSET * sigma;
                onset_cal_ready = true;
            end
        else
            % --- ONSET DETECTION (threshold is now valid) ---
            if frame_energy_scalar > onset_threshold
                onset_counter = onset_counter + int32(1);
            else
                onset_counter = int32(0);
            end

            if onset_counter >= int32(ONSET_DEBOUNCE)
                state           = int32(1);
                frames_in_state = int32(0);
                onset_counter   = int32(0);
                lost_counter    = int32(0);
                flight_frame_count = int32(0);
                processing_active = true;
            end
        end
    end

    current_state    = state;
    state_duration   = frames_in_state;
    return;
end

% ===================== STATE: FLIGHT (1) =====================
if state == int32(1)

    processing_active  = true;
    flight_frame_count = flight_frame_count + int32(1);

    if confidence < LOST_CONF_THRESH
        lost_counter = lost_counter + int32(1);
    else
        lost_counter = int32(0);
    end

    exited_frame = (path_x < FRAME_MARGIN) || ...
                   (path_x > FRAME_W - FRAME_MARGIN) || ...
                   (path_y < FRAME_MARGIN) || ...
                   (path_y > FRAME_H - FRAME_MARGIN);

    if exited_frame && tracking_active
        state           = int32(3);
        frames_in_state = int32(0);
        lost_counter    = int32(0);
    elseif lost_counter >= int32(LOST_DEBOUNCE) && ...
           flight_frame_count > int32(FLIGHT_GRACE)
        % Grace period: don't go LOST during initial seed collection.
        % The trajectory block needs cooldown + N_INIT frames to
        % establish tracking before confidence becomes meaningful.
        state           = int32(2);
        frames_in_state = int32(0);
        lost_counter    = int32(0);
        reacq_counter   = int32(0);
    end

    current_state    = state;
    state_duration   = frames_in_state;
    return;
end

% ===================== STATE: LOST (2) =====================
if state == int32(2)

    processing_active  = true;
    flight_frame_count = flight_frame_count + int32(1);

    if confidence > REACQ_CONF_THRESH
        reacq_counter = reacq_counter + int32(1);
    else
        reacq_counter = int32(0);
    end

    if reacq_counter >= int32(REACQ_DEBOUNCE)
        state           = int32(1);
        frames_in_state = int32(0);
        reacq_counter   = int32(0);
        lost_counter    = int32(0);
    elseif frames_in_state >= int32(LOST_TIMEOUT)
        state           = int32(3);
        frames_in_state = int32(0);
    end

    current_state    = state;
    state_duration   = frames_in_state;
    return;
end

% ===================== STATE: IMPACT (3) =====================
if state == int32(3)

    processing_active = false;

    current_state    = state;
    state_duration   = frames_in_state;
    return;
end

current_state     = state;
processing_active = false;
state_duration    = frames_in_state;

end
