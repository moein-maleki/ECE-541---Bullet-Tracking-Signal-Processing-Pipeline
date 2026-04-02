%% MAIN_PIPELINE — Schlieren trail tracking (no Simulink)
% Runs the full detection/tracking pipeline frame-by-frame.
clc;
close all;
clear all; %#ok<CLALL> — reset persistent state in all functions

%% ===================== USER CONFIG =====================
% Impact ROI: approximate bullet impact location.
% Run select_impact_roi('../longer.mp4') to choose interactively,
% or set manually from visual inspection.
% Set IMPACT_ROI_X = 0 to disable (uses full ground search).
% IMPACT_ROI_X      = 0;       % X pixel coorclsoedinate of impact area center
% IMPACT_ROI_Y      = 0;       % Y pixel coordinate of impact area center
% IMPACT_ROI_RADIUS = 150;     % search radius around impact center (pixels)

IMPACT_ROI_X      = 1451;
IMPACT_ROI_Y      = 589;
IMPACT_ROI_RADIUS = 100;


%% ===================== VIDEO I/O =====================
input_video  = fullfile('..', 'longer.mp4');
output_video = 'output.avi';

reader = VideoReader(input_video);
writer = VideoWriter(output_video, 'Uncompressed AVI');
writer.FrameRate = reader.FrameRate;
open(writer);

num_frames = floor(reader.Duration * reader.FrameRate);
fprintf('Video: %dx%d, %.1f fps, ~%d frames\n', ...
    reader.Width, reader.Height, reader.FrameRate, num_frames);

%% ===================== RUN DIRECTORY =====================
run_dir = fullfile('.', 'runs', datestr(now, 'yyyy-mm-dd_HH-MM-SS'));
mkdir(run_dir);
maps_dir = fullfile(run_dir, 'maps');
mkdir(maps_dir);
maps_det_dir       = fullfile(maps_dir, 'detection');      mkdir(maps_det_dir);
maps_raw_dir       = fullfile(maps_dir, 'raw_deficit');     mkdir(maps_raw_dir);
maps_texw_dir      = fullfile(maps_dir, 'tex_weight');      mkdir(maps_texw_dir);
maps_texlo_dir     = fullfile(maps_dir, 'tex_lo');          mkdir(maps_texlo_dir);
maps_texhi_dir     = fullfile(maps_dir, 'tex_hi');          mkdir(maps_texhi_dir);
maps_prerow_dir    = fullfile(maps_dir, 'pre_rowmed');      mkdir(maps_prerow_dir);
maps_postrow_dir   = fullfile(maps_dir, 'post_rowmed');     mkdir(maps_postrow_dir);
maps_spatial_dir   = fullfile(maps_dir, 'spatial_cleaned'); mkdir(maps_spatial_dir);
maps_tdz_dir       = fullfile(maps_dir, 'td_zscore');       mkdir(maps_tdz_dir);
maps_tdc_dir       = fullfile(maps_dir, 'td_cleaned');      mkdir(maps_tdc_dir);
maps_intc_dir      = fullfile(maps_dir, 'int_cleaned');     mkdir(maps_intc_dir);
maps_phase_dir     = fullfile(maps_dir, 'phase_cleaned');   mkdir(maps_phase_dir);
maps_gabor_dir     = fullfile(maps_dir, 'gabor_dc');        mkdir(maps_gabor_dir);
maps_dwt_dir       = fullfile(maps_dir, 'dwt_approx');      mkdir(maps_dwt_dir);
maps_pca_pre_dir   = fullfile(maps_dir, 'pca_pre_weight');  mkdir(maps_pca_pre_dir);
maps_pca_aniso_dir = fullfile(maps_dir, 'pca_aniso');       mkdir(maps_pca_aniso_dir);
maps_pca_post_dir  = fullfile(maps_dir, 'pca_post_weight'); mkdir(maps_pca_post_dir);
fprintf('Run directory: %s\n', run_dir);

%% ===================== CONSOLE LOG =====================
diary_file = fullfile(run_dir, 'console.md');
diary(diary_file);

%% ===================== PREALLOCATE LOG =====================
log = struct();
log.frame_energy_scalar = zeros(num_frames, 1);
log.current_state       = zeros(num_frames, 1);
log.confidence          = zeros(num_frames, 1);
log.path_x              = zeros(num_frames, 1);
log.path_y              = zeros(num_frames, 1);
log.noise_floor_ready   = false(num_frames, 1);
log.tracking_active     = false(num_frames, 1);
log.path_len            = zeros(num_frames, 1);
log.onset_threshold     = zeros(num_frames, 1);
log.background_ready    = false(num_frames, 1);
log.state_duration      = zeros(num_frames, 1);
log.processing_active   = false(num_frames, 1);
log.ax = zeros(num_frames, 1); log.bx = zeros(num_frames, 1); log.cx = zeros(num_frames, 1);
log.ay = zeros(num_frames, 1); log.by = zeros(num_frames, 1); log.cy = zeros(num_frames, 1);
log.t_ref               = zeros(num_frames, 1);
log.detection_max       = zeros(num_frames, 1);
log.detection_mean      = zeros(num_frames, 1);
log.debug_centroid_x    = zeros(num_frames, 1);
log.debug_centroid_y    = zeros(num_frames, 1);
log.impact_x            = zeros(num_frames, 1);
log.impact_y            = zeros(num_frames, 1);
log.impact_detected     = false(num_frames, 1);
log.splash_energy       = zeros(num_frames, 1);
log.debug_n_detections  = zeros(num_frames, 1);
log.debug_mode          = zeros(num_frames, 1);
log.debug_merged        = zeros(num_frames, 1);
log.debug_rejected      = zeros(num_frames, 1);
log.debug_change_max    = zeros(num_frames, 1);
log.debug_pca_elongation = zeros(num_frames, 1);
log.debug_accum_count    = zeros(num_frames, 1);

% Previous-frame outputs for the trajectory block (breaks algebraic loop)
prev_tracking_active = false;
prev_confidence      = 0;
prev_path_x          = 0;
prev_path_y          = 0;

% Impact detection state (passed to overlay)
impact_x_final        = 0;
impact_y_final        = 0;
impact_detected_final = false;

% Blob (first centroid) position for debug overlay
blob_x_final = 0;
blob_y_final = 0;

% Centroid log for permanent overlay markers
MAX_CENTROID_LOG = 200;
centroid_log_x   = zeros(MAX_CENTROID_LOG, 1);
centroid_log_y   = zeros(MAX_CENTROID_LOG, 1);
centroid_log_len = 0;

frame_idx = 0;
flight_map_count = 0;       % detection maps saved during flight
MAX_FLIGHT_MAPS  = 30;      % cap on saved maps
compare_saved = false;       % debug path comparison frame
compare_wait  = 0;           % wait N frames into tracking for stable fit

%% ===================== FRAME LOOP =====================
while hasFrame(reader)
    frame_idx = frame_idx + 1;
    rgb = readFrame(reader);  % uint8 HxWx3

    % --- BT.709 grayscale ---
    gray = double(rgb(:,:,1)) * 0.2126 + ...
           double(rgb(:,:,2)) * 0.7152 + ...
           double(rgb(:,:,3)) * 0.0722;

    % --- 1. Background estimation ---
    [background_frame, background_ready] = ...
        temporal_median_background(gray);

    % --- 2. Sharpness detection + mirage suppression ---
    [clean_detection, frame_energy_scalar, noise_floor_ready, ...
     debug_raw_deficit, debug_tex_weight, debug_pre_rowmed, ...
     debug_tex_weight_lo, debug_tex_weight_hi, ...
     debug_post_rowmed, debug_spatial_cleaned, ...
     debug_td_zscore, debug_td_cleaned, ...
     debug_intensity_cleaned, ...
     debug_phase_cleaned, ...
     debug_gabor_dc, ...
     debug_dwt_approx] = ...
        sharpness_detection_with_mirage_suppression( ...
            gray, background_frame, background_ready);

    % --- 3. State machine (uses PREVIOUS frame's trajectory outputs) ---
    [current_state, processing_active, state_duration, onset_threshold] = ...
        flight_state_machine( ...
            frame_energy_scalar, noise_floor_ready, ...
            prev_tracking_active, prev_confidence, ...
            prev_path_x, prev_path_y);

    % --- 3b. Impact detection (independent, parallel with trail) ---
    [impact_x_cur, impact_y_cur, impact_det_cur, ~, splash_energy] = ...
        impact_detection(gray, background_frame, background_ready, ...
                         current_state, noise_floor_ready, ...
                         IMPACT_ROI_X, IMPACT_ROI_Y, IMPACT_ROI_RADIUS);
    if impact_det_cur
        impact_x_final        = impact_x_cur;
        impact_y_final        = impact_y_cur;
        impact_detected_final = true;
    end

    % --- 4. Trajectory integration (uses CURRENT state) ---
    [enhanced_map, path_x, path_y, tracking_active, confidence, ...
     ax, bx, cx, ay, by, cy, t_ref, ...
     debug_centroid_x, debug_centroid_y, ...
     debug_n_detections, debug_mode, debug_merged, ...
     debug_rejected, debug_change_max, ...
     debug_pca_elongation, debug_accum_count, ...
     debug_pca_pre_weight, debug_pca_aniso, debug_pca_post_weight] = ...
        trajectory_integrated_detection( ...
            clean_detection, frame_energy_scalar, ...
            noise_floor_ready, current_state, ...
            impact_x_final, impact_y_final, impact_detected_final);

    % --- 4b. Track first centroid (blob) and log all centroids ---
    if blob_x_final == 0 && debug_centroid_x > 0 && debug_n_detections >= 1
        blob_x_final = debug_centroid_x;
        blob_y_final = debug_centroid_y;
    end
    if debug_centroid_x > 0 && debug_centroid_y > 0 && centroid_log_len < MAX_CENTROID_LOG
        centroid_log_len = centroid_log_len + 1;
        centroid_log_x(centroid_log_len) = debug_centroid_x;
        centroid_log_y(centroid_log_len) = debug_centroid_y;
    end

    % --- 5. Path history buffer ---
    [path_xs, path_ys, path_confs, path_states, path_len] = ...
        path_history_buffer( ...
            path_x, path_y, confidence, current_state, tracking_active);

    % --- 6. Video overlay ---
    annotated_frame = video_overlay( ...
        rgb, ...
        path_xs, path_ys, path_confs, path_states, path_len, ...
        path_x, path_y, ...
        current_state, confidence, state_duration, ...
        ax, bx, cx, ay, by, cy, t_ref, ...
        impact_x_final, impact_y_final, impact_detected_final, ...
        blob_x_final, blob_y_final, ...
        centroid_log_x, centroid_log_y, centroid_log_len, ...
        clean_detection);

    % --- Write output ---
    writeVideo(writer, uint8(annotated_frame));

    % --- Capture mid-FLIGHT frame for debug comparison ---
    if current_state == 1 && tracking_active && ~compare_saved
        compare_wait = compare_wait + 1;
        if compare_wait >= 5
            imwrite(uint8(annotated_frame), fullfile(run_dir, 'debug_path_compare.png'));
            compare_saved = true;
        end
    end

    % --- Save detection map snapshots during flight ---
    if current_state >= 1 && flight_map_count < MAX_FLIGHT_MAPS
        flight_map_count = flight_map_count + 1;
        det_max = max(clean_detection(:));
        if det_max > 0
            det_img = uint8(clean_detection / det_max * 255);
        else
            det_img = uint8(clean_detection);
        end
        imwrite(det_img, fullfile(maps_det_dir, sprintf('det_%04d.png', frame_idx)));

        % Save intermediate detection stages for first 20 FLIGHT frames
        if flight_map_count <= 20
            fname = sprintf('%04d.png', frame_idx);
            % 1. Raw deficit (before texture gating)
            rd_max = max(debug_raw_deficit(:));
            if rd_max > 0
                imwrite(uint8(debug_raw_deficit / rd_max * 255), ...
                    fullfile(maps_raw_dir, fname));
            end
            % 2. Texture weight masks
            imwrite(uint8(debug_tex_weight * 255), ...
                fullfile(maps_texw_dir, fname));
            imwrite(uint8(debug_tex_weight_lo * 255), ...
                fullfile(maps_texlo_dir, fname));
            imwrite(uint8(debug_tex_weight_hi * 255), ...
                fullfile(maps_texhi_dir, fname));
            % 3. Z-score before row median subtraction
            pr_max = max(debug_pre_rowmed(:));
            if pr_max > 0
                imwrite(uint8(debug_pre_rowmed / pr_max * 255), ...
                    fullfile(maps_prerow_dir, fname));
            end
            % 4. Z-score after row median subtraction
            postr_max = max(debug_post_rowmed(:));
            if postr_max > 0
                imwrite(uint8(debug_post_rowmed / postr_max * 255), ...
                    fullfile(maps_postrow_dir, fname));
            end
            % 5. Spatial channel (after noise margin)
            sc_max = max(debug_spatial_cleaned(:));
            if sc_max > 0
                imwrite(uint8(debug_spatial_cleaned / sc_max * 255), ...
                    fullfile(maps_spatial_dir, fname));
            end
            % 6. Temporal diff z-score (after row median)
            tdz_max = max(debug_td_zscore(:));
            if tdz_max > 0
                imwrite(uint8(debug_td_zscore / tdz_max * 255), ...
                    fullfile(maps_tdz_dir, fname));
            end
            % 7. Temporal channel (after noise margin)
            tdc_max = max(debug_td_cleaned(:));
            if tdc_max > 0
                imwrite(uint8(debug_td_cleaned / tdc_max * 255), ...
                    fullfile(maps_tdc_dir, fname));
            end
            % 8. Intensity channel (after noise margin)
            ic_max = max(debug_intensity_cleaned(:));
            if ic_max > 0
                imwrite(uint8(debug_intensity_cleaned / ic_max * 255), ...
                    fullfile(maps_intc_dir, fname));
            end
            % 9. Gabor phase channel (after noise margin)
            pc_max = max(debug_phase_cleaned(:));
            if pc_max > 0
                imwrite(uint8(debug_phase_cleaned / pc_max * 255), ...
                    fullfile(maps_phase_dir, fname));
            end
            % 10. Gabor directional contrast (filter bank output)
            gdc_max = max(debug_gabor_dc(:));
            if gdc_max > 0
                imwrite(uint8(debug_gabor_dc / gdc_max * 255), ...
                    fullfile(maps_gabor_dir, fname));
            end
            % 11. DWT LL2 approximation (low-freq content removed)
            dwt_max = max(debug_dwt_approx(:));
            if dwt_max > 0
                imwrite(uint8(debug_dwt_approx / dwt_max * 255), ...
                    fullfile(maps_dwt_dir, fname));
            end
            % 12-14. PCA intermediate maps (only non-zero on PCA fire frame)
            pca_pre_max = max(debug_pca_pre_weight(:));
            if pca_pre_max > 0
                imwrite(uint8(debug_pca_pre_weight / pca_pre_max * 255), ...
                    fullfile(maps_pca_pre_dir, fname));
            end
            pca_aniso_max = max(debug_pca_aniso(:));
            if pca_aniso_max > 0
                imwrite(uint8(debug_pca_aniso / pca_aniso_max * 255), ...
                    fullfile(maps_pca_aniso_dir, fname));
            end
            pca_post_max = max(debug_pca_post_weight(:));
            if pca_post_max > 0
                imwrite(uint8(debug_pca_post_weight / pca_post_max * 255), ...
                    fullfile(maps_pca_post_dir, fname));
            end
        end
    end

    % --- Log signals ---
    if frame_idx <= num_frames
        log.frame_energy_scalar(frame_idx) = frame_energy_scalar;
        log.current_state(frame_idx)       = current_state;
        log.confidence(frame_idx)          = confidence;
        log.path_x(frame_idx)             = path_x;
        log.path_y(frame_idx)             = path_y;
        log.noise_floor_ready(frame_idx)   = noise_floor_ready;
        log.tracking_active(frame_idx)     = tracking_active;
        log.path_len(frame_idx)           = path_len;
        log.onset_threshold(frame_idx)     = onset_threshold;
        log.background_ready(frame_idx)    = background_ready;
        log.state_duration(frame_idx)      = state_duration;
        log.processing_active(frame_idx)   = processing_active;
        log.ax(frame_idx) = ax; log.bx(frame_idx) = bx; log.cx(frame_idx) = cx;
        log.ay(frame_idx) = ay; log.by(frame_idx) = by; log.cy(frame_idx) = cy;
        log.t_ref(frame_idx)              = t_ref;
        log.detection_max(frame_idx)       = max(clean_detection(:));
        log.detection_mean(frame_idx)      = mean(clean_detection(:));
        log.debug_centroid_x(frame_idx)    = debug_centroid_x;
        log.debug_centroid_y(frame_idx)    = debug_centroid_y;
        log.impact_x(frame_idx)            = impact_x_final;
        log.impact_y(frame_idx)            = impact_y_final;
        log.impact_detected(frame_idx)     = impact_detected_final;
        log.splash_energy(frame_idx)       = splash_energy;
        log.debug_n_detections(frame_idx)  = debug_n_detections;
        log.debug_mode(frame_idx)          = debug_mode;
        log.debug_merged(frame_idx)        = debug_merged;
        log.debug_rejected(frame_idx)      = debug_rejected;
        log.debug_change_max(frame_idx)    = debug_change_max;
        log.debug_pca_elongation(frame_idx) = debug_pca_elongation;
        log.debug_accum_count(frame_idx)    = debug_accum_count;
    end

    % --- Save for next iteration ---
    prev_tracking_active = tracking_active;
    prev_confidence      = confidence;
    prev_path_x          = path_x;
    prev_path_y          = path_y;

    % --- Progress ---
    if mod(frame_idx, 30) == 0
        fprintf('Frame %d / ~%d  |  state=%d  conf=%.4f  path=(%.0f,%.0f)\n', ...
            frame_idx, num_frames, current_state, confidence, path_x, path_y);
    end
end

close(writer);
fprintf('Done. %d frames written to %s\n', frame_idx, output_video);
fprintf('%d detection maps saved to %s\n', flight_map_count, maps_dir);

% Trim log in case num_frames estimate was off
if frame_idx < num_frames
    fields = fieldnames(log);
    for i = 1:numel(fields)
        log.(fields{i}) = log.(fields{i})(1:frame_idx);
    end
end

%% ===================== SAVE & ANALYZE =====================
save_log(log, frame_idx, run_dir);
analyze_video(log, run_dir);

%% ===================== CLOSE CONSOLE LOG =====================
diary off;
