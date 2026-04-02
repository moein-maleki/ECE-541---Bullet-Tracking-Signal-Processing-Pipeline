function [enhanced_map, path_x, path_y, tracking_active, confidence, ...
          ax_out, bx_out, cx_out, ay_out, by_out, cy_out, t_ref_out, ...
          debug_centroid_x, debug_centroid_y, ...
          debug_n_detections, debug_mode, debug_merged, ...
          debug_rejected, debug_change_max, ...
          debug_pca_elongation, debug_accum_count, ...
          debug_pca_pre_weight, debug_pca_aniso, debug_pca_post_weight] = ...
    trajectory_integrated_detection(clean_detection, frame_energy_scalar, ...
                                       noise_floor_ready, current_state, ...
                                       impact_x, impact_y, impact_detected)
%#codegen
% TRAJECTORY_INTEGRATED_DETECTION
%
% Modes:
%   0 — Waiting for flight onset (from state machine)
%   1 — Near-field: anisotropy-weighted PCA trail axis extraction,
%       then quadratic fit through 3 seed points
%   2 — Far-field: trajectory-guided integration and refinement
%
% Inputs:
%   clean_detection    : HxW detection map from sharpness detector
%   frame_energy_scalar: scalar energy of current frame
%   noise_floor_ready  : logical, noise baseline calibrated
%   current_state      : int32, state machine output (0-3)
%   impact_x, impact_y : double, detected impact location
%   impact_detected    : logical, whether impact has been confirmed
%
% Key design:
%   - Mode 1 accumulates baseline-subtracted detection maps, then
%     applies intensity capping + morphological line filtering
%     to isolate elongated linear features (trail) from blobs/noise.
%     PCA on the filtered map extracts the trail axis and produces all
%     3 seed points for the quadratic fit.
%   - Onset cooldown disabled (ONSET_COOLDOWN_MAX=0) — no muzzle
%     blast in this video.
%   - t_ref set when PCA fires, not at mode transition.

% ===================== PARAMETERS =====================
N_INIT            = 3;       % distinct key points before quadratic fit
INTEGRATION_LEN   = 5;       % frames for trajectory-guided integration
PATH_SAMPLE_WIDTH = 30;      % perpendicular sampling width (pixels)
GRAVITY_PRIOR     = 0;       % disabled — trail is nearly straight in this video
CENTROID_TOP_FRAC = 0.02;    % top fraction of pixels for centroid
MIN_CLUSTER_PIXELS= 20;      % minimum bright pixels for valid centroid

% Onset cooldown: DISABLED — no muzzle blast in this video.
% The bright blob at frame onset IS the trail entry point.
ONSET_COOLDOWN_FRAC = 0.5;
ONSET_COOLDOWN_MAX  = 0;   % 0 = skip cooldown entirely

% PCA trail axis extraction with morphological line filtering.
% Accumulate detection maps, then cap intensity and apply morphological
% opening with elongated line structuring elements at multiple angles.
% Only features matching the element's length AND angle survive —
% this tests GLOBAL linear extent, not local orientation.
PCA_ACCUM_FRAMES      = 5;     % frames to accumulate before PCA
TRAIL_THRESH_FRAC     = 0.15;  % bright pixel threshold (fraction of max)
MIN_TRAIL_PIXELS      = 100;   % min bright pixels for valid trail
TRAIL_ELONGATION_MIN  = 2.0;   % min eigenvalue ratio (lambda1/lambda2)
TRAIL_DURATION        = 20;    % pseudo-time span for PCA points (frames)
INTENSITY_CAP_PCTILE  = 0.80;  % cap accumulated map at this percentile
MORPH_LINE_LEN        = 80;    % structuring element length (pixels)
MORPH_N_ANGLES        = 12;    % number of angles to test (0 to 165 deg)
PCA_SPATIAL_MARGIN    = 400;   % keep pixels within this X-range of peak column
MHAT_R_DEFAULT        = 10;    % default Mexican Hat radius (pixels, full-res)
MHAT_R_MIN            = 5;     % minimum adaptive radius
MHAT_R_MAX            = 20;    % maximum adaptive radius
MHAT_DENSITY_PCTILE   = 0.50;  % percentile for HWHM radius estimation

% Centroid deduplication: if a new centroid is within this radius of
% an existing key point, merge instead of adding.
MIN_CENTROID_DIST = 80;  % pixels

% ===================== PERSISTENT STATE =====================
persistent mode
persistent frame_count

persistent centroid_xs centroid_ys centroid_frames
persistent n_detections

persistent ax bx cx ay by cy
persistent t_ref

persistent evidence_buffer buf_write_ptr integrated_map

% Onset cooldown state
persistent mode1_peak_energy cooldown_done mode1_cooldown_count

% Temporal change map: previous detection for frame differencing
persistent prev_detection

% Trajectory freeze: once IMPACT or trail lost, lock coefficients
persistent trajectory_frozen
persistent impact_used

% PCA accumulation state
persistent accum_map accum_count pca_done
persistent baseline_detection  % last pre-flight detection (for background subtraction)

[H, W] = size(clean_detection);

% ===================== INITIALIZATION =====================
if isempty(mode)
    mode = int32(0);
    frame_count = int32(0);

    centroid_xs     = zeros(N_INIT, 1);
    centroid_ys     = zeros(N_INIT, 1);
    centroid_frames = zeros(N_INIT, 1);
    n_detections    = int32(0);

    ax = 0; bx = 0; cx = 0;
    ay = 0; by = 0; cy = 0;
    t_ref = int32(0);

    evidence_buffer = zeros(INTEGRATION_LEN, H, W);
    buf_write_ptr = int32(1);
    integrated_map = zeros(H, W);

    mode1_peak_energy    = 0;
    cooldown_done        = false;
    mode1_cooldown_count = int32(0);

    prev_detection = zeros(H, W);

    trajectory_frozen = false;
    impact_used       = false;

    accum_map   = zeros(H, W);
    accum_count = int32(0);
    pca_done    = false;
    baseline_detection = zeros(H, W);
end

frame_count = frame_count + int32(1);

% ===================== DEFAULT OUTPUTS =====================
enhanced_map     = clean_detection;
path_x           = 0;
path_y           = 0;
tracking_active  = false;
confidence       = 0;
debug_centroid_x = 0;
debug_centroid_y = 0;
debug_n_detections = double(n_detections);
debug_mode         = double(mode);
debug_merged       = 0;
debug_rejected     = 0;
debug_change_max   = 0;
debug_pca_elongation = 0;
debug_accum_count    = double(accum_count);
debug_pca_pre_weight  = zeros(H, W);
debug_pca_aniso       = zeros(H, W);
debug_pca_post_weight = zeros(H, W);

if ~noise_floor_ready
    ax_out = ax; bx_out = bx; cx_out = cx;
    ay_out = ay; by_out = by; cy_out = cy;
    t_ref_out = t_ref;
    return;
end

% ===================== IMPACT REFIT (one-shot) =====================
% When the impact location becomes available, add it as the final
% point in the trajectory fit and refit the quadratic.  This anchors
% the far end of the trajectory to the physical impact site.
if impact_detected && ~impact_used && n_detections >= int32(1) && t_ref > int32(0)
    % Add impact point to the centroid arrays
    if n_detections < int32(N_INIT)
        n_detections = n_detections + int32(1);
    else
        % Shift oldest out to make room
        for i = 1:N_INIT-1
            centroid_xs(i) = centroid_xs(i+1);
            centroid_ys(i) = centroid_ys(i+1);
            centroid_frames(i) = centroid_frames(i+1);
        end
    end
    idx = min(double(n_detections), N_INIT);
    centroid_xs(idx) = impact_x;
    centroid_ys(idx) = impact_y;
    centroid_frames(idx) = double(frame_count);  % approximate frame

    % Refit trajectory with impact endpoint
    if n_detections >= int32(3)
        nn = min(double(n_detections), N_INIT);
        t_vals = centroid_frames(1:nn) - double(t_ref);
        x_vals = centroid_xs(1:nn);
        y_vals = centroid_ys(1:nn);
        A_mat_imp = zeros(nn, 3);
        for i = 1:nn
            tv = t_vals(i);
            A_mat_imp(i,:) = [tv*tv, tv, 1];
        end
        ATA_imp = A_mat_imp' * A_mat_imp;
        ATA_imp(1,1) = ATA_imp(1,1) + 1e-6;
        ATA_imp(2,2) = ATA_imp(2,2) + 1e-6;
        ATA_imp(3,3) = ATA_imp(3,3) + 1e-6;
        px_imp = solve_3x3(ATA_imp, A_mat_imp' * x_vals);
        ax = px_imp(1); bx = px_imp(2); cx = px_imp(3);
        py_imp = solve_3x3(ATA_imp, A_mat_imp' * y_vals);
        ay = py_imp(1); by = py_imp(2); cy = py_imp(3);
    end

    impact_used = true;
    trajectory_frozen = true;
    mode = int32(2);
    tracking_active = true;
end

% ===================== FROZEN TRAJECTORY (post-IMPACT) =====================
% After IMPACT state or once trajectory is frozen, stop all centroid
% collection and path accumulation.  The trajectory coefficients are
% locked — the video overlay can still draw the curve from them, but
% we do NOT feed more points to the path history buffer.
if trajectory_frozen || current_state >= int32(3)
    trajectory_frozen = true;
    tracking_active = false;   % STOP path history accumulation
    path_x = 0;               % no new path points
    path_y = 0;
    confidence = 0;

    enhanced_map = clean_detection;
    ax_out = ax; bx_out = bx; cx_out = cx;
    ay_out = ay; by_out = by; cy_out = cy;
    t_ref_out = t_ref;
    return;
end

% ===================== MODE 0: WAITING =====================
if mode == int32(0)
    if current_state >= int32(1)
        % Transition to Mode 1.
        % Save PREVIOUS frame's detection as pre-flight baseline
        % for background subtraction in PCA accumulation.
        % prev_detection holds frame N-1's map (set last call).
        % Current frame (N) already contains the trail onset, so
        % we must use the PRIOR frame as the clean baseline.
        baseline_detection = prev_detection;
        mode = int32(1);
        mode1_peak_energy    = frame_energy_scalar;
        cooldown_done        = false;
        mode1_cooldown_count = int32(0);
    end

    % Buffer detection for temporal differencing (AFTER baseline save)
    prev_detection = clean_detection;
    ax_out = ax; bx_out = bx; cx_out = cx;
    ay_out = ay; by_out = by; cy_out = cy;
    t_ref_out = t_ref;
    return;
end

% ===================== MODE 1: NEAR-FIELD DETECTION =====================
if mode == int32(1)

    % --- Temporal change map (for debug only) ---
    change_map = max(clean_detection - prev_detection, 0);
    prev_detection = clean_detection;
    debug_change_max = max(change_map(:));

    % --- Onset cooldown ---
    if frame_energy_scalar > mode1_peak_energy
        mode1_peak_energy = frame_energy_scalar;
    end

    if ~cooldown_done
        mode1_cooldown_count = mode1_cooldown_count + int32(1);
        if frame_energy_scalar < ONSET_COOLDOWN_FRAC * mode1_peak_energy || ...
           mode1_cooldown_count >= int32(ONSET_COOLDOWN_MAX)
            cooldown_done = true;
        else
            ax_out = ax; bx_out = bx; cx_out = cx;
            ay_out = ay; by_out = by; cy_out = cy;
            t_ref_out = t_ref;
            return;
        end
    end

    % ====== Accumulate detection maps for morphological-filtered PCA ======
    diff_map = max(clean_detection - baseline_detection, 0);
    accum_map = accum_map + diff_map;
    accum_count = accum_count + int32(1);
    debug_accum_count = double(accum_count);

    if ~pca_done && accum_count >= int32(PCA_ACCUM_FRAMES)
        % --- Intensity cap: flatten the blob to trail level ---
        % Histogram-based percentile cap (codegen-safe, no sort).
        cap_val = percentile_cap(accum_map, H, W, INTENSITY_CAP_PCTILE);
        capped = zeros(H, W);
        for r = 1:H
            for c = 1:W
                if accum_map(r,c) > cap_val
                    capped(r,c) = cap_val;
                else
                    capped(r,c) = accum_map(r,c);
                end
            end
        end

        % --- Morphological line filtering ---
        % Tests GLOBAL linear extent: a morphological opening with an
        % elongated line structuring element removes any feature shorter
        % than the element.  Blobs, noise, and short edges are erased;
        % only structures matching the line length AND angle survive.
        % We test MORPH_N_ANGLES angles, keep the max response (best
        % angle), which preserves the trail regardless of its orientation.
        morph_filtered = morph_line_open(capped, H, W, ...
            MORPH_LINE_LEN, MORPH_N_ANGLES);

        % --- Spatial mask: keep only the region around peak signal ---
        % The trail is a localized feature. Vegetation edges at the
        % treeline and codec artifacts elsewhere can survive the morph
        % filter because they are long enough. Find the column with
        % the highest total signal in the morph-filtered map and mask
        % to within PCA_SPATIAL_MARGIN pixels of it. This excludes
        % distant contamination (treeline at left, etc.) without
        % hard-coding a frame region.
        col_sum = zeros(W, 1);
        for c = 1:W
            for r = 1:H
                col_sum(c) = col_sum(c) + morph_filtered(r, c);
            end
        end
        peak_col = 1;
        peak_val = col_sum(1);
        for c = 2:W
            if col_sum(c) > peak_val
                peak_val = col_sum(c);
                peak_col = c;
            end
        end
        col_lo = max(1, peak_col - PCA_SPATIAL_MARGIN);
        col_hi = min(W, peak_col + PCA_SPATIAL_MARGIN);
        masked = zeros(H, W);
        for r = 1:H
            for c = col_lo:col_hi
                masked(r, c) = morph_filtered(r, c);
            end
        end

        % --- Mexican Hat radial filter: suppress features wider than trail ---
        % Estimate the trail radius from the peak region of the masked
        % map, then convolve with a Mexican Hat (LoG) kernel of that
        % radius.  Features matching the trail width get positive
        % response; features wider than the trail (blob) get negative
        % response and are suppressed below zero.
        r_opt = estimate_trail_radius(masked, H, W, ...
            MHAT_DENSITY_PCTILE, MHAT_R_MIN, MHAT_R_MAX, MHAT_R_DEFAULT);
        mhat_out = mexican_hat_filter(masked, H, W, r_opt);

        weighted_pca_map = mhat_out;

        % --- Debug: PCA intermediate maps ---
        debug_pca_pre_weight  = capped;             % intensity-capped accum (before filtering)
        debug_pca_aniso       = morph_filtered;      % morphological line filter output
        debug_pca_post_weight = mhat_out;            % morph + mask + Mexican Hat (actual PCA input)

        % --- Run PCA on the filtered map ---
        [pca_pts_x, pca_pts_y, pca_n, pca_elong, pca_ok] = ...
            find_trail_axis(weighted_pca_map, H, W, ...
                TRAIL_THRESH_FRAC, MIN_TRAIL_PIXELS, ...
                TRAIL_ELONGATION_MIN, impact_x, impact_y);

        debug_pca_elongation = pca_elong;
        pca_done = true;

        if pca_ok && pca_n == 3
            % PCA extracts 3 points along the trail axis:
            %   (1) near-field entry, (2) centroid, (3) far-field end
            n_detections = int32(3);
            t_ref = frame_count;
            centroid_xs(1) = pca_pts_x(1);
            centroid_ys(1) = pca_pts_y(1);
            centroid_frames(1) = double(t_ref);
            centroid_xs(2) = pca_pts_x(2);
            centroid_ys(2) = pca_pts_y(2);
            centroid_frames(2) = double(t_ref) + TRAIL_DURATION / 2;
            centroid_xs(3) = pca_pts_x(3);
            centroid_ys(3) = pca_pts_y(3);
            centroid_frames(3) = double(t_ref) + TRAIL_DURATION;

            path_x = pca_pts_x(2);
            path_y = pca_pts_y(2);
            confidence = 1.0;

            debug_centroid_x = pca_pts_x(2);
            debug_centroid_y = pca_pts_y(2);
        end
    end

    if ~pca_done
        % Still accumulating — no centroids yet
        debug_n_detections = double(n_detections);
        debug_mode = double(mode);
        ax_out = ax; bx_out = bx; cx_out = cx;
        ay_out = ay; by_out = by; cy_out = cy;
        t_ref_out = t_ref;
        return;
    end

    debug_n_detections = double(n_detections);
    debug_mode         = double(mode);

    if n_detections >= int32(N_INIT)
        % Fit quadratic trajectory
        t_vals = centroid_frames(1:N_INIT) - double(t_ref);
        x_vals = centroid_xs(1:N_INIT);
        y_vals = centroid_ys(1:N_INIT);

        A_mat = zeros(N_INIT, 3);
        for i = 1:N_INIT
            t = t_vals(i);
            A_mat(i,:) = [t*t, t, 1];
        end

        ATA = A_mat' * A_mat;
        ATA(1,1) = ATA(1,1) + 1e-6;
        ATA(2,2) = ATA(2,2) + 1e-6;
        ATA(3,3) = ATA(3,3) + 1e-6;

        px = solve_3x3(ATA, A_mat' * x_vals);
        ax = px(1); bx = px(2); cx = px(3);

        py = solve_3x3(ATA, A_mat' * y_vals);
        ay = py(1); by = py(2); cy = py(3);

        if ay < 0.01
            ay = GRAVITY_PRIOR;
        end

        % --- Fit RMSE quality check ---
        % Compute root-mean-square error of the 3-point fit.
        fit_rmse = 0;
        for i = 1:N_INIT
            tv = t_vals(i);
            ex = ax*tv*tv + bx*tv + cx - x_vals(i);
            ey = ay*tv*tv + by*tv + cy - y_vals(i);
            fit_rmse = fit_rmse + ex*ex + ey*ey;
        end
        fit_rmse = sqrt(fit_rmse / double(N_INIT));
        % Note: with exactly 3 points and 3 coefficients per axis,
        % RMSE should be near-zero (perfect fit). A non-zero RMSE
        % would indicate numerical issues.

        % Entry-point extrapolation REMOVED.
        % The PCA near-field and far-field endpoints already define
        % the trail bounds. Extrapolating backward to the frame edge
        % caused massive over-extrapolation (full-frame trajectory
        % from a ~280px trail). The trail is a static structure;
        % the PCA endpoints are sufficient.

        mode = int32(2);
        tracking_active = true;
    end

    enhanced_map = clean_detection;
    ax_out = ax; bx_out = bx; cx_out = cx;
    ay_out = ay; by_out = by; cy_out = cy;
    t_ref_out = t_ref;
    return;
end

% ===================== MODE 2: TRAJECTORY-GUIDED TRACKING =====================
if mode == int32(2)
    prev_detection = clean_detection;  % keep buffer current
    tracking_active = true;

    t_raw = double(frame_count - t_ref);
    % Clamp t to the PCA data range [0, TRAIL_DURATION].
    % The PCA points define the trail extent; evaluating outside
    % this range is pure extrapolation into empty space.
    t = max(0, min(t_raw, TRAIL_DURATION));
    pred_x = ax*t*t + bx*t + cx;
    pred_y = ay*t*t + by*t + cy;

    pred_x = max(1, min(W, pred_x));
    pred_y = max(1, min(H, pred_y));

    path_x = pred_x;
    path_y = pred_y;

    % Store in evidence buffer
    evidence_buffer(buf_write_ptr, :, :) = clean_detection;
    buf_write_ptr = int32(mod(double(buf_write_ptr), INTEGRATION_LEN) + 1);

    % Trajectory-aligned integration
    integrated_map = zeros(H, W);

    for k = 0:INTEGRATION_LEN-1
        slot = mod(double(buf_write_ptr) - 1 - k - 1, INTEGRATION_LEN) + 1;

        t_past_raw = double(frame_count - t_ref) - k;
        t_past = max(0, min(t_past_raw, TRAIL_DURATION));
        past_x = ax*t_past*t_past + bx*t_past + cx;
        past_y = ay*t_past*t_past + by*t_past + cy;

        half_w = round(PATH_SAMPLE_WIDTH / 2);

        dx_dt = 2*ax*t_past + bx;
        dy_dt = 2*ay*t_past + by;
        speed = sqrt(max(dx_dt^2 + dy_dt^2, 0)) + 1e-10;

        perp_x = -dy_dt / speed;
        perp_y =  dx_dt / speed;

        stored_frame = squeeze(evidence_buffer(slot, :, :));

        for s = -half_w:half_w
            sx = round(past_x + s * perp_x);
            sy = round(past_y + s * perp_y);

            if sx >= 1 && sx <= W && sy >= 1 && sy <= H
                cx_map = round(pred_x + s * perp_x);
                cy_map = round(pred_y + s * perp_y);
                if cx_map >= 1 && cx_map <= W && cy_map >= 1 && cy_map <= H
                    integrated_map(cy_map, cx_map) = ...
                        integrated_map(cy_map, cx_map) + stored_frame(sy, sx);
                end
            end
        end
    end

    integrated_map = integrated_map / INTEGRATION_LEN;

    % Confidence at predicted location
    px_r = round(pred_x);
    py_r = round(pred_y);
    sample_r = 15;
    conf_sum = 0;
    conf_count = 0;
    for dr = -sample_r:sample_r
        for dc = -sample_r:sample_r
            rr = py_r + dr;
            cc = px_r + dc;
            if rr >= 1 && rr <= H && cc >= 1 && cc <= W
                conf_sum = conf_sum + integrated_map(rr, cc);
                conf_count = conf_count + 1;
            end
        end
    end
    if conf_count > 0
        confidence = conf_sum / conf_count;
    else
        confidence = 0;
    end

    % Try to refine trajectory if strong detection near prediction
    [cy_det, cx_det, det_conf] = find_centroid_in_roi( ...
        clean_detection, pred_x, pred_y, 80, ...
        CENTROID_TOP_FRAC, MIN_CLUSTER_PIXELS, H, W);

    debug_centroid_x = cx_det;
    debug_centroid_y = cy_det;

    % Mode 2 trajectory refinement DISABLED.
    % The initial Mode 1 fit + impact endpoint refit is sufficient.
    % Continuing to update with weak detections causes the trajectory
    % to drift as the trail signal weakens with distance.

    enhanced_map = clean_detection;
    debug_n_detections = double(n_detections);
    debug_mode         = double(mode);
end

ax_out = ax; bx_out = bx; cx_out = cx;
ay_out = ay; by_out = by; cy_out = cy;
t_ref_out = t_ref;

end


% ================================================================
% PCA TRAIL AXIS EXTRACTION
% ================================================================
function [pts_x, pts_y, n_pts, elongation, success] = find_trail_axis( ...
    accumulated_map, H, W, thresh_frac, min_pixels, min_elongation, ...
    impact_x, impact_y)
% Extracts the trail's principal axis from an accumulated detection map.
% Returns 3 points along the axis: near-field endpoint, centroid, far-field.
% Uses intensity-weighted PCA with closed-form 2x2 eigendecomposition.

    pts_x = zeros(3, 1);
    pts_y = zeros(3, 1);
    n_pts = 0;
    elongation = 0;
    success = false;

    % --- Step 1: Threshold to isolate bright pixels ---
    max_val = 0;
    for r = 1:H
        for c = 1:W
            if accumulated_map(r, c) > max_val
                max_val = accumulated_map(r, c);
            end
        end
    end

    if max_val < 1e-10
        return;
    end

    thresh = thresh_frac * max_val;

    % --- Step 2: Intensity-weighted centroid ---
    sum_x = 0; sum_y = 0; sum_w = 0;
    bright_count = 0;
    for r = 1:H
        for c = 1:W
            if accumulated_map(r, c) > thresh
                w = accumulated_map(r, c);
                sum_x = sum_x + double(c) * w;
                sum_y = sum_y + double(r) * w;
                sum_w = sum_w + w;
                bright_count = bright_count + 1;
            end
        end
    end

    if bright_count < min_pixels || sum_w < 1e-10
        return;
    end

    mean_x = sum_x / sum_w;
    mean_y = sum_y / sum_w;

    % --- Step 3: Intensity-weighted 2x2 covariance ---
    cov_xx = 0; cov_yy = 0; cov_xy = 0;
    for r = 1:H
        for c = 1:W
            if accumulated_map(r, c) > thresh
                w = accumulated_map(r, c);
                dx = double(c) - mean_x;
                dy = double(r) - mean_y;
                cov_xx = cov_xx + w * dx * dx;
                cov_yy = cov_yy + w * dy * dy;
                cov_xy = cov_xy + w * dx * dy;
            end
        end
    end
    cov_xx = cov_xx / sum_w;
    cov_yy = cov_yy / sum_w;
    cov_xy = cov_xy / sum_w;

    % --- Step 4: Closed-form eigendecomposition of 2x2 symmetric ---
    tr = cov_xx + cov_yy;
    det_val = cov_xx * cov_yy - cov_xy * cov_xy;
    disc = tr * tr / 4 - det_val;
    if disc < 0
        disc = 0;
    end
    sqrt_disc = sqrt(disc);
    lambda1 = tr / 2 + sqrt_disc;  % larger eigenvalue
    lambda2 = tr / 2 - sqrt_disc;  % smaller eigenvalue

    % Elongation check
    if lambda2 > 1e-10
        elongation = lambda1 / lambda2;
    else
        elongation = 1e6;  % degenerate = very elongated (line)
    end

    if elongation < min_elongation
        return;  % not elongated enough — blob, not trail
    end

    % Principal eigenvector (for lambda1)
    if abs(cov_xy) > 1e-10
        v1_x = lambda1 - cov_yy;
        v1_y = cov_xy;
    elseif cov_xx >= cov_yy
        v1_x = 1; v1_y = 0;
    else
        v1_x = 0; v1_y = 1;
    end
    v_len = sqrt(v1_x * v1_x + v1_y * v1_y) + 1e-10;
    v1_x = v1_x / v_len;
    v1_y = v1_y / v_len;

    % --- Step 5: Project bright pixels onto axis, find endpoints ---
    % Use histogram to find 5th and 95th percentile projections.
    N_BINS = 100;
    proj_min_raw = 1e10;
    proj_max_raw = -1e10;

    % First pass: find projection range
    for r = 1:H
        for c = 1:W
            if accumulated_map(r, c) > thresh
                proj = (double(c) - mean_x) * v1_x + (double(r) - mean_y) * v1_y;
                if proj < proj_min_raw
                    proj_min_raw = proj;
                end
                if proj > proj_max_raw
                    proj_max_raw = proj;
                end
            end
        end
    end

    proj_range = proj_max_raw - proj_min_raw;
    if proj_range < 1
        return;
    end

    bin_width = proj_range / N_BINS;
    bin_weights = zeros(N_BINS, 1);

    % Second pass: build weighted histogram of projections
    for r = 1:H
        for c = 1:W
            if accumulated_map(r, c) > thresh
                proj = (double(c) - mean_x) * v1_x + (double(r) - mean_y) * v1_y;
                b = min(N_BINS, max(1, ceil((proj - proj_min_raw) / bin_width)));
                bin_weights(b) = bin_weights(b) + accumulated_map(r, c);
            end
        end
    end

    % Find 5th and 95th percentile
    total_weight = sum(bin_weights);
    cum = 0;
    p5_bin = 1;
    p95_bin = N_BINS;
    for b = 1:N_BINS
        cum = cum + bin_weights(b);
        if cum >= 0.05 * total_weight
            p5_bin = b;
            break;
        end
    end
    cum = 0;
    for b = N_BINS:-1:1
        cum = cum + bin_weights(b);
        if cum >= 0.05 * total_weight
            p95_bin = b;
            break;
        end
    end

    proj_near = proj_min_raw + (p5_bin - 0.5) * bin_width;
    proj_far  = proj_min_raw + (p95_bin - 0.5) * bin_width;

    % Endpoints in image coordinates
    end1_x = mean_x + proj_near * v1_x;
    end1_y = mean_y + proj_near * v1_y;
    end2_x = mean_x + proj_far * v1_x;
    end2_y = mean_y + proj_far * v1_y;

    % Check minimum extent
    extent = sqrt((end2_x - end1_x)^2 + (end2_y - end1_y)^2);
    if extent < 50
        return;  % trail too short for reliable 3-point fit
    end

    % --- Step 6: Orient — trail entry (t=0) is FARTHER from impact ---
    % The bullet enters from one side and impacts on the other.
    % The trail entry point (blob) is at the far side from impact.
    % The trail end (closer to impact) is where the bullet is heading.
    % So: entry (t=0) = FARTHER from impact, end (t=max) = CLOSER.
    if impact_x > 0 && impact_y > 0
        d1 = (end1_x - impact_x)^2 + (end1_y - impact_y)^2;
        d2 = (end2_x - impact_x)^2 + (end2_y - impact_y)^2;
        if d1 > d2
            % end1 is farther from impact = trail entry (t=0)
            near_x = end1_x; near_y = end1_y;
            far_x  = end2_x; far_y  = end2_y;
        else
            near_x = end2_x; near_y = end2_y;
            far_x  = end1_x; far_y  = end1_y;
        end
    else
        % Heuristic: trail entry has larger Y (lower in frame = blob)
        if end1_y > end2_y
            near_x = end1_x; near_y = end1_y;
            far_x  = end2_x; far_y  = end2_y;
        else
            near_x = end2_x; near_y = end2_y;
            far_x  = end1_x; far_y  = end1_y;
        end
    end

    % Clamp to frame bounds
    near_x = max(1, min(W, near_x));
    near_y = max(1, min(H, near_y));
    far_x  = max(1, min(W, far_x));
    far_y  = max(1, min(H, far_y));

    % --- Output 3 points: near-field, centroid, far-field ---
    pts_x(1) = near_x;  pts_y(1) = near_y;
    pts_x(2) = mean_x;  pts_y(2) = mean_y;
    pts_x(3) = far_x;   pts_y(3) = far_y;
    n_pts = 3;
    success = true;
end


% ================================================================
% CENTER-SURROUND BLOB DETECTOR (integral-image contrast filter)
% ================================================================
function [cy, cx, conf, best_contrast] = find_densest_cluster(map, half_w, ...
                                    top_frac, min_pixels, H, W)
    % Finds the most ISOLATED bright cluster using center-surround
    % contrast (Difference-of-Boxes blob detector).
    %
    % For each pixel, computes:
    %   inner_mean  = mean intensity within half_w radius
    %   outer_mean  = mean intensity in annulus from half_w to 2*half_w
    %   contrast    = inner_mean - outer_mean
    %
    % High contrast = isolated bright cluster surrounded by dark areas.
    % Low contrast  = diffuse texture (vegetation) with no clear border.
    %
    % Uses integral image for O(H*W) computation of both box sizes.

    outer_hw = 2 * half_w;  % outer box half-width

    % --- Step 1: Integral image ---
    integral_img = zeros(H+1, W+1);
    for r = 1:H
        row_sum = 0;
        for c = 1:W
            row_sum = row_sum + map(r, c);
            integral_img(r+1, c+1) = integral_img(r, c+1) + row_sum;
        end
    end

    % --- Step 2: Find peak of center-surround contrast ---
    best_contrast = -1e10;
    best_r = 1;
    best_c = 1;
    for r = 1:H
        % Inner box bounds
        ir_lo = max(1, r - half_w);
        ir_hi = min(H, r + half_w);
        % Outer box bounds
        or_lo = max(1, r - outer_hw);
        or_hi = min(H, r + outer_hw);

        for c = 1:W
            ic_lo = max(1, c - half_w);
            ic_hi = min(W, c + half_w);
            oc_lo = max(1, c - outer_hw);
            oc_hi = min(W, c + outer_hw);

            % Inner box sum
            inner_sum = integral_img(ir_hi+1, ic_hi+1) ...
                      - integral_img(ir_lo, ic_hi+1) ...
                      - integral_img(ir_hi+1, ic_lo) ...
                      + integral_img(ir_lo, ic_lo);
            inner_area = (ir_hi - ir_lo + 1) * (ic_hi - ic_lo + 1);

            % Outer box sum (full large box)
            outer_sum_full = integral_img(or_hi+1, oc_hi+1) ...
                           - integral_img(or_lo, oc_hi+1) ...
                           - integral_img(or_hi+1, oc_lo) ...
                           + integral_img(or_lo, oc_lo);
            outer_area_full = (or_hi - or_lo + 1) * (oc_hi - oc_lo + 1);

            % Annulus = outer - inner
            surround_sum  = outer_sum_full - inner_sum;
            surround_area = outer_area_full - inner_area;

            % Center-surround contrast
            inner_mean = inner_sum / max(inner_area, 1);
            if surround_area > 0
                surround_mean = surround_sum / surround_area;
            else
                surround_mean = 0;
            end
            contrast = inner_mean - surround_mean;

            if contrast > best_contrast
                best_contrast = contrast;
                best_r = r;
                best_c = c;
            end
        end
    end

    if best_contrast < 1e-10
        cy = 0; cx = 0; conf = 0;
        return;
    end

    % --- Step 3: Centroid within the best cluster ---
    [cy, cx, conf] = find_centroid_in_roi(map, double(best_c), double(best_r), ...
        half_w, top_frac, min_pixels, H, W);
    % Return the contrast so caller can threshold
end


% ================================================================
% PEAK-LOCAL CENTROID FINDER (full frame)
% ================================================================
function [cy, cx, conf] = find_centroid_peak_local(map, search_radius, ...
                                    top_frac, min_pixels, H, W)
    % Find the single brightest pixel in the detection map, then
    % compute a weighted centroid within search_radius around it.
    % This prevents distant artifacts from pulling the centroid.

    max_val = 0;
    peak_r = 1;
    peak_c = 1;
    for r = 1:H
        for c = 1:W
            if map(r,c) > max_val
                max_val = map(r,c);
                peak_r = r;
                peak_c = c;
            end
        end
    end

    if max_val < 1e-10
        cy = 0; cx = 0; conf = 0;
        return;
    end

    % Centroid within local ROI around the peak
    [cy, cx, conf] = find_centroid_in_roi(map, double(peak_c), double(peak_r), ...
        search_radius, top_frac, min_pixels, H, W);
end


% ================================================================
% ADAPTIVE CENTROID FINDER (within ROI)
% ================================================================
function [cy, cx, conf] = find_centroid_in_roi(map, pred_x, pred_y, ...
                                    radius, top_frac, min_pixels, H, W)
    r_lo = max(1, round(pred_y) - radius);
    r_hi = min(H, round(pred_y) + radius);
    c_lo = max(1, round(pred_x) - radius);
    c_hi = min(W, round(pred_x) + radius);

    % Find max in ROI
    max_val = 0;
    for r = r_lo:r_hi
        for c = c_lo:c_hi
            if map(r,c) > max_val
                max_val = map(r,c);
            end
        end
    end

    if max_val < 1e-10
        cx = pred_x; cy = pred_y; conf = 0;
        return;
    end

    % Threshold at top fraction of ROI pixels
    roi_area = (r_hi - r_lo + 1) * (c_hi - c_lo + 1);
    target_count = max(min_pixels, round(top_frac * roi_area));

    N_BINS = 50;
    bin_width = max_val / N_BINS;
    bin_counts = zeros(N_BINS, 1);

    for r = r_lo:r_hi
        for c = c_lo:c_hi
            b = min(N_BINS, max(1, ceil(map(r,c) / bin_width)));
            bin_counts(b) = bin_counts(b) + 1;
        end
    end

    cumulative = 0;
    thresh_bin = N_BINS;
    for b = N_BINS:-1:1
        cumulative = cumulative + bin_counts(b);
        if cumulative >= target_count
            thresh_bin = b;
            break;
        end
    end

    adaptive_thresh = (thresh_bin - 1) * bin_width;

    sum_x = 0; sum_y = 0; sum_w = 0; count_above = 0;
    for r = r_lo:r_hi
        for c = c_lo:c_hi
            if map(r,c) > adaptive_thresh
                w = map(r,c);
                sum_x = sum_x + c * w;
                sum_y = sum_y + r * w;
                sum_w = sum_w + w;
                count_above = count_above + 1;
            end
        end
    end

    if count_above >= min_pixels && sum_w > 0
        cx = sum_x / sum_w;
        cy = sum_y / sum_w;
        % Confidence = mean intensity of bright pixels relative to max.
        % This avoids dilution from the large ROI area.
        conf = (sum_w / count_above) / (max_val + 1e-10);
    else
        cx = pred_x;
        cy = pred_y;
        conf = 0;
    end
end


% ================================================================
% 3x3 SOLVER (Cramer's rule)
% ================================================================
function x = solve_3x3(A, b)
    d = A(1,1)*(A(2,2)*A(3,3)-A(2,3)*A(3,2)) ...
      - A(1,2)*(A(2,1)*A(3,3)-A(2,3)*A(3,1)) ...
      + A(1,3)*(A(2,1)*A(3,2)-A(2,2)*A(3,1));

    x = zeros(3,1);
    if abs(d) < 1e-12
        return;
    end

    x(1) = (b(1)*(A(2,2)*A(3,3)-A(2,3)*A(3,2)) ...
          - A(1,2)*(b(2)*A(3,3)-A(2,3)*b(3)) ...
          + A(1,3)*(b(2)*A(3,2)-A(2,2)*b(3))) / d;

    x(2) = (A(1,1)*(b(2)*A(3,3)-A(2,3)*b(3)) ...
          - b(1)*(A(2,1)*A(3,3)-A(2,3)*A(3,1)) ...
          + A(1,3)*(A(2,1)*b(3)-b(2)*A(3,1))) / d;

    x(3) = (A(1,1)*(A(2,2)*b(3)-b(2)*A(3,2)) ...
          - A(1,2)*(A(2,1)*b(3)-b(2)*A(3,1)) ...
          + b(1)*(A(2,1)*A(3,2)-A(2,2)*A(3,1))) / d;
end


% ================================================================
% ESTIMATE TRAIL RADIUS — adaptive from peak region density
% ================================================================
function r_opt = estimate_trail_radius(map, H, W, pctile, r_min, r_max, r_default)
% Estimates the trail's characteristic half-width by finding the
% brightest pixel in the map and measuring the half-width at
% half-maximum (HWHM) radially outward from it.
%
% Returns a radius in [r_min, r_max].  If the map is empty or
% the HWHM measurement fails, returns r_default.

    % Find peak pixel
    peak_r = 1; peak_c = 1; peak_v = 0;
    for r = 1:H
        for c = 1:W
            if map(r,c) > peak_v
                peak_v = map(r,c);
                peak_r = r;
                peak_c = c;
            end
        end
    end

    if peak_v < 1e-10
        r_opt = r_default;
        return;
    end

    half_max = peak_v * pctile;

    % Radial profile: measure intensity at increasing radii
    % Average over 8 directions for robustness
    MAX_R = r_max * 3;
    radial = zeros(MAX_R, 1);
    radial_count = zeros(MAX_R, 1);
    for dir = 1:8
        theta = double(dir - 1) * pi / 4.0;
        ct = cos(theta); st = sin(theta);
        for ri = 1:MAX_R
            rr = round(double(peak_r) + double(ri) * st);
            cc = round(double(peak_c) + double(ri) * ct);
            if rr >= 1 && rr <= H && cc >= 1 && cc <= W
                radial(ri) = radial(ri) + map(rr, cc);
                radial_count(ri) = radial_count(ri) + 1;
            end
        end
    end

    % Average radial profile
    for ri = 1:MAX_R
        if radial_count(ri) > 0
            radial(ri) = radial(ri) / radial_count(ri);
        end
    end

    % Find HWHM: first radius where intensity drops below half_max
    hwhm = r_default;
    for ri = 1:MAX_R
        if radial(ri) < half_max
            hwhm = ri;
            break;
        end
    end

    r_opt = max(r_min, min(r_max, hwhm));
end


% ================================================================
% MEXICAN HAT FILTER — radial band-pass for trail width
% ================================================================
function out = mexican_hat_filter(map, H, W, r_opt)
% Convolves the map with a 2D Mexican Hat (Laplacian of Gaussian)
% kernel tuned to radius r_opt.
%
% Profile: w(r) = (1 - (r/r_opt)^2) * exp(-r^2 / (2*r_opt^2))
%
% Features matching r_opt in width get positive response.
% Features wider than r_opt (blob) get strong negative response.
% The kernel is zero-mean, so uniform regions cancel to zero.
%
% 2x downsampled for performance. Codegen-safe.

    % --- 2x downsample (block-average) ---
    H2 = floor(H / 2);
    W2 = floor(W / 2);
    small = zeros(H2, W2);
    for r = 1:H2
        r1 = 2*r - 1;
        r2 = min(H, 2*r);
        for c = 1:W2
            c1 = 2*c - 1;
            c2 = min(W, 2*c);
            small(r,c) = (map(r1,c1) + map(r1,c2) ...
                        + map(r2,c1) + map(r2,c2)) * 0.25;
        end
    end

    % Halved radius for downsampled grid
    sigma = max(2, round(double(r_opt) / 2));
    khw = min(3 * sigma, 30);  % kernel half-width, capped at 30
    ksz = 2 * khw + 1;

    % --- Build Mexican Hat kernel ---
    kern = zeros(ksz, ksz);
    sigma2 = double(sigma * sigma);
    for kr = -khw:khw
        for kc = -khw:khw
            r2 = double(kr*kr + kc*kc);
            norm_r2 = r2 / sigma2;
            kern(kr+khw+1, kc+khw+1) = (1.0 - norm_r2) * exp(-r2 / (2.0 * sigma2));
        end
    end

    % Zero-mean normalization (ensure blob suppression)
    kern_sum = 0;
    for kr = 1:ksz
        for kc = 1:ksz
            kern_sum = kern_sum + kern(kr, kc);
        end
    end
    kern_mean = kern_sum / double(ksz * ksz);
    for kr = 1:ksz
        for kc = 1:ksz
            kern(kr, kc) = kern(kr, kc) - kern_mean;
        end
    end

    % --- Convolve ---
    filtered = conv2(small, kern, 'same');

    % --- Clip negative to zero (only keep positive response) ---
    for r = 1:H2
        for c = 1:W2
            if filtered(r,c) < 0
                filtered(r,c) = 0;
            end
        end
    end

    % --- 2x upsample (nearest neighbour) ---
    out = zeros(H, W);
    for r = 1:H
        sr = min(H2, max(1, ceil(double(r) / 2)));
        for c = 1:W
            sc = min(W2, max(1, ceil(double(c) / 2)));
            out(r,c) = filtered(sr, sc);
        end
    end
end


% ================================================================
% MORPHOLOGICAL LINE OPENING — multi-angle max
% ================================================================
function out = morph_line_open(map, H, W, line_len, n_angles)
% Morphological opening with line structuring elements at multiple
% angles.  For each angle, erode then dilate with a line SE of the
% given length.  The output is the per-pixel MAX across all angles,
% preserving the best-fitting orientation at each pixel.
%
% This removes features shorter than line_len and features that
% don't align with any of the tested angles.  Blobs, short edges,
% and noise are suppressed; elongated structures survive.
%
% 2x downsampling for performance (same strategy as gabor_filter_bank).
% line_len is in full-res pixels; halved for the downsampled grid.
%
% Codegen-safe: fixed-size SE, no imerode/imdilate (manual impl).

    % --- 2x downsample (block-average) ---
    H2 = floor(H / 2);
    W2 = floor(W / 2);
    small = zeros(H2, W2);
    for r = 1:H2
        r1 = 2*r - 1;
        r2 = min(H, 2*r);
        for c = 1:W2
            c1 = 2*c - 1;
            c2 = min(W, 2*c);
            small(r,c) = (map(r1,c1) + map(r1,c2) ...
                        + map(r2,c1) + map(r2,c2)) * 0.25;
        end
    end

    half_line = max(1, floor(line_len / 4));  % halved line for 2x down
    result_small = zeros(H2, W2);

    for ai = 1:n_angles
        theta = double(ai - 1) * pi / double(n_angles);
        ct = cos(theta);
        st = sin(theta);

        % Build line structuring element as list of (dr, dc) offsets
        se_len = 2 * half_line + 1;
        se_dr = zeros(se_len, 1);
        se_dc = zeros(se_len, 1);
        for k = 1:se_len
            t = double(k - 1 - half_line);
            se_dc(k) = round(t * ct);
            se_dr(k) = round(t * st);
        end

        % --- Erode: min over SE neighbourhood ---
        eroded = zeros(H2, W2);
        for r = 1:H2
            for c = 1:W2
                min_val = small(r, c);
                for k = 1:se_len
                    rr = r + se_dr(k);
                    cc = c + se_dc(k);
                    if rr >= 1 && rr <= H2 && cc >= 1 && cc <= W2
                        if small(rr, cc) < min_val
                            min_val = small(rr, cc);
                        end
                    else
                        min_val = 0;
                    end
                end
                eroded(r, c) = min_val;
            end
        end

        % --- Dilate: max over SE neighbourhood ---
        opened = zeros(H2, W2);
        for r = 1:H2
            for c = 1:W2
                max_val = eroded(r, c);
                for k = 1:se_len
                    rr = r + se_dr(k);
                    cc = c + se_dc(k);
                    if rr >= 1 && rr <= H2 && cc >= 1 && cc <= W2
                        if eroded(rr, cc) > max_val
                            max_val = eroded(rr, cc);
                        end
                    end
                end
                opened(r, c) = max_val;
            end
        end

        % --- Max across angles ---
        for r = 1:H2
            for c = 1:W2
                if opened(r, c) > result_small(r, c)
                    result_small(r, c) = opened(r, c);
                end
            end
        end
    end

    % --- 2x upsample (nearest neighbour) ---
    out = zeros(H, W);
    for r = 1:H
        sr = min(H2, max(1, ceil(double(r) / 2)));
        for c = 1:W
            sc = min(W2, max(1, ceil(double(c) / 2)));
            out(r,c) = result_small(sr, sc);
        end
    end
end


% ================================================================
% STRUCTURE TENSOR ANISOTROPY (deprecated — kept for reference)
% ================================================================
function aniso = compute_anisotropy(map, H, W, K)
% Computes per-pixel anisotropy from the structure tensor.
% Uses image gradients + box-filter smoothing of tensor components.
%
% Output: aniso(r,c) in [0, 1].
%   0 = isotropic (blob interior, uniform)
%   1 = perfectly linear (thin ridge/edge)
%
% Based on eigenvalue ratio of the smoothed structure tensor:
%   aniso = (lambda1 - lambda2) / (lambda1 + lambda2 + eps)

    % --- Gradients (central differences, zero-padded borders) ---
    Gx = zeros(H, W);
    Gy = zeros(H, W);
    for r = 2:H-1
        for c = 2:W-1
            Gx(r,c) = (map(r, c+1) - map(r, c-1)) * 0.5;
            Gy(r,c) = (map(r+1, c) - map(r-1, c)) * 0.5;
        end
    end

    % --- Structure tensor components ---
    J11 = Gx .* Gx;
    J12 = Gx .* Gy;
    J22 = Gy .* Gy;

    % --- Smooth with box filter ---
    win_area = (2*K+1)^2;
    J11 = box_smooth_local(J11, H, W, K) / win_area;
    J12 = box_smooth_local(J12, H, W, K) / win_area;
    J22 = box_smooth_local(J22, H, W, K) / win_area;

    % --- Eigenvalue ratio (closed-form 2x2 symmetric) ---
    aniso = zeros(H, W);
    for r = 1:H
        for c = 1:W
            tr = J11(r,c) + J22(r,c);
            det_val = J11(r,c) * J22(r,c) - J12(r,c) * J12(r,c);
            disc = tr * tr / 4 - det_val;
            if disc < 0
                disc = 0;
            end
            sqrt_disc = sqrt(disc);
            lam1 = tr / 2 + sqrt_disc;
            lam2 = tr / 2 - sqrt_disc;
            denom = lam1 + lam2 + 1e-10;
            aniso(r,c) = (lam1 - lam2) / denom;
        end
    end
end


% ================================================================
% PERCENTILE CAP (histogram-based, codegen-safe)
% ================================================================
function cap_val = percentile_cap(map, H, W, pctile)
% Returns the value at the given percentile (0-1) of all positive
% pixels in the map. Uses histogram binning (no sort).
    max_val = 0;
    n_pos = 0;
    for r = 1:H
        for c = 1:W
            if map(r,c) > 0
                n_pos = n_pos + 1;
                if map(r,c) > max_val
                    max_val = map(r,c);
                end
            end
        end
    end

    if n_pos == 0 || max_val < 1e-10
        cap_val = 0;
        return;
    end

    N_BINS = 200;
    bin_w = max_val / double(N_BINS);
    hist_c = zeros(N_BINS, 1);
    for r = 1:H
        for c = 1:W
            if map(r,c) > 0
                b = min(N_BINS, max(1, ceil(map(r,c) / bin_w)));
                hist_c(b) = hist_c(b) + 1;
            end
        end
    end

    target = pctile * double(n_pos);
    cum = 0;
    pct_bin = N_BINS;
    for b = 1:N_BINS
        cum = cum + hist_c(b);
        if cum >= target
            pct_bin = b;
            break;
        end
    end
    cap_val = (double(pct_bin) - 0.5) * bin_w;
end


% ================================================================
% BOX SMOOTH (separable cumulative-sum filter)
% ================================================================
function out = box_smooth_local(in, H, W, K)
% Local sum within (2K+1)x(2K+1) window per pixel.
% Same algorithm as box_smooth in sharpness_detection.
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
