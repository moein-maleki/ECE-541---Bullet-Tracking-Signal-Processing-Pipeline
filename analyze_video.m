function analyze_video(log, run_dir)
% ANALYZE_VIDEO  Plot diagnostic signals from the pipeline run.
%
%   analyze_video(log, run_dir)
%
%   log is a struct with fields (each Nx1 vector, one entry per frame).
%   run_dir is the output directory for saving figures.

N = numel(log.current_state);
frames = 1:N;

pics_dir = fullfile(run_dir, 'pics');
if ~exist(pics_dir, 'dir')
    mkdir(pics_dir);
end

% --- Key frame indices ---
bg_frame     = find(log.background_ready, 1, 'first');
nf_frame     = find(log.noise_floor_ready, 1, 'first');
flight_frame = find(log.current_state == 1, 1, 'first');
track_frame  = find(log.tracking_active, 1, 'first');
lost_frame   = find(log.current_state == 2, 1, 'first');
impact_frame = find(log.current_state == 3, 1, 'first');

fprintf('=== TIMELINE ===\n');
fprintf('Background ready at frame : %d\n', bg_frame);
fprintf('Noise floor ready at frame: %d\n', nf_frame);
if ~isempty(flight_frame)
    fprintf('First FLIGHT at frame     : %d\n', flight_frame);
end
if ~isempty(track_frame)
    fprintf('Tracking active at frame  : %d\n', track_frame);
end
if ~isempty(lost_frame)
    fprintf('First LOST at frame       : %d\n', lost_frame);
end
if ~isempty(impact_frame)
    fprintf('IMPACT at frame           : %d\n', impact_frame);
end
fprintf('\n=== SIGNAL STATS ===\n');
fprintf('Max confidence       : %.6f\n', max(log.confidence));
fprintf('Max frame energy     : %.6f\n', max(log.frame_energy_scalar));
fprintf('Max detection map val: %.6f\n', max(log.detection_max));
fprintf('Total path points    : %d\n',   max(log.path_len));
active = log.path_x > 0;
if any(active)
    fprintf('Path X range: [%.1f, %.1f]\n', min(log.path_x(active)), max(log.path_x(active)));
    fprintf('Path Y range: [%.1f, %.1f]\n', min(log.path_y(active)), max(log.path_y(active)));
end
if ~isempty(nf_frame) && nf_frame + 10 <= N
    fprintf('Onset threshold (stabilized): %.6f\n', log.onset_threshold(min(N, nf_frame + 80)));
end

% --- Debug centroid summary ---
has_centroids = isfield(log, 'debug_centroid_x');
if has_centroids
    cx_valid = log.debug_centroid_x > 0;
    n_centroids = sum(cx_valid);
    fprintf('\n=== CENTROID DEBUG ===\n');
    fprintf('Total centroid detections: %d\n', n_centroids);
    if n_centroids > 0
        fprintf('First centroid at frame  : %d\n', find(cx_valid, 1, 'first'));
        fprintf('Centroid X range: [%.1f, %.1f]\n', ...
            min(log.debug_centroid_x(cx_valid)), max(log.debug_centroid_x(cx_valid)));
        fprintf('Centroid Y range: [%.1f, %.1f]\n', ...
            min(log.debug_centroid_y(cx_valid)), max(log.debug_centroid_y(cx_valid)));
    end
end

% --- Impact detection summary ---
has_impact = isfield(log, 'impact_detected');
if has_impact
    imp_frame = find(log.impact_detected, 1, 'first');
    fprintf('\n=== IMPACT DETECTION ===\n');
    if ~isempty(imp_frame)
        fprintf('Impact detected at frame : %d\n', imp_frame);
        fprintf('Impact location          : (%.1f, %.1f)\n', ...
            log.impact_x(imp_frame), log.impact_y(imp_frame));
        fprintf('Max splash energy        : %.1f\n', max(log.splash_energy));
    else
        fprintf('Impact NOT detected\n');
        fprintf('Max splash signal        : %.1f\n', max(log.splash_energy));
        % Show pre-flight baseline for debugging
        nf_mask = log.noise_floor_ready & (log.current_state == 0);
        pre_flight = log.splash_energy(nf_mask);
        if ~isempty(pre_flight) && any(pre_flight > 0)
            fprintf('Baseline (mean,post-NF)  : %.1f\n', mean(pre_flight(pre_flight > 0)));
            fprintf('Baseline (max,post-NF)   : %.1f\n', max(pre_flight));
        end
        % Show post-flight splash stats
        post_flight = log.splash_energy(log.current_state >= 1);
        if ~isempty(post_flight) && any(post_flight > 0)
            fprintf('Post-flight max          : %.1f\n', max(post_flight));
            fprintf('Post-flight mean         : %.1f\n', mean(post_flight(post_flight > 0)));
        end
    end
end

%% ===================== FIGURE 1: Main diagnostics =====================
figure('Name', 'System Diagnostic', 'Position', [50 50 1300 900]);

% --- Frame energy + onset threshold ---
subplot(5,1,1);
plot(frames, log.frame_energy_scalar, 'b'); hold on;
plot(frames, log.onset_threshold, 'r--', 'LineWidth', 1.2);
if ~isempty(flight_frame)
    xline(flight_frame, 'g-', 'FLIGHT', 'LineWidth', 1.5);
end
ylabel('Energy'); title('Frame Energy vs Onset Threshold');
legend('frame\_energy', 'onset\_thresh', 'Location', 'best');
grid on; xlim([1 N]);

% --- State machine ---
subplot(5,1,2);
plot(frames, log.current_state, 'b', 'LineWidth', 2);
yticks([0 1 2 3]);
yticklabels({'WAITING','FLIGHT','LOST','IMPACT'});
title('State Machine'); grid on; xlim([1 N]);
ylim([-0.5 3.5]);

% --- Confidence ---
subplot(5,1,3);
plot(frames, log.confidence, 'r'); hold on;
yline(0.02, 'b--', 'LOST thresh');
yline(0.05, 'g--', 'REACQ thresh');
title('Confidence'); grid on; xlim([1 N]);

% --- Path position ---
subplot(5,1,4);
plot(frames, log.path_x, 'r'); hold on;
plot(frames, log.path_y, 'b');
legend('path\_x', 'path\_y', 'Location', 'best');
title('Predicted Position'); ylabel('pixels'); grid on; xlim([1 N]);

% --- Detection map strength + splash energy ---
subplot(5,1,5);
plot(frames, log.detection_max, 'r'); hold on;
plot(frames, log.detection_mean, 'b');
if has_impact
    yyaxis right;
    plot(frames, log.splash_energy, 'm');
    ylabel('Splash Energy');
    legend('detection\_max', 'detection\_mean', 'splash\_energy', 'Location', 'best');
else
    legend('detection\_max', 'detection\_mean', 'Location', 'best');
end
title('Detection Map Statistics'); xlabel('Frame'); grid on; xlim([1 N]);

sgtitle('SYSTEM DIAGNOSTIC');
saveas(gcf, fullfile(pics_dir, 'system_diagnostic.png'));

%% ===================== FIGURE 2: Trajectory coefficients =====================
figure('Name', 'Trajectory Coefficients', 'Position', [100 100 1200 600]);

subplot(2,1,1);
plot(frames, log.ax, 'r'); hold on;
plot(frames, log.bx, 'g');
plot(frames, log.cx, 'b');
legend('ax','bx','cx', 'Location', 'best');
title('X trajectory coefficients (ax t^2 + bx t + cx)');
grid on; xlim([1 N]);

subplot(2,1,2);
plot(frames, log.ay, 'r'); hold on;
plot(frames, log.by, 'g');
plot(frames, log.cy, 'b');
legend('ay','by','cy', 'Location', 'best');
title('Y trajectory coefficients (ay t^2 + by t + cy)');
xlabel('Frame'); grid on; xlim([1 N]);

sgtitle('TRAJECTORY FIT EVOLUTION');
saveas(gcf, fullfile(pics_dir, 'trajectory_coefficients.png'));

%% ===================== FIGURE 3: 2D flight path + centroids =====================
if any(active)
    figure('Name', 'Flight Path', 'Position', [150 150 900 600]);

    plot(log.path_x(active), log.path_y(active), 'g.-', 'MarkerSize', 8); hold on;

    % Color-code by state
    flight_mask = active & (log.current_state == 1);
    lost_mask   = active & (log.current_state == 2);
    if any(flight_mask)
        plot(log.path_x(flight_mask), log.path_y(flight_mask), 'go', ...
            'MarkerSize', 6, 'MarkerFaceColor', 'g');
    end
    if any(lost_mask)
        plot(log.path_x(lost_mask), log.path_y(lost_mask), 'yo', ...
            'MarkerSize', 6, 'MarkerFaceColor', [1 0.7 0]);
    end

    % Overlay raw centroid detections (red x markers)
    if has_centroids
        cx_valid = log.debug_centroid_x > 0;
        if any(cx_valid)
            plot(log.debug_centroid_x(cx_valid), log.debug_centroid_y(cx_valid), ...
                'rx', 'MarkerSize', 10, 'LineWidth', 2);
        end
    end

    % Plot impact location if detected
    if has_impact
        imp_frame = find(log.impact_detected, 1, 'first');
        if ~isempty(imp_frame)
            plot(log.impact_x(imp_frame), log.impact_y(imp_frame), ...
                'mp', 'MarkerSize', 18, 'MarkerFaceColor', 'm', 'LineWidth', 2);
        end
    end

    set(gca, 'YDir', 'reverse');  % image coordinates
    xlim([0 1920]); ylim([0 1080]);
    xlabel('X (pixels)'); ylabel('Y (pixels)');
    title('Detected Flight Path (image coords)');
    leg_entries = {'path'};
    if any(flight_mask), leg_entries{end+1} = 'FLIGHT'; end
    if any(lost_mask),   leg_entries{end+1} = 'LOST'; end
    if has_centroids && any(cx_valid), leg_entries{end+1} = 'raw centroids'; end
    if has_impact && ~isempty(find(log.impact_detected, 1, 'first'))
        leg_entries{end+1} = 'IMPACT site';
    end
    legend(leg_entries{:}, 'Location', 'best');
    grid on; axis equal;
    saveas(gcf, fullfile(pics_dir, 'flight_path.png'));
end

%% ===================== FIGURE 4: Detection map montage =====================
maps_dir = fullfile(run_dir, 'maps');
maps_det_dir = fullfile(maps_dir, 'detection');
if exist(maps_det_dir, 'dir')
    map_files = dir(fullfile(maps_det_dir, 'det_*.png'));
    n_maps = numel(map_files);
    if n_maps > 0
        % Show up to 20 maps in a grid
        n_show = min(n_maps, 20);
        cols = 5;
        rows = ceil(n_show / cols);

        figure('Name', 'Detection Maps', 'Position', [200 200 1400 700]);
        for i = 1:n_show
            subplot(rows, cols, i);
            img = imread(fullfile(maps_det_dir, map_files(i).name));
            imagesc(img); axis image off; colormap(hot);
            % Extract frame number from filename
            fname = map_files(i).name;
            title(strrep(fname(1:end-4), '_', ' '), 'FontSize', 8);
        end
        sgtitle('Detection Maps at Flight Onset');
        saveas(gcf, fullfile(pics_dir, 'detection_maps.png'));
        fprintf('Detection map montage: %d maps shown\n', n_show);
    end
end

%% ===================== FIGURE 5: Centroid diagnostics =====================
has_ndet = isfield(log, 'debug_n_detections');
has_chg  = isfield(log, 'debug_change_max');
has_mrg  = isfield(log, 'debug_merged');

if has_ndet || has_chg || has_mrg || has_centroids
    figure('Name', 'Centroid Diagnostics', 'Position', [250 250 1300 900]);

    % --- n_detections over time ---
    if has_ndet
        subplot(2,2,1);
        plot(frames, log.debug_n_detections, 'b', 'LineWidth', 1.5);
        hold on;
        % Mark PCA fire frame
        pca_fire = find(log.debug_n_detections >= 3, 1);
        if ~isempty(pca_fire)
            xline(pca_fire, 'g--', 'PCA', 'LineWidth', 1.5, 'LabelOrientation', 'horizontal');
        end
        hold off;
        ylabel('n\_detections'); title('Centroid Buffer Count');
        grid on; xlim([1 N]);
        fprintf('Max n_detections reached: %d\n', max(log.debug_n_detections));
    end

    % --- change_map max over time ---
    if has_chg
        subplot(2,2,2);
        plot(frames, log.debug_change_max, 'r', 'LineWidth', 1);
        ylabel('max value'); title('Change Map Peak (per frame)');
        grid on; xlim([1 N]);
    end

    % --- 2D centroid scatter + PCA points ---
    if has_centroids
        subplot(2,2,3);
        cx_valid = log.debug_centroid_x > 0;
        if any(cx_valid)
            scatter(log.debug_centroid_x(cx_valid), ...
                    log.debug_centroid_y(cx_valid), ...
                    20, frames(cx_valid)', 'filled');
            colorbar; colormap(gca, 'parula');
            set(gca, 'YDir', 'reverse');
            xlim([0 1920]); ylim([0 1080]);
            xlabel('X'); ylabel('Y');
            title('Centroid Positions (color = frame #)');
            axis equal; grid on;
            hold on;
            % Mark PCA frame: find first frame where n_detections jumps to 3
            if has_ndet
                pca_frame = find(log.debug_n_detections >= 3, 1);
                if ~isempty(pca_frame)
                    % Plot the 3 PCA seed points from the path log
                    % They are the centroid_x/y at the PCA frame and its
                    % trajectory endpoints (from coefficients)
                    pca_cx = log.debug_centroid_x(pca_frame);
                    pca_cy = log.debug_centroid_y(pca_frame);
                    if pca_cx > 0
                        plot(pca_cx, pca_cy, 'd', 'MarkerSize', 14, ...
                             'MarkerFaceColor', 'c', 'MarkerEdgeColor', 'w', ...
                             'LineWidth', 1.5);
                    end
                end
            end
            hold off;
        end
    end

    % --- Merged / rejected flags ---
    if has_mrg
        subplot(2,2,4);
        stem(frames, log.debug_merged, 'b', 'Marker', 'none', 'LineWidth', 0.5); hold on;
        stem(frames, -log.debug_rejected, 'r', 'Marker', 'none', 'LineWidth', 0.5);
        legend('merged', 'rejected'); ylim([-1.5 1.5]);
        title('Centroid Merged (+1) / Rejected (-1)');
        xlabel('Frame'); grid on; xlim([1 N]);
    end

    sgtitle('CENTROID DIAGNOSTICS');
    saveas(gcf, fullfile(pics_dir, 'centroid_diagnostics.png'));
end

fprintf('Figures saved to %s\n', pics_dir);

end
