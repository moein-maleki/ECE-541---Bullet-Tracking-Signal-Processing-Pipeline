function [roi_x, roi_y] = select_impact_roi(video_path)
% SELECT_IMPACT_ROI  Interactive tool to mark the bullet impact area.
%
%   [roi_x, roi_y] = select_impact_roi(video_path)
%
%   Displays a frame from near the end of the video and lets you click
%   the approximate impact location.  Returns pixel coordinates.
%
%   Usage:
%     [x, y] = select_impact_roi('../longer.mp4');
%     % Then set IMPACT_ROI_X and IMPACT_ROI_Y in main_pipeline.m

if nargin < 1
    video_path = fullfile('..', 'longer.mp4');
end

reader = VideoReader(video_path);
n_frames = floor(reader.Duration * reader.FrameRate);

% Read a frame from near the end (after impact visible)
target = min(n_frames, round(n_frames * 0.85));
rgb = [];
for i = 1:target
    rgb = readFrame(reader);
end

figure('Name', 'Select Impact Location', 'NumberTitle', 'off');
imshow(rgb);
title({'Click the approximate bullet impact location'; ...
       '(where dirt/debris appears)'});

[x, y] = ginput(1);
roi_x = round(x);
roi_y = round(y);

hold on;
r = 150;
th = linspace(0, 2*pi, 100);
plot(roi_x + r*cos(th), roi_y + r*sin(th), 'm-', 'LineWidth', 2);
plot(roi_x, roi_y, 'mx', 'MarkerSize', 20, 'LineWidth', 3);
title(sprintf('Impact ROI: (%d, %d), radius=%d', roi_x, roi_y, r));

fprintf('\n=== Copy these into main_pipeline.m ===\n');
fprintf('IMPACT_ROI_X      = %d;\n', roi_x);
fprintf('IMPACT_ROI_Y      = %d;\n', roi_y);
fprintf('IMPACT_ROI_RADIUS = %d;\n', r);

end
