%% DEFINE_IDEAL_PATH — Interactive tool to mark the ideal bullet trail
%
% Opens a video frame where the trail is visible. Click along the
% trail from near-field (entry, bottom-right) to far-field (exit).
% Press Enter when done. Saves ideal_path.mat with ideal_x, ideal_y.
%
% Usage: run this script, click on the trail, press Enter.

video_path = fullfile('..', 'longer.mp4');
FRAME_NUM  = 190;  % frame where trail is clearly visible

reader = VideoReader(video_path);
reader.CurrentTime = (FRAME_NUM - 1) / reader.FrameRate;
frame = readFrame(reader);

figure('Name', 'Define Ideal Path', 'NumberTitle', 'off');
imshow(frame);
title(sprintf('Frame %d — Click along the ideal trail (near-field to far-field). Press Enter when done.', FRAME_NUM));
hold on;

[ideal_x, ideal_y] = ginput;

plot(ideal_x, ideal_y, 'm-o', 'LineWidth', 2, 'MarkerSize', 8, 'MarkerFaceColor', 'm');
title(sprintf('Ideal path: %d points. Close figure to continue.', length(ideal_x)));
drawnow;

save('ideal_path.mat', 'ideal_x', 'ideal_y');
fprintf('Saved %d ideal path points to ideal_path.mat\n', length(ideal_x));
