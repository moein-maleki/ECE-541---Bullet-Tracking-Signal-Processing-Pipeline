function annotated_frame = video_overlay(original_rgb, ...
    path_xs, path_ys, path_confs, path_states, path_len, ...
    current_path_x, current_path_y, ...
    current_state, confidence, state_duration, ...
    ax, bx, cx, ay, by, cy, t_ref, ...
    impact_x, impact_y, impact_detected, ...
    blob_x, blob_y, ...
    centroid_log_x, centroid_log_y, centroid_log_len, ...
    clean_detection)
%#codegen
% VIDEO_OVERLAY  Annotate a video frame with tracking overlays.
%
% Inputs:
%   original_rgb     : HxWx3 uint8, raw video frame
%   path_xs/ys       : MAX_PATH x 1 double, historical positions
%   path_confs       : MAX_PATH x 1 double, confidence per point
%   path_states      : MAX_PATH x 1 int32, state at each point
%   path_len         : int32, valid history entries
%   current_path_x/y : double, current predicted position
%   current_state    : int32, (0=WAITING,1=FLIGHT,2=LOST,3=IMPACT)
%   confidence       : double, [0,1]
%   state_duration   : int32, frames in current state
%   ax,bx,cx,ay,by,cy: double, trajectory coefficients
%   t_ref            : int32, reference frame for trajectory evaluation
%   impact_x/y       : double, detected impact position
%   impact_detected  : logical, whether impact has been confirmed
%   blob_x, blob_y   : double, first centroid (blob) position for debug
%   centroid_log_x/y : MAX_CENTROID x 1 double, all logged centroid positions
%   centroid_log_len : int32, number of valid centroid log entries
%   clean_detection  : HxW double, detection map overlaid as green intensity

[H, W, ~] = size(original_rgb);

% Force all inputs to real to prevent complex propagation
ax = real(ax); bx = real(bx); cx = real(cx);
ay = real(ay); by = real(by); cy = real(cy);
current_state  = int32(real(current_state));
state_duration = int32(real(state_duration));
t_ref          = int32(real(t_ref));
path_len       = int32(real(path_len));
confidence     = real(confidence);
current_path_x = real(current_path_x);
current_path_y = real(current_path_y);
path_xs    = real(path_xs);
path_ys    = real(path_ys);
path_confs = real(path_confs);

out = double(original_rgb);

% ================================================================
% 0. DETECTION MAP OVERLAY — boost green channel by detection intensity
%    Normalizes clean_detection to [0,1] and adds to green channel,
%    making active detection regions glow green on the video.
% ================================================================
det_max = max(clean_detection(:));
if det_max > 0
    det_norm = clean_detection / det_max;
    out(:,:,2) = min(out(:,:,2) + det_norm * 150, 255);
end

n_path = double(path_len);

persistent count
if isempty(count)
    count = int32(0);
end
count = count + int32(1);
frame_count = count;

% ================================================================
% 1. CONNECTING LINES — dim green lines between consecutive path points
%    Position: along the detected flight path (anywhere on frame)
% ================================================================
for i = 1:n_path-1
    x0 = round(path_xs(i));  y0 = round(path_ys(i));
    x1 = round(path_xs(i+1)); y1 = round(path_ys(i+1));
    if x0 < 1 || x0 > W || y0 < 1 || y0 > H, continue; end
    if x1 < 1 || x1 > W || y1 < 1 || y1 > H, continue; end
    n_steps = max(abs(x1 - x0), abs(y1 - y0));
    if n_steps == 0, continue; end
    for s = 0:n_steps
        frac = double(s) / double(n_steps);
        lx = round(double(x0) + frac * double(x1 - x0));
        ly = round(double(y0) + frac * double(y1 - y0));
        if lx >= 1 && lx <= W && ly >= 1 && ly <= H
            for dt = -1:1
                rr = ly + dt;
                if rr >= 1 && rr <= H
                    out(rr, lx, 1) = 0;
                    out(rr, lx, 2) = 150;
                    out(rr, lx, 3) = 0;
                end
            end
        end
    end
end

% ================================================================
% 2. HISTORICAL PATH DOTS — color-coded by state
%    Position: along the detected flight path (anywhere on frame)
%    Green = FLIGHT, Orange = LOST, White = other
% ================================================================
for i = 1:n_path
    px = round(path_xs(i)); py = round(path_ys(i));
    st = path_states(i); cn = path_confs(i);
    if px < 1 || px > W || py < 1 || py > H, continue; end
    if st == int32(1)
        cr = 0; cg = 255; cb = 0;
    elseif st == int32(2)
        cr = 255; cg = 200; cb = 0;
    else
        cr = 255; cg = 255; cb = 255;
    end
    radius = max(2, min(6, round(2 + cn * 20)));
    for dr = -radius:radius
        for dc = -radius:radius
            if dr*dr + dc*dc <= radius*radius
                rr = py + dr; cc = px + dc;
                if rr >= 1 && rr <= H && cc >= 1 && cc <= W
                    out(rr,cc,1) = cr; out(rr,cc,2) = cg; out(rr,cc,3) = cb;
                end
            end
        end
    end
end

% ================================================================
% 3. CURRENT POSITION CROSSHAIR — red cross at predicted position
%    Position: at current prediction (anywhere on frame)
% ================================================================
if current_state >= int32(1) && current_path_x > 0 && current_path_y > 0
    cpx = round(current_path_x); cpy = round(current_path_y);
    arm = 18;
    for dc = -arm:arm
        for dt = -1:1
            cc = cpx + dc; rr = cpy + dt;
            if rr >= 1 && rr <= H && cc >= 1 && cc <= W
                out(rr,cc,1) = 255; out(rr,cc,2) = 0; out(rr,cc,3) = 0;
            end
        end
    end
    for dr = -arm:arm
        for dt = -1:1
            rr = cpy + dr; cc = cpx + dt;
            if rr >= 1 && rr <= H && cc >= 1 && cc <= W
                out(rr,cc,1) = 255; out(rr,cc,2) = 0; out(rr,cc,3) = 0;
            end
        end
    end
end

% ================================================================
% 4. PREDICTED FUTURE TRAJECTORY — cyan dashed dots
%    Position: projected from current position forward (anywhere on frame)
% ================================================================
PRED_HORIZON = 30;
if current_state == int32(1) || current_state == int32(2)
    t_now = double(frame_count) - double(t_ref);
    for k = 1:PRED_HORIZON
        t_future = t_now + double(k);
        fx = ax*t_future*t_future + bx*t_future + cx;
        fy = ay*t_future*t_future + by*t_future + cy;
        fpx = round(fx); fpy = round(fy);
        if mod(k,3) ~= 0, continue; end
        if fpx < 1 || fpx > W || fpy < 1 || fpy > H, continue; end
        for dr = -2:2
            for dc = -2:2
                if dr*dr + dc*dc <= 4
                    rr = fpy + dr; cc = fpx + dc;
                    if rr >= 1 && rr <= H && cc >= 1 && cc <= W
                        out(rr,cc,1) = 0; out(rr,cc,2) = 255; out(rr,cc,3) = 255;
                    end
                end
            end
        end
    end
end

% ================================================================
% 4b. IMPACT MARKER — large magenta X at detected impact site
%     Position: at impact location (anywhere on frame)
% ================================================================
if impact_detected && impact_x > 0 && impact_y > 0
    ipx = round(impact_x); ipy = round(impact_y);
    imp_arm = 25;
    % Draw X shape (two diagonal lines, 3px thick)
    for k = -imp_arm:imp_arm
        for dt = -1:1
            % Diagonal 1: top-left to bottom-right
            rr = ipy + k + dt; cc = ipx + k;
            if rr >= 1 && rr <= H && cc >= 1 && cc <= W
                out(rr,cc,1) = 255; out(rr,cc,2) = 0; out(rr,cc,3) = 255;
            end
            % Diagonal 2: top-right to bottom-left
            rr = ipy - k + dt; cc = ipx + k;
            if rr >= 1 && rr <= H && cc >= 1 && cc <= W
                out(rr,cc,1) = 255; out(rr,cc,2) = 0; out(rr,cc,3) = 255;
            end
        end
    end
    % Label "IMPACT" next to marker
    out = draw_text(out, 'IMPACT', ipx + imp_arm + 5, ipy - 10, ...
                    255, 0, 255, 2, H, W);
end

% ================================================================
% 4b2. FAR-FIELD ENDPOINT — red circle at trajectory far-field (t=20)
%      Marks where the quadratic fit ends (Mode 2 clamp point)
% ================================================================
TRAIL_DUR_VIS = 20;  % must match TRAIL_DURATION in trajectory block
if t_ref > int32(0) && (current_state == int32(1) || current_state == int32(2))
    t_far = double(TRAIL_DUR_VIS);
    far_x = round(ax*t_far*t_far + bx*t_far + cx);
    far_y = round(ay*t_far*t_far + by*t_far + cy);
    far_r = 12;  % circle radius
    if far_x >= 1 && far_x <= W && far_y >= 1 && far_y <= H
        % Draw circle outline (ring, 2px thick)
        for dr = -(far_r+1):(far_r+1)
            for dc = -(far_r+1):(far_r+1)
                d2 = dr*dr + dc*dc;
                if d2 >= (far_r-1)*(far_r-1) && d2 <= (far_r+1)*(far_r+1)
                    rr = far_y + dr; cc = far_x + dc;
                    if rr >= 1 && rr <= H && cc >= 1 && cc <= W
                        out(rr,cc,1) = 255; out(rr,cc,2) = 0; out(rr,cc,3) = 0;
                    end
                end
            end
        end
        out = draw_text(out, 'FAR', far_x - 8, far_y + far_r + 4, ...
                        255, 0, 0, 1, H, W);
    end
end

% ================================================================
% 4c. BLOB MARKER — yellow circle at first centroid (debug)
%     Position: at blob/first-detection location
% ================================================================
if blob_x > 0 && blob_y > 0
    bpx = round(blob_x); bpy = round(blob_y);
    blob_r = 20;  % radius
    % Draw circle outline (3px thick)
    for ang = 0:359
        rad = double(ang) * 3.14159265 / 180.0;
        for thickness = -1:1
            rr = round(double(bpy) + (blob_r + thickness) * sin(rad));
            cc = round(double(bpx) + (blob_r + thickness) * cos(rad));
            if rr >= 1 && rr <= H && cc >= 1 && cc <= W
                out(rr,cc,1) = 255; out(rr,cc,2) = 255; out(rr,cc,3) = 0;
            end
        end
    end
    % Label "BLOB" next to marker
    out = draw_text(out, 'BLOB', bpx + blob_r + 5, bpy - 5, ...
                    255, 255, 0, 2, H, W);
end

% ================================================================
% 4d. CENTROID LOG — permanent cyan X markers at every raw centroid
%     Position: at each logged centroid location
% ================================================================
n_centroids = double(centroid_log_len);
for i = 1:n_centroids
    cpx = round(centroid_log_x(i));
    cpy = round(centroid_log_y(i));
    if cpx < 1 || cpx > W || cpy < 1 || cpy > H, continue; end
    % Draw small X (8px arms, 1px thick)
    for k = -5:5
        % Diagonal 1
        rr = cpy + k; cc = cpx + k;
        if rr >= 1 && rr <= H && cc >= 1 && cc <= W
            out(rr,cc,1) = 0; out(rr,cc,2) = 255; out(rr,cc,3) = 255;
        end
        % Diagonal 2
        rr = cpy - k; cc = cpx + k;
        if rr >= 1 && rr <= H && cc >= 1 && cc <= W
            out(rr,cc,1) = 0; out(rr,cc,2) = 255; out(rr,cc,3) = 255;
        end
    end
end

% ================================================================
% 5. STATUS BAR WITH STATE NAME
%    Position: TOP-LEFT corner (y=10..50, x=10..310)
%    Color-coded background: blue=WAITING, green=FLIGHT,
%    orange=LOST, red=IMPACT.  State name rendered as pixel text.
%    In LOST state, a shrinking white countdown bar at the bottom
%    of the status bar shows time remaining before IMPACT.
% ================================================================
bar_x = 10; bar_y = 10; bar_w = 300; bar_h = 40;

if current_state == int32(0)
    br = 50; bg_ = 100; bb = 200;
    state_str = 'WAITING';
    str_len = int32(7);
elseif current_state == int32(1)
    br = 0; bg_ = 200; bb = 0;
    state_str = 'FLIGHT ';
    str_len = int32(6);
elseif current_state == int32(2)
    br = 220; bg_ = 180; bb = 0;
    state_str = 'LOST   ';
    str_len = int32(4);
else
    br = 200; bg_ = 0; bb = 0;
    state_str = 'IMPACT ';
    str_len = int32(6);
end

% Draw colored bar background (semi-transparent blend)
for r = bar_y:min(bar_y + bar_h - 1, H)
    for c = bar_x:min(bar_x + bar_w - 1, W)
        out(r,c,1) = 0.7*br + 0.3*out(r,c,1);
        out(r,c,2) = 0.7*bg_ + 0.3*out(r,c,2);
        out(r,c,3) = 0.7*bb + 0.3*out(r,c,3);
    end
end

% Draw state name centered in bar (scale=3, ~18px per char)
text_scale = 3;
char_w = 6 * text_scale;
text_w = double(str_len) * char_w;
text_x = bar_x + round((bar_w - text_w) / 2);
text_y = bar_y + round((bar_h - 7*text_scale) / 2);
out = draw_text(out, state_str, text_x, text_y, 255, 255, 255, text_scale, H, W);

% LOST timeout countdown: shrinking bar at bottom of status bar
if current_state == int32(2)
    LOST_TIMEOUT_VIS = 8;   % must match LOST_TIMEOUT in flight_state_machine
    remaining_frac = max(0, 1.0 - double(state_duration)/double(LOST_TIMEOUT_VIS));
    remaining_w = round(remaining_frac * (bar_w - 4));
    inner_y_start = bar_y + bar_h - 6;
    inner_y_end = min(bar_y + bar_h - 2, H);
    inner_x_start = bar_x + 2;
    for r = inner_y_start:inner_y_end
        for c = inner_x_start:min(inner_x_start + remaining_w - 1, W)
            out(r,c,1) = 255; out(r,c,2) = 255; out(r,c,3) = 255;
        end
    end
end

% ================================================================
% 6. CONFIDENCE BAR
%    Position: TOP-LEFT, below status bar (y=55..70, x=10..310)
%    White fill proportional to confidence; dark background.
% ================================================================
cbar_y = bar_y + bar_h + 5;
cbar_h = 15;
conf_fill = max(0, min(1, confidence));
fill_w = round(conf_fill * bar_w);
for r = cbar_y:min(cbar_y + cbar_h - 1, H)
    for c = bar_x:min(bar_x + bar_w - 1, W)
        if c < bar_x + fill_w
            out(r,c,1) = 255; out(r,c,2) = 255; out(r,c,3) = 255;
        else
            out(r,c,1) = 40; out(r,c,2) = 40; out(r,c,3) = 40;
        end
    end
end

% ================================================================
% 7. INFO PANEL — frame number, state duration, confidence
%    Position: TOP-RIGHT corner (y=5..65, x=W-130..W-5)
%    Three lines of text on a dark semi-transparent background.
%    Line 1: F:XXXX  (frame number, 4 digits)
%    Line 2: D:XXXX  (state duration, 4 digits)
%    Line 3: C:XXXX  (confidence x10000, 4 digits)
% ================================================================
info_scale = 2;
info_char_w = 6 * info_scale;
info_line_h = 7 * info_scale + 4;
info_text_w = 6 * info_char_w;
info_x = W - info_text_w - 15;
info_y = 8;

% Dark background panel for readability
panel_x = max(1, info_x - 6);
panel_y = max(1, info_y - 4);
panel_w = info_text_w + 12;
panel_h = 3 * info_line_h + 8;
for r = panel_y:min(panel_y + panel_h - 1, H)
    for c = panel_x:min(panel_x + panel_w - 1, W)
        out(r,c,1) = out(r,c,1) * 0.3;
        out(r,c,2) = out(r,c,2) * 0.3;
        out(r,c,3) = out(r,c,3) * 0.3;
    end
end

% Line 1: Frame number
f_str = ['F:' int_to_chars(frame_count, 4)];
out = draw_text(out, f_str, info_x, info_y, 200, 200, 200, info_scale, H, W);

% Line 2: State duration
d_str = ['D:' int_to_chars(state_duration, 4)];
out = draw_text(out, d_str, info_x, info_y + info_line_h, 200, 200, 200, info_scale, H, W);

% Line 3: Confidence (x10000 for readability)
conf_int = int32(round(confidence * 10000));
c_str = ['C:' int_to_chars(conf_int, 4)];
out = draw_text(out, c_str, info_x, info_y + 2*info_line_h, 200, 200, 200, info_scale, H, W);

% ================================================================
% SCALE GRID — 100px spaced lines across the frame (yellow, 1px, semi-transparent)
% ================================================================
grid_alpha = 0.15;  % blend factor (subtle)
for gi = 1:floor(double(W) / 100)
    c = gi * 100;
    if c >= 1 && c <= W
        for r = 1:H
            out(r, c, 1) = out(r, c, 1) * (1 - grid_alpha) + 255 * grid_alpha;
            out(r, c, 2) = out(r, c, 2) * (1 - grid_alpha) + 255 * grid_alpha;
            out(r, c, 3) = out(r, c, 3) * (1 - grid_alpha) + 0   * grid_alpha;
        end
    end
end
for gi = 1:floor(double(H) / 100)
    r = gi * 100;
    if r >= 1 && r <= H
        for c = 1:W
            out(r, c, 1) = out(r, c, 1) * (1 - grid_alpha) + 255 * grid_alpha;
            out(r, c, 2) = out(r, c, 2) * (1 - grid_alpha) + 255 * grid_alpha;
            out(r, c, 3) = out(r, c, 3) * (1 - grid_alpha) + 0   * grid_alpha;
        end
    end
end

% ================================================================
% IDEAL PATH OVERLAY — magenta line from ideal_path.mat (if exists)
% ================================================================
persistent ideal_x_store ideal_y_store ideal_n ideal_loaded
if isempty(ideal_loaded)
    ideal_loaded = false;
    ideal_n = 0;
    ideal_x_store = zeros(200, 1);
    ideal_y_store = zeros(200, 1);
    if exist('ideal_path.mat', 'file')
        ip = load('ideal_path.mat', 'ideal_x', 'ideal_y');
        n = min(length(ip.ideal_x), 200);
        ideal_x_store(1:n) = ip.ideal_x(1:n);
        ideal_y_store(1:n) = ip.ideal_y(1:n);
        ideal_n = n;
        ideal_loaded = true;
    end
end

if ideal_loaded && ideal_n >= 2
    % Draw connecting lines (magenta, 3px thick)
    for i = 1:ideal_n-1
        x0 = round(ideal_x_store(i)); y0 = round(ideal_y_store(i));
        x1 = round(ideal_x_store(i+1)); y1 = round(ideal_y_store(i+1));
        n_seg = max(abs(x1-x0), abs(y1-y0));
        if n_seg == 0, continue; end
        for s = 0:n_seg
            frac = double(s) / double(n_seg);
            lx = round(double(x0) + frac * double(x1-x0));
            ly = round(double(y0) + frac * double(y1-y0));
            for th = -1:1
                for tv = -1:1
                    rr = ly+tv; cc = lx+th;
                    if rr>=1 && rr<=H && cc>=1 && cc<=W
                        out(rr,cc,1) = 255; out(rr,cc,2) = 0; out(rr,cc,3) = 255;
                    end
                end
            end
        end
    end
    % Draw dots at each clicked point
    for i = 1:ideal_n
        ix = round(ideal_x_store(i)); iy = round(ideal_y_store(i));
        for dr = -4:4
            for dc = -4:4
                if dr*dr+dc*dc <= 16
                    rr = iy+dr; cc = ix+dc;
                    if rr>=1 && rr<=H && cc>=1 && cc<=W
                        out(rr,cc,1) = 255; out(rr,cc,2) = 0; out(rr,cc,3) = 255;
                    end
                end
            end
        end
    end
end

% ================================================================
% OUTPUT
% ================================================================
annotated_frame = real(min(max(out, 0), 255));

end


% ================================================================
% LOCAL FUNCTION: draw_text — render a string using 5x7 pixel font
% ================================================================
function out = draw_text(out, str, x, y, cr, cg, cb, scale, H, W)
    n = length(str);
    for i = 1:n
        bm = char_bitmap(str(i));
        ox = x + (i-1) * 6 * scale;
        for row = 1:7
            for col = 1:5
                if bm(row, col)
                    for sr = 0:scale-1
                        for sc = 0:scale-1
                            pr = y + (row-1)*scale + sr;
                            pc = ox + (col-1)*scale + sc;
                            if pr >= 1 && pr <= H && pc >= 1 && pc <= W
                                out(pr,pc,1) = cr;
                                out(pr,pc,2) = cg;
                                out(pr,pc,3) = cb;
                            end
                        end
                    end
                end
            end
        end
    end
end


% ================================================================
% LOCAL FUNCTION: char_bitmap — return 7x5 logical bitmap for a char
% ================================================================
function bm = char_bitmap(ch)
    persistent font
    if isempty(font)
        font = zeros(128, 7, 'uint8');
        % Space (ASCII 32)
        % already zeros
        % Dash '-' (ASCII 45)
        font(46,:) = [0,0,0,14,0,0,0];
        % Period '.' (ASCII 46)
        font(47,:) = [0,0,0,0,0,0,4];
        % Digits 0-9 (ASCII 48-57)
        font(49,:) = [14,17,17,17,17,17,14];   % 0
        font(50,:) = [4,12,4,4,4,4,14];         % 1
        font(51,:) = [14,17,1,2,4,8,31];        % 2
        font(52,:) = [14,17,1,6,1,17,14];       % 3
        font(53,:) = [2,6,10,18,31,2,2];        % 4
        font(54,:) = [31,16,30,1,1,17,14];      % 5
        font(55,:) = [14,17,16,30,17,17,14];    % 6
        font(56,:) = [31,1,2,4,4,4,4];          % 7
        font(57,:) = [14,17,17,14,17,17,14];    % 8
        font(58,:) = [14,17,17,15,1,17,14];     % 9
        % Colon ':' (ASCII 58)
        font(59,:) = [0,0,4,0,4,0,0];
        % Letters A-Z (ASCII 65-90)
        font(66,:)  = [14,17,17,31,17,17,17];   % A
        font(67,:)  = [30,17,17,30,17,17,30];   % B
        font(68,:)  = [14,17,16,16,16,17,14];   % C
        font(69,:)  = [28,18,17,17,17,18,28];   % D
        font(70,:)  = [31,16,16,30,16,16,31];   % E
        font(71,:)  = [31,16,16,30,16,16,16];   % F
        font(72,:)  = [14,17,16,19,17,17,14];   % G
        font(73,:)  = [17,17,17,31,17,17,17];   % H
        font(74,:)  = [14,4,4,4,4,4,14];        % I
        font(75,:)  = [7,2,2,2,2,18,12];        % J
        font(76,:)  = [17,18,20,24,20,18,17];   % K
        font(77,:)  = [16,16,16,16,16,16,31];   % L
        font(78,:)  = [17,27,21,21,17,17,17];   % M
        font(79,:)  = [17,17,25,21,19,17,17];   % N
        font(80,:)  = [14,17,17,17,17,17,14];   % O
        font(81,:)  = [30,17,17,30,16,16,16];   % P
        font(82,:)  = [14,17,17,17,21,18,13];   % Q
        font(83,:)  = [30,17,17,30,20,18,17];   % R
        font(84,:)  = [14,17,16,14,1,17,14];    % S
        font(85,:)  = [31,4,4,4,4,4,4];         % T
        font(86,:)  = [17,17,17,17,17,17,14];   % U
        font(87,:)  = [17,17,17,17,10,10,4];    % V
        font(88,:)  = [17,17,17,21,21,21,10];   % W
        font(89,:)  = [17,17,10,4,10,17,17];    % X
        font(90,:)  = [17,17,10,4,4,4,4];       % Y
        font(91,:)  = [31,1,2,4,8,16,31];       % Z
    end

    code = min(max(double(ch) + 1, 1), 128);
    bm = false(7, 5);
    mask = uint8([16, 8, 4, 2, 1]);
    for r = 1:7
        v = uint8(font(code, r));
        for c = 1:5
            bm(r, c) = bitand(v, mask(c)) > 0;
        end
    end
end


% ================================================================
% LOCAL FUNCTION: int_to_chars — convert integer to fixed-width string
% ================================================================
function s = int_to_chars(n, w)
    s = char(zeros(1, w) + 48);
    v = abs(round(double(n)));
    for i = w:-1:1
        s(i) = char(48 + mod(v, 10));
        v = floor(v / 10);
    end
end
