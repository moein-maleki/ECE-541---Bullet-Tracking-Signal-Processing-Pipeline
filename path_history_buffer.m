function [path_xs, path_ys, path_confs, path_states, path_len] = ...
    path_history_buffer(path_x, path_y, confidence, current_state, ...
                           tracking_active)
%#codegen
% PATH_HISTORY_BUFFER_V1
%
% Accumulates trajectory points frame by frame.
% Outputs fixed-size arrays padded with zeros beyond path_len.
%
% Inputs:
%   path_x, path_y    : double, current predicted/detected position
%   confidence         : double, detection confidence at this position
%   current_state      : int32, state machine output (0-3)
%   tracking_active    : boolean, from trajectory integrator
%
% Outputs:
%   path_xs, path_ys   : MAX_PATH x 1 double, accumulated positions
%   path_confs         : MAX_PATH x 1 double, confidence per point
%   path_states        : MAX_PATH x 1 int32, state at each point
%   path_len           : int32, number of valid entries
%
% Design note: fixed-size output arrays are required for Simulink
% signal dimension propagation. MAX_PATH must be >= total flight
% frames. At 30fps and ~1s flight, 60 is sufficient. Set to 120
% for safety (4s at 30fps).

MAX_PATH = 120;

persistent xs ys confs states len_count

if isempty(xs)
    xs     = zeros(MAX_PATH, 1);
    ys     = zeros(MAX_PATH, 1);
    confs  = zeros(MAX_PATH, 1);
    states = zeros(MAX_PATH, 1, 'int32');
    len_count = int32(0);
end

% Only record when tracking is active and we have a valid position
if tracking_active && path_x > 0 && path_y > 0
    if len_count < int32(MAX_PATH)
        len_count = len_count + int32(1);
        idx = double(len_count);
        xs(idx)     = path_x;
        ys(idx)     = path_y;
        confs(idx)  = confidence;
        states(idx) = current_state;
    end
end

path_xs     = xs;
path_ys     = ys;
path_confs  = confs;
path_states = states;
path_len    = len_count;

end