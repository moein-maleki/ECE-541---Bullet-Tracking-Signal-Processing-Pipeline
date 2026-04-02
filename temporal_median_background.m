function [B, background_ready] = temporal_median_background(frame)
%#codegen
% TEMPORAL_MEDIAN_BACKGROUND_V2
%
% Change from v1: added background_ready output flag.
% False until K frames accumulated and median computed.
% All downstream blocks must wait for this flag before operating.

K = 60;

[H, W] = size(frame);

persistent buffer frame_count B_computed B_out

if isempty(buffer)
    buffer      = zeros(H, W, K, 'like', frame);
    frame_count = 0;
    B_computed  = false;
    B_out       = zeros(H, W, 'like', frame);
end

if ~B_computed
    frame_count = frame_count + 1;

    if frame_count <= K
        buffer(:,:,frame_count) = frame;
    end

    if frame_count == K
        sorted_buffer = sort(buffer, 3);
        mid = floor(K/2) + 1;
        B_out = sorted_buffer(:,:,mid);
        B_computed = true;
    end
end

B = B_out;
background_ready = B_computed;

end