function [rgb_out, debug_info] = video_debug(rgb_in)
%#codegen
% Passes through unchanged. Reports signal properties.

rgb_out = rgb_in;

% debug_info: [ndims, dim1, dim2, dim3, min_val, max_val, class_id]
% class_id: 1=uint8, 2=double, 3=single, 4=other
sz = size(rgb_in);
if numel(sz) == 3
    d3 = sz(3);
else
    d3 = 1;
end

mn = double(min(rgb_in(:)));
mx = double(max(rgb_in(:)));

% Detect class
if isa(rgb_in, 'uint8')
    cid = 1;
elseif isa(rgb_in, 'double')
    cid = 2;
elseif isa(rgb_in, 'single')
    cid = 3;
else
    cid = 4;
end

debug_info = [double(numel(sz)), double(sz(1)), double(sz(2)), ...
              double(d3), mn, mx, double(cid)];
end