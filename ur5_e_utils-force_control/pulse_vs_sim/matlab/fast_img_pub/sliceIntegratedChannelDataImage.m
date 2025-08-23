function [ch_data_slice] = sliceIntegratedChannelDataImage(input_file_name, metadata, x, x_offset, x_res, varargin)
	p = inputParser();

	addParameter(p, 'depthShiftPx', 0, @isnumeric);

	parse(p, varargin{:});

	depth_shift_px = round(p.Results.depthShiftPx);

	x_start_id = floor((abs(x) - x_offset) / x_res) + 1;
	x_end_id = x_start_id + metadata.trans.elements.num - 1;
	ch_data = load(input_file_name);
	ch_data_slice = (double(ch_data.ch_data(x_start_id:x_end_id, :)) .* (ch_data.ch_data_max / 255.0)) ...
		+ ch_data.ch_data_min;

	if x > 0
		ch_data_slice = flip(ch_data_slice);
	end

	ch_data_slice = ch_data_slice';

	% Shift the feature down (used for reflection artifacts).
	if depth_shift_px > 0
		if depth_shift_px <= size(ch_data_slice, 1)
			% Downshift and replace the empty top region with the median value.
			ch_data_slice = circshift(ch_data_slice, depth_shift_px, 1);
			ch_data_slice(1:depth_shift_px, :) = median(ch_data_slice(:));
		else
			% The down shift is larger than the image dimensions, so replace the entire image with the median value.
			ch_data_slice = median(ch_data_slice(:));
		end
	end
end
