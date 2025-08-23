function plotTarget(target_pose_base, target_pose_valid, varargin)
	p = inputParser();

	addParameter(p, 'lineWidth', 1.5, @isnumeric);
	addParameter(p, 'markerSize', 100, @isnumeric);
	addParameter(p, 'markerStyle', '+', @ischar);
	addParameter(p, 'validColor', [0, 1, 0], @isnumeric);
	addParameter(p, 'invalidColor', [1, 0, 0], @isnumeric);
	addParameter(p, 'initialTile', 1, @isnumeric);

	parse(p, varargin{:});

	line_width = p.Results.lineWidth;
	marker_size = p.Results.markerSize;
	marker_style = p.Results.markerStyle;
	initial_tile = p.Results.initialTile;

	if target_pose_valid
		color_vec = p.Results.validColor;
	else
		color_vec = p.Results.invalidColor;
	end

	% Plot the xz-plane
	nexttile(initial_tile);
	scatter(...
		1e3 * target_pose_base(1), 1e3 * target_pose_base(3), ...
		'CData', color_vec, 'Marker', marker_style, 'LineWidth', line_width, 'SizeData', marker_size);
	hold on

	% Plot the xy-plane
	nexttile(initial_tile + 1);
	scatter(...
		1e3 * target_pose_base(1), 1e3 * target_pose_base(2), ...
		'CData', color_vec, 'Marker', marker_style, 'LineWidth', line_width, 'SizeData', marker_size);
	hold on
end
