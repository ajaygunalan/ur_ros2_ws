function plotSource(tgt_pose_base, varargin)
	p = inputParser();

	addParameter(p, 'markerSize', 100, @isnumeric);
	addParameter(p, 'markerStyle', 'o', @ischar);
	addParameter(p, 'plotColor', [1, 0, 0], @isnumeric);
	addParameter(p, 'initialTile', 1, @isnumeric);

	parse(p, varargin{:});

	marker_size = p.Results.markerSize;
	marker_style = p.Results.markerStyle;
	plot_color = p.Results.plotColor;
	initial_tile = p.Results.initialTile;

	% Plot the xz-plane
	nexttile(initial_tile);
	scatter(...
		1e3 * tgt_pose_base(1, 4), 1e3 * tgt_pose_base(3, 4), ...
		'Marker', marker_style, 'MarkerEdgeColor', plot_color, 'MarkerFaceColor', plot_color, 'SizeData', marker_size);
	hold on

	% Plot the xy-plane
	nexttile(initial_tile + 1);
	scatter(...
		1e3 * tgt_pose_base(1, 4), 1e3 * tgt_pose_base(2, 4), ...
		'Marker', marker_style, 'MarkerEdgeColor', plot_color, 'MarkerFaceColor', plot_color, 'SizeData', marker_size);
	hold on
end
