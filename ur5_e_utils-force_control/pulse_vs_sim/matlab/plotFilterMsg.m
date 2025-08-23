function plotFilterMsg(msg, varargin)
	p = inputParser();

	addParameter(p, 'markerSize', 100, @isnumeric);
	addParameter(p, 'markerStyle', 'o', @ischar);
	addParameter(p, 'plotColor', [0, 0, 1], @isnumeric);
	addParameter(p, 'initialTile', 1, @isnumeric);

	parse(p, varargin{:});

	marker_size = p.Results.markerSize;
	marker_style = p.Results.markerStyle;
	plot_color = p.Results.plotColor;
	initial_tile = p.Results.initialTile;

	if ~isempty(msg.Poses)
		tgt_pose_mat = zeros(3, length(msg.Poses));

		for i0 = 1:length(msg.Poses)
			tgt_pose_mat(:, i0) = [...
				msg.Poses(i0).Pose.Position.X, ...
				msg.Poses(i0).Pose.Position.Y, ...
				msg.Poses(i0).Pose.Position.Z];
		end

		% Plot the xz-plane
		nexttile(initial_tile);
		scatter(...
			1e3 * tgt_pose_mat(1, :), 1e3 * tgt_pose_mat(3, :), ...
			'Marker', marker_style, 'MarkerEdgeColor', plot_color, 'MarkerFaceColor', plot_color, ...
			'SizeData', marker_size);
		hold on

		% Plot the xy-plane
		nexttile(initial_tile + 1);
		scatter(...
			1e3 * tgt_pose_mat(1, :), 1e3 * tgt_pose_mat(2, :), ...
			'Marker', marker_style, 'MarkerEdgeColor', plot_color, 'MarkerFaceColor', plot_color, ...
			'SizeData', marker_size);
		hold on
	end
end
