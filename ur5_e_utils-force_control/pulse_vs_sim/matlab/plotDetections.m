function plotDetections(detection_msg, probe_pose_base, varargin)
	p = inputParser();

	addParameter(p, 'lineWidth', 1.5, @isnumeric);
	addParameter(p, 'markerSize', 100, @isnumeric);
	addParameter(p, 'markerStyle', 'x', @ischar);
	addParameter(p, 'srcColor', [0, 0.4470, 0.7410], @isnumeric);
	addParameter(p, 'refColor', [0.8500, 0.3250, 0.0980], @isnumeric);
	addParameter(p, 'initialTile', 1, @isnumeric);

	parse(p, varargin{:});

	line_width = p.Results.lineWidth;
	marker_size = p.Results.markerSize;
	marker_style = p.Results.markerStyle;
	src_color = p.Results.srcColor;
	ref_color = p.Results.refColor;
	initial_tile = p.Results.initialTile;

	detection_mat = zeros(length(detection_msg.Detections), 4);

	for i0 = 1:size(detection_mat, 1)
		detection_mat(i0, 1) = detection_msg.Detections(i0).Results.Id;
		detection_mat(i0, 2) = detection_msg.Detections(i0).Results.Pose.Pose.Position.X;
		detection_mat(i0, 3) = detection_msg.Detections(i0).Results.Pose.Pose.Position.Y;
		detection_mat(i0, 4) = detection_msg.Detections(i0).Results.Pose.Pose.Position.Z;
		fprintf(...
			'%d: [%.2f, %.2f, %.2f] mm\n', detection_mat(i0, 1), 1e3 * detection_mat(i0, 2), ...
			1e3 * detection_mat(i0, 3), 1e3 * detection_mat(i0, 4));
	end

	detection_mat_probe = [detection_mat(:, 2:4)'; ones(1, size(detection_mat, 1))];
	detection_mat_base = probe_pose_base * detection_mat_probe;
	detection_mat(:, 2:4) = detection_mat_base(1:3, :)';

	src_id_vec = find(detection_mat(:, 1) == 0);
	ref_id_vec = find(detection_mat(:, 1) == 1);

	% Plot the xz-plane
	nexttile(initial_tile);
	scatter(...
		1e3 * detection_mat(src_id_vec, 2), 1e3 * detection_mat(src_id_vec, 4), ...
		'CData', src_color, 'Marker', marker_style, 'LineWidth', line_width, 'SizeData', marker_size);
	hold on
	scatter(...
		1e3 * detection_mat(ref_id_vec, 2), 1e3 * detection_mat(ref_id_vec, 4), ...
		'CData', ref_color, 'Marker', marker_style, 'LineWidth', line_width, 'SizeData', marker_size);

	% Plot the xy-plane
	nexttile(initial_tile + 1);
	scatter(...
		1e3 * detection_mat(src_id_vec, 2), 1e3 * detection_mat(src_id_vec, 3), ...
		'CData', src_color, 'Marker', marker_style, 'LineWidth', line_width, 'SizeData', marker_size);
	hold on
	scatter(...
		1e3 * detection_mat(ref_id_vec, 2), 1e3 * detection_mat(ref_id_vec, 3), ...
		'CData', ref_color, 'Marker', marker_style, 'LineWidth', line_width, 'SizeData', marker_size);
end
