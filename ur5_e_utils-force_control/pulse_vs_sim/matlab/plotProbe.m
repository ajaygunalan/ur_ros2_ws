function plotProbe(probe_pose_base, varargin)
	p = inputParser();

	addParameter(p, 'transElementHeight', 14e-3, @isnumeric);
	addParameter(p, 'transApertureWidth', 19.2e-3, @isnumeric);
	addParameter(p, 'probeColor', [0, 0, 0], @isnumeric);

	parse(p, varargin{:});

	trans_element_height = p.Results.transElementHeight;
	trans_aperture_width = p.Results.transApertureWidth;
	probe_color = p.Results.probeColor;

	probe_coords_probe = [-1, -1, 1, 1, -1; -1, 1, 1, -1, -1; 0, 0, 0, 0, 0; 1, 1, 1, 1, 1] ...
		.* [trans_aperture_width / 2.0, trans_element_height / 2.0, 1, 1]';
	probe_coords_base = probe_pose_base * probe_coords_probe;

	probe_x_probe = [0, trans_aperture_width; 0, 0; 0, 0; 1, 1];
	probe_x_base = probe_pose_base * probe_x_probe;

	nexttile(1);
	plot(1e3 * probe_coords_base(1, :), 1e3 * probe_coords_base(3, :), 'Color', probe_color);
	hold on
	plot(1e3 * probe_x_base(1, :), 1e3 * probe_x_base(3, :), 'Color', probe_color);

	nexttile(2);
	plot(1e3 * probe_coords_base(1, :), 1e3 * probe_coords_base(2, :), 'Color', probe_color);
	hold on
	plot(1e3 * probe_x_base(1, :), 1e3 * probe_x_base(2, :), 'Color', probe_color);
end
