function [sys_ab_pred_err, sys_ab_r_sq, sys_c_pred_err, sys_c_r_sq] = polyRegressionCatheterBase(...
		sys_ab_struct, sys_c_struct, input_time_vec, varargin)
	p = inputParser();

	addParameter(p, 'outputImgFileBaseName', '_phant_01.png', @ischar);
	addParameter(p, 'outputImgDirName', fullfile('.', 'img'), @ischar);
	addParameter(p, 'ptSrcAxialLimits', [20e-3, 40e-3], @isnumeric);
	addParameter(p, 'saveFile', true, @islogical);

	parse(p, varargin{:});

	pt_src_axial_limits = p.Results.ptSrcAxialLimits;

	sys_ab_id_vec = find(...
		sys_ab_struct.pt_src_time >= input_time_vec(1) ...
		& sys_ab_struct.pt_src_time <= input_time_vec(2) ...
		& sys_ab_struct.pt_src_p42v_link1(:, 3) >= pt_src_axial_limits(1) ...
		& sys_ab_struct.pt_src_p42v_link1(:, 3) <= pt_src_axial_limits(2));
	sys_c_id_vec = find(...
		sys_c_struct.pt_src_time >= input_time_vec(1) ...
		& sys_c_struct.pt_src_time <= input_time_vec(2) ...
		& sys_c_struct.pt_src_p42v_link1(:, 3, end) >= pt_src_axial_limits(1) ...
		& sys_c_struct.pt_src_p42v_link1(:, 3, end) <= pt_src_axial_limits(2));

	sys_ab_pos_base = [sys_ab_struct.pt_src_base_link(sys_ab_id_vec, :), ones(length(sys_ab_id_vec), 1)];
	sys_c_pos_base = [sys_c_struct.pt_src_base_link(sys_c_id_vec, :), ones(length(sys_c_id_vec), 1)];

	% Perform linear regression with along-track and vertical position components.
	sys_ab_x_mat = [ones(size(sys_ab_pos_base, 1), 1), sys_ab_pos_base(:, 2)];
	sys_ab_y_vec = sys_ab_pos_base(:, 3);
	sys_ab_coeffs = sys_ab_x_mat \ sys_ab_y_vec;

	% Compute R^2 to characterize linear regression quality.
	sys_ab_predictions = sys_ab_x_mat * sys_ab_coeffs;
	sys_ab_pred_err = sum((sys_ab_y_vec - sys_ab_predictions) .^ 2);
	sys_ab_y_var = (length(sys_ab_y_vec) - 1) * var(sys_ab_y_vec);
	sys_ab_r_sq = 1.0 - (sys_ab_pred_err / sys_ab_y_var);

	% Repeat for System C.
	sys_c_x_mat = [ones(size(sys_c_pos_base, 1), 1), sys_c_pos_base(:, 2)];
	sys_c_y_vec = sys_c_pos_base(:, 3);
	sys_c_coeffs = sys_c_x_mat \ sys_c_y_vec;

	sys_c_predictions = sys_c_x_mat * sys_c_coeffs;
	sys_c_pred_err = sum((sys_c_y_vec - sys_c_predictions) .^ 2);
	sys_c_y_var = (length(sys_c_y_vec) - 1) * var(sys_c_y_vec);
	sys_c_r_sq = 1.0 - (sys_c_pred_err / sys_c_y_var);

	tiledlayout(1, 2);
	nexttile(1);
	scatter(sys_ab_x_mat(:, 2), sys_ab_y_vec);
	hold on
	plot(sys_ab_x_mat(:, 2), sys_ab_predictions);
	hold off;

	nexttile(2);
	scatter(sys_c_x_mat(:, 2), sys_c_y_vec);
	hold on
	plot(sys_c_x_mat(:, 2), sys_c_predictions);
	hold off;

	output_img_dir_name = p.Results.outputImgDirName;
	output_img_file_base_name = p.Results.outputImgFileBaseName;
	output_img_file_name = fullfile(output_img_dir_name, ['lin_reg', output_img_file_base_name]);
	fprintf(['Saving file ''', output_img_file_name, '''...\n']);
	saveas(gcf, output_img_file_name);
end
