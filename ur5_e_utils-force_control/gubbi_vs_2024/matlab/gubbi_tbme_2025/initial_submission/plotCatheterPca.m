function plotCatheterPca(sys_ab_struct, sys_c_struct, input_time_vec, output_img_file_base_name, varargin)
	p = inputParser();

	addParameter(p, 'outputImgDirName', fullfile('.', 'img'), @ischar);
	addParameter(p, 'ptSrcAxialLimits', [20e-3, 40e-3], @isnumeric);
	addParameter(p, 'legendStr', 'DetectionNet', @ischar);

	parse(p, varargin{:});

	output_img_dir_name = p.Results.outputImgDirName;

	pt_src_axial_limits = p.Results.ptSrcAxialLimits;
	legend_str = p.Results.legendStr;

	robot_id_vec = find(sys_ab_struct.time >= input_time_vec(1) & sys_ab_struct.time <= input_time_vec(2));
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

	tf_base_pca = computePcaTransform(sys_ab_struct.tf_trans(robot_id_vec, :)');

	robot_pos_base = [sys_ab_struct.tf_trans(robot_id_vec, :), ones(length(robot_id_vec), 1)];
	robot_pos_pca = tf_base_pca * robot_pos_base';
	robot_pos_pca = robot_pos_pca(1:3, :)';

	sys_ab_pos_base = [sys_ab_struct.pt_src_base_link(sys_ab_id_vec, :), ones(length(sys_ab_id_vec), 1)];
	sys_ab_pos_pca = tf_base_pca * sys_ab_pos_base';
	sys_ab_pos_pca = sys_ab_pos_pca(1:3, :)';

	sys_c_pos_base = [sys_c_struct.pt_src_base_link(sys_c_id_vec, :), ones(length(sys_c_id_vec), 1)];
	sys_c_pos_pca = tf_base_pca * sys_c_pos_base';
	sys_c_pos_pca = sys_c_pos_pca(1:3, :)';

	scatter(1e3 .* robot_pos_base(:, 1), 1e3 .* robot_pos_base(:, 2), '.', 'DisplayName', 'Transducer');
	hold on
	scatter(1e3 .* sys_ab_pos_base(:, 1), 1e3 .* sys_ab_pos_base(:, 2), '.', 'DisplayName', legend_str);
	scatter(1e3 .* sys_c_pos_base(:, 1), 1e3 .* sys_c_pos_base(:, 2), '.', 'DisplayName', 'WaveSegNet-2');
	hold off
	legend();

	xlabel('x [mm]');
	ylabel('y [mm]');

	output_img_file_name = fullfile(output_img_dir_name, ['raw_cath_base', output_img_file_base_name]);
	fprintf(['Saving file ''', output_img_file_name, '''...\n']);
	saveas(gcf, output_img_file_name);

	scatter(1e3 .* robot_pos_pca(:, 1), 1e3 .* robot_pos_pca(:, 2), '.', 'DisplayName', 'Transducer');
	hold on
	scatter(1e3 .* sys_ab_pos_pca(:, 1), 1e3 .* sys_ab_pos_pca(:, 2), '.', 'DisplayName', legend_str);
	scatter(1e3 .* sys_c_pos_pca(:, 1), 1e3 .* sys_c_pos_pca(:, 2), '.', 'DisplayName', 'WaveSegNet-2');
	hold off
	legend();

	xlabel('x [mm]');
	ylabel('y [mm]');

	output_img_file_name = fullfile(output_img_dir_name, ['raw_cath_pca', output_img_file_base_name]);
	fprintf(['Saving file ''', output_img_file_name, '''...\n']);
	saveas(gcf, output_img_file_name);
end
