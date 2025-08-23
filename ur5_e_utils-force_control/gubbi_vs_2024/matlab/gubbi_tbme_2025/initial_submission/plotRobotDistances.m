function plotRobotDistances(sys_ab_struct, sys_c_struct, input_time_vec, output_img_file_base_name, varargin)
	p = inputParser();

	addParameter(p, 'outputImgDirName', fullfile('.', 'img'), @ischar);

	parse(p, varargin{:});

	output_img_dir_name = p.Results.outputImgDirName;

	sys_ab_id_vec = find(sys_ab_struct.time >= input_time_vec(1) & sys_ab_struct.time <= input_time_vec(2));
	sys_ab_robot_dist_vec = sqrt(...
		sum((sys_ab_struct.tf_trans(sys_ab_id_vec, :) - sys_ab_struct.tf_trans(sys_ab_id_vec(1), :)) .^ 2, 1));

	sys_c_id_vec = find(sys_c_struct.time >= input_time_vec(1) & sys_c_struct.time <= input_time_vec(2));
	sys_c_robot_dist_vec = sqrt(...
		sum((sys_c_struct.tf_trans(sys_c_id_vec, :) - sys_c_struct.tf_trans(sys_c_id_vec(1), :)) .^ 2, 1));

	plot(sys_ab_struct.time(sys_ab_id_vec), 1e3 .* sys_ab_robot_dist_vec, 'DisplayName', 'System A/B');
	hold on
	plot(sys_c_struct.time(sys_c_id_vec), 1e3 .* sys_c_robot_dist_vec, 'DisplayName', 'System C');
	hold off
	legend('location', 'southeast');

	output_img_file_name = fullfile(output_img_dir_name, ['raw_robot_dist', output_img_file_base_name]);
	fprintf(['Saving file ''', output_img_file_name, '''...\n']);
	saveas(gcf, output_img_file_name);
end
