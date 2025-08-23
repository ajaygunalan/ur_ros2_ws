function plotRobotPositions(sys_ab_struct, sys_c_struct, input_time_vec, output_img_file_base_name, varargin)
	p = inputParser();

	addParameter(p, 'outputImgDirName', fullfile('.', 'img'), @ischar);

	parse(p, varargin{:});

	output_img_dir_name = p.Results.outputImgDirName;

	sys_ab_id_vec = find(sys_ab_struct.time >= input_time_vec(1) & sys_ab_struct.time <= input_time_vec(2));
	sys_c_id_vec = find(sys_c_struct.time >= input_time_vec(1) & sys_c_struct.time <= input_time_vec(2));

	robot_coords_str_list = {'x', 'y', 'z'};

	for i0 = 1:length(robot_coords_str_list)
		plot(...
			sys_ab_struct.time(sys_ab_id_vec), ...
			1e3 .* sys_ab_struct.robot_program_running(sys_ab_id_vec) .* sys_ab_struct.tf_trans(sys_ab_id_vec, i0), ...
			'DisplayName', 'System A/B');
		hold on
		plot(...
			sys_c_struct.time(sys_c_id_vec), ...
			1e3 .* sys_c_struct.robot_program_running(sys_c_id_vec) .* sys_c_struct.tf_trans(sys_c_id_vec, i0), ...
			'DisplayName', 'System C');
		hold off
		legend('location', 'southeast');

		xlabel('Time [s]');
		ylabel('Robot Position [mm]');

		output_img_file_name = fullfile(...
			output_img_dir_name, ['raw_robot_', robot_coords_str_list{i0}, output_img_file_base_name]);
		fprintf(['Saving file ''', output_img_file_name, '''...\n']);
		saveas(gcf, output_img_file_name);
	end
end
