function plotNumDetections(sys_ab_struct, sys_c_struct, input_time_vec, output_img_file_base_name, varargin)
	p = inputParser();

	addParameter(p, 'outputImgDirName', fullfile('.', 'img'), @ischar);

	parse(p, varargin{:});

	output_img_dir_name = p.Results.outputImgDirName;

	sys_ab_id_vec = find(sys_ab_struct.time >= input_time_vec(1) & sys_ab_struct.time <= input_time_vec(2));
	sys_c_id_vec = find(sys_c_struct.time >= input_time_vec(1) & sys_c_struct.time <= input_time_vec(2));

	plot(...
		sys_ab_struct.time(sys_ab_id_vec), ...
		sys_ab_struct.robot_program_running(sys_ab_id_vec) .* sys_ab_struct.num_detections(sys_ab_id_vec, 1), ...
		'DisplayName', 'System A/B');
	hold on
	plot(...
		sys_c_struct.time(sys_c_id_vec), ...
		sys_c_struct.robot_program_running(sys_c_id_vec) .* sys_c_struct.num_detections(sys_c_id_vec, 1), ...
		'DisplayName', 'System C');
	hold off
	legend();

	xlabel('Time [s]');
	ylabel('Num Sources');

	output_img_file_name = fullfile(output_img_dir_name, ['raw_num_src', output_img_file_base_name]);
	fprintf(['Saving file ''', output_img_file_name, '''...\n']);
	saveas(gcf, output_img_file_name);
end
