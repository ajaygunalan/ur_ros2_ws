addpath(genpath(fullfile(...
	'..', '..', '..', '..', '..', '..', '..', '..', 'beamforming_code', 'image_quality_metrics', 'utils')));

load_data;

output_img_dir_name = fullfile('.', 'img');
createMissingDirectories(output_img_dir_name);

for i0 = 1:length(phant_sys_ab_struct)
	plot(phant_sys_ab_struct(i0).time, phant_sys_ab_struct(i0).robot_program_running, 'DisplayName', 'System A/B');
	hold on
	plot(phant_sys_c_struct(i0).time, phant_sys_c_struct(i0).robot_program_running, 'DisplayName', 'System C');
	hold off
	legend();

	xlabel('Time [s]');
	ylabel('ROS Running');

	output_img_file_name = fullfile(output_img_dir_name, sprintf('raw_robot_running_phant_%02d.png', i0));
	fprintf(['Saving file ''', output_img_file_name, '''...\n']);
	saveas(gcf, output_img_file_name);
end

for i0 = 1:length(vivo_sys_ab_struct)
	plot(vivo_sys_ab_struct(i0).time, vivo_sys_ab_struct(i0).robot_program_running, 'DisplayName', 'System A/B');
	hold on
	plot(vivo_sys_c_struct(i0).time, vivo_sys_c_struct(i0).robot_program_running, 'DisplayName', 'System C');
	hold off
	legend();

	xlabel('Time [s]');
	ylabel('ROS Running');

	output_img_file_name = fullfile(output_img_dir_name, sprintf('raw_robot_running_vivo_%02d.png', i0));
	fprintf(['Saving file ''', output_img_file_name, '''...\n']);
	saveas(gcf, output_img_file_name);
end
