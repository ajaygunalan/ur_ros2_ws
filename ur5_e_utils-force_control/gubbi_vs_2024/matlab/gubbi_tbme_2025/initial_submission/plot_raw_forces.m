addpath(genpath(fullfile(...
	'..', '..', '..', '..', '..', '..', '..', '..', 'beamforming_code', 'image_quality_metrics', 'utils')));

load_data;

output_img_dir_name = fullfile('.', 'img');
createMissingDirectories(output_img_dir_name);

ft_str_list = {'fx', 'fy', 'fz', 'tx', 'ty', 'tz'};

for i0 = 1:length(phant_netft_struct)
	for i1 = 1:6
		scatter(...
			phant_netft_struct(i0).time, phant_netft_struct(i0).proc_probe(i1, :), '.', ...
			'DisplayName', 'System A/B');

		xlabel('Time [s]');
		ylabel('Force/Torque [N/Nm]');

		output_img_file_name = fullfile(...
			output_img_dir_name, sprintf(['raw_force_', ft_str_list{i1}, '_phant_%02d.png'], i0));
		fprintf(['Saving file ''', output_img_file_name, '''...\n']);
		saveas(gcf, output_img_file_name);
	end
end

for i0 = 1:length(vivo_netft_struct)
	for i1 = 1:6
		scatter(...
			vivo_netft_struct(i0).time, vivo_netft_struct(i0).proc_probe(i1, :), '.', ...
			'DisplayName', 'System A/B');

		xlabel('Time [s]');
		ylabel('Force/Torque [N/Nm]');

		output_img_file_name = fullfile(...
			output_img_dir_name, sprintf(['raw_force_', ft_str_list{i1}, '_vivo_%02d.png'], i0));
		fprintf(['Saving file ''', output_img_file_name, '''...\n']);
		saveas(gcf, output_img_file_name);
	end
end
