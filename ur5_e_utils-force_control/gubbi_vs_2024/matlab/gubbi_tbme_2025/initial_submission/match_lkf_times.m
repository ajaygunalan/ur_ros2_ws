addpath(genpath(fullfile(...
	'..', '..', '..', '..', '..', '..', '..', '..', 'beamforming_code', 'image_quality_metrics', 'utils')));

load_data;

output_img_dir_name = fullfile('.', 'img');
createMissingDirectories(output_img_dir_name);

for i0 = 1:size(phant_time_mat, 1)
	p0 = phant_id_vec(i0);

	[phant_mtlkf_time_vec, phant_mtlkf_state_vec] = matchLkfTimes(...
		phant_sys_ab_struct(p0), phant_mtlkf_struct(p0), phant_time_mat(i0, :));
	phant_mtlkf_state_vec(~isfinite(phant_mtlkf_state_vec)) = 5;

	scatter(phant_mtlkf_time_vec, phant_mtlkf_state_vec);
	output_img_file_name = fullfile(output_img_dir_name, sprintf('mtlkf_state_phant_%02d.png', i0));
	fprintf(['Saving file ''', output_img_file_name, '''...\n']);
	saveas(gcf, output_img_file_name);
end

for i0 = 1:size(vivo_time_mat, 1)
	v0 = vivo_id_vec(i0);

	[vivo_mtlkf_time_vec, vivo_mtlkf_state_vec] = matchLkfTimes(...
		vivo_sys_ab_struct(v0), vivo_mtlkf_struct(v0), vivo_time_mat(i0, :));
	vivo_mtlkf_state_vec(~isfinite(vivo_mtlkf_state_vec)) = 5;

	scatter(vivo_mtlkf_time_vec, vivo_mtlkf_state_vec);
	output_img_file_name = fullfile(output_img_dir_name, sprintf('mtlkf_state_vivo_%02d.png', i0));
	fprintf(['Saving file ''', output_img_file_name, '''...\n']);
	saveas(gcf, output_img_file_name);
end

