addpath(genpath(fullfile(...
	'..', '..', '..', '..', '..', '..', '..', '..', 'beamforming_code', 'image_quality_metrics', 'utils')));

load_data;

output_img_dir_name = fullfile('.', 'img');
createMissingDirectories(output_img_dir_name);

for i0 = 1:size(phant_time_mat, 1)
	p0 = phant_id_vec(i0);

	plotCatheterPca(...
		phant_sys_ab_struct(p0), phant_sys_c_struct(p0), phant_time_mat(i0, :), sprintf('_phant_t%02d.png', i0));
end

for i0 = 1:size(vivo_time_mat, 1)
	v0 = vivo_id_vec(i0);

	plotCatheterPca(...
		vivo_sys_ab_struct(v0), vivo_sys_c_struct(v0), vivo_time_mat(i0, :), sprintf('_vivo_t%02d.png', i0), ...
		'ptSrcAxialLimits', vivo_pt_src_ax_limit_mat(i0, :), 'legendStr', vivo_sys_str_list{i0});
end

