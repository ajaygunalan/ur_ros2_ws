addpath(genpath(fullfile(...
	'..', '..', '..', '..', '..', '..', '..', '..', 'beamforming_code', 'image_quality_metrics', 'utils')));

load_data;

output_img_dir_name = fullfile('.', 'img');
createMissingDirectories(output_img_dir_name);

phant_sys_ab_pred_err_vec = zeros(size(phant_id_vec));
phant_sys_c_pred_err_vec = zeros(size(phant_id_vec));

phant_sys_ab_r_sq_vec = zeros(size(phant_id_vec));
phant_sys_c_r_sq_vec = zeros(size(phant_id_vec));

figure(1);
set(gcf, 'Position', [10, 10, 1000, 500]);

fprintf('Phantom:\n');

for i0 = 1:size(phant_time_mat, 1)
	p0 = phant_id_vec(i0);

	[phant_sys_ab_pred_err_vec(i0), phant_sys_ab_r_sq_vec(i0), phant_sys_c_pred_err_vec(i0), ...
		phant_sys_c_r_sq_vec(i0)] = polyRegressionCatheterBase(...
		phant_sys_ab_struct(p0), phant_sys_c_struct(p0), phant_time_mat(i0, :));
	fprintf(...
		'\tTrial %d: %.3e, %.3e, %.3f, %.3f\n', ...
		i0, phant_sys_ab_pred_err_vec(i0), phant_sys_c_pred_err_vec(i0), phant_sys_ab_r_sq_vec(i0), ...
		phant_sys_c_r_sq_vec(i0));
end

vivo_sys_ab_pred_err_vec = zeros(size(vivo_id_vec));
vivo_sys_c_pred_err_vec = zeros(size(vivo_id_vec));

vivo_sys_ab_r_sq_vec = zeros(size(vivo_id_vec));
vivo_sys_c_r_sq_vec = zeros(size(vivo_id_vec));

fprintf('In vivo:\n');

for i0 = 1:size(vivo_time_mat, 1)
	v0 = vivo_id_vec(i0);

	[vivo_sys_ab_pred_err_vec(i0), vivo_sys_ab_r_sq_vec(i0), vivo_sys_c_pred_err_vec(i0), vivo_sys_c_r_sq_vec(i0)] = ...
		polyRegressionCatheterBase(...
		vivo_sys_ab_struct(v0), vivo_sys_c_struct(v0), vivo_time_mat(i0, :), ...
		'ptSrcAxialLimits', vivo_pt_src_ax_limit_mat(i0, :));
	fprintf(...
		'\tTrial %d: %.3e, %.3e, %.3f, %.3f\n', ...
		i0, vivo_sys_ab_pred_err_vec(i0), vivo_sys_c_pred_err_vec(i0), vivo_sys_ab_r_sq_vec(i0), ...
		vivo_sys_c_r_sq_vec(i0));
end
