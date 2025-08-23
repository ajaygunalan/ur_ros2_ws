addpath(genpath(fullfile(...
	'..', '..', '..', '..', '..', '..', '..', '..', 'beamforming_code', 'image_quality_metrics', 'utils')));

load_data;

phant_dt_dist_vec = zeros(size(phant_time_mat, 1), 1);
vivo_dt_dist_vec = zeros(size(vivo_time_mat, 1), 1);

fprintf('\n');
fprintf('\\begin{tabular}{llll}\n');
fprintf('\\toprule\n');
fprintf('\\multicolumn{2}{c}{\\multirow{2}{*}{\\textbf{Trial Number}}} & \\multicolumn{2}{c}{\\textbf{Tracking Error [mm]}} \\\\\n');
fprintf('& & \\multicolumn{1}{c}{DetectionNet} & \\multicolumn{1}{c}{WaveSegNet-1} \\\\\n');
fprintf('\\midrule\n');
fprintf('\\multirow{5}{*}{Phantom}\n');

for i0 = 1:length(phant_dt_dist_vec)
	p0 = phant_id_vec(i0);
	phant_dt_dist_vec(i0) = computeRobotDistances(phant_sys_ab_struct(p0), phant_time_mat(i0, :));
	fprintf('& %d & %.2f & - \\\\\n', i0, 1e3 * abs(phant_gt_dist_vec(i0) - phant_dt_dist_vec(i0)));
end

fprintf('\\multirow{4}{*}{In Vivo}\n');

for i0 = 1:2
	v0 = vivo_id_vec(i0);
	vivo_dt_dist_vec(i0) = computeRobotDistances(vivo_sys_ab_struct(v0), vivo_time_mat(i0, :));
	fprintf(...
		'& %d & %.2f & - \\\\\n', i0, 1e3 * abs(vivo_gt_dist_vec(i0) - vivo_dt_dist_vec(i0)));
end

for i0 = 3:4
	v0 = vivo_id_vec(i0);
	vivo_dt_dist_vec(i0) = computeRobotDistances(vivo_sys_ab_struct(v0), vivo_time_mat(i0, :));
	fprintf(...
		'& %d & - & %.2f \\\\\n', i0, 1e3 * abs(vivo_gt_dist_vec(i0) - vivo_dt_dist_vec(i0)));
end

fprintf('\\bottomrule\n');
fprintf('\\end{tabular}\n\n');
