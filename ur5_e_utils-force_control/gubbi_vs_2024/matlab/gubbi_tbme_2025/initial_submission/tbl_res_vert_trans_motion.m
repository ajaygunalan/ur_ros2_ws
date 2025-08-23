addpath(genpath(fullfile(...
	'..', '..', '..', '..', '..', '..', '..', '..', 'beamforming_code', 'image_quality_metrics', 'utils')));

load_data;

phant_dt_dist_vec = zeros(size(phant_time_mat, 1), 1);
vivo_dt_dist_vec = zeros(size(vivo_time_mat, 1), 1);

fprintf('\n');
fprintf('\\begin{tabular}{lc}\n');
fprintf('\\toprule\n');
fprintf('\\multicolumn{1}{c}{\\textbf{Trial Number}} & \\multicolumn{1}{c}{\\textbf{Vertical Transducer Motion Range [mm]}} \\\\\n');
fprintf('\\midrule\n');

for i0 = 1:length(phant_dt_dist_vec)
	p0 = phant_id_vec(i0);
	[~, phant_dt_dist_vec(i0)] = computeRobotDistances(phant_sys_ab_struct(p0), phant_time_mat(i0, :));
	fprintf('Phantom Trial %d & %.2f \\\\\n', i0, 1e3 * phant_dt_dist_vec(i0));
end

fprintf('\\midrule\n');
for i0 = 1:length(vivo_id_vec)
	v0 = vivo_id_vec(i0);
	[~, vivo_dt_dist_vec(i0)] = computeRobotDistances(vivo_sys_ab_struct(v0), vivo_time_mat(i0, :));
	fprintf(...
		'\\textit{In Vivo} Trial %d & %.2f \\\\\n', i0, 1e3 * vivo_dt_dist_vec(i0));
end

fprintf('\\bottomrule\n');
fprintf('\\end{tabular}\n\n');
