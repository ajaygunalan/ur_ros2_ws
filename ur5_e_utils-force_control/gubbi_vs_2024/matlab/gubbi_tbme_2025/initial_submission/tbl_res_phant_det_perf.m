addpath(genpath(fullfile(...
	'..', '..', '..', '..', '..', '..', '..', '..', 'beamforming_code', 'image_quality_metrics', 'utils')));

load_data;

sys_ab_precision_vec = zeros(size(phant_time_mat, 1), 1);
sys_ab_recall_vec = zeros(size(sys_ab_precision_vec));
sys_ab_f1_score_vec = zeros(size(sys_ab_precision_vec));

sys_c_precision_vec = zeros(size(phant_time_mat, 1), 1);
sys_c_recall_vec = zeros(size(sys_c_precision_vec));
sys_c_f1_score_vec = zeros(size(sys_c_precision_vec));

for i0 = 1:size(phant_time_mat, 1)
	p0 = phant_id_vec(i0);

	[...
		sys_ab_precision_vec(i0), sys_ab_recall_vec(i0), sys_ab_f1_score_vec(i0), ...
		sys_c_precision_vec(i0), sys_c_recall_vec(i0), sys_c_f1_score_vec(i0)] = computePrecisionMetrics(...
		phant_sys_ab_struct(p0), phant_sys_c_struct(p0), phant_time_mat(i0, :));
end

fprintf('\n');
fprintf('\\begin{tabular}{clrrrrr}\n');
fprintf('\\toprule\n');
fprintf('\\multicolumn{1}{c}{\\multirow{2}{*}{\\textbf{System}}} ');
fprintf('& \\multicolumn{1}{c}{\\multirow{2}{*}{\\textbf{Performance Metric}}} ');
fprintf('& \\multicolumn{5}{c}{\\textbf{Trial Number}} \\\\\n');
fprintf('\\multicolumn{2}{}{} & 1 & 2 & 3 & 4 & 5\\\\ \\midrule\n');
fprintf(...
	'\\multirow{4}{*}{\\textbf{DetectionNet}} & Recall [\\%%] & %.1f & %.1f & %.1f & %.1f & %.1f \\\\\n', ...
	100.0 .* sys_ab_recall_vec);
fprintf('& Precision [\\%%] & %.1f & %.1f & %.1f & %.1f & %.1f \\\\\n', 100.0 .* sys_ab_precision_vec);
fprintf(...
	'& F1 Score [\\%%] & %.1f & %.1f & %.1f & %.1f & %.1f \\\\ \\midrule\n', ...
	100.0 .* sys_ab_f1_score_vec);
% fprintf('& Tracking Error [mm] & 0.52 & 2.75 & 3.50 & 3.06 & 3.12 \\\\ \\cmidrule(l){1-7}\n');
fprintf(...
	'\\multirow{3}{*}{\\textbf{WaveSegNet-2}} & Recall [\\%%] & %.1f & %.1f & %.1f & %.1f & %.1f \\\\\n', ...
	100.0 .* sys_c_recall_vec);
fprintf('& Precision [\\%%] & %.1f & %.1f & %.1f & %.1f & %.1f \\\\\n', 100.0 .* sys_c_precision_vec);
fprintf(...
	'& F1 Score [\\%%] & %.1f & %.1f & %.1f & %.1f & %.1f \\\\ \\bottomrule\n', 100.0 .* sys_c_f1_score_vec);
fprintf('\\end{tabular}\n');
