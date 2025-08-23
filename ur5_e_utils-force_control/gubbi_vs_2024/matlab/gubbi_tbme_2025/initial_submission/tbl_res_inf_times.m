addpath(genpath(fullfile(...
	'..', '..', '..', '..', '..', '..', '..', '..', 'beamforming_code', 'image_quality_metrics', 'utils')));

load_data;

phant_sys_ab_inf_time_mean_vec = zeros(size(phant_id_vec));
phant_sys_ab_inf_time_std_vec = zeros(size(phant_id_vec));
phant_sys_c_inf_time_mean_vec = zeros(size(phant_id_vec));
phant_sys_c_inf_time_std_vec = zeros(size(phant_id_vec));

for i0 = 1:length(phant_id_vec)
	p0 = phant_id_vec(i0);
	[phant_sys_ab_inf_times, phant_sys_c_inf_times] = computeInferenceTimes(...
		phant_sys_ab_struct(p0), phant_sys_c_struct(p0), phant_time_mat(i0, :));
	phant_sys_ab_inf_time_mean_vec(i0) = mean(phant_sys_ab_inf_times);
	phant_sys_ab_inf_time_std_vec(i0) = std(phant_sys_ab_inf_times);
	phant_sys_c_inf_time_mean_vec(i0) = mean(phant_sys_c_inf_times);
	phant_sys_c_inf_time_std_vec(i0) = std(phant_sys_c_inf_times);
end

vivo_sys_ab_inf_time_mean_vec = zeros(size(vivo_id_vec));
vivo_sys_ab_inf_time_std_vec = zeros(size(vivo_id_vec));
vivo_sys_c_inf_time_mean_vec = zeros(size(vivo_id_vec));
vivo_sys_c_inf_time_std_vec = zeros(size(vivo_id_vec));

for i0 = 1:length(vivo_id_vec)
	p0 = vivo_id_vec(i0);
	[vivo_sys_ab_inf_times, vivo_sys_c_inf_times] = computeInferenceTimes(...
		vivo_sys_ab_struct(p0), vivo_sys_c_struct(p0), vivo_time_mat(i0, :));
	vivo_sys_ab_inf_time_mean_vec(i0) = mean(vivo_sys_ab_inf_times);
	vivo_sys_ab_inf_time_std_vec(i0) = std(vivo_sys_ab_inf_times);
	vivo_sys_c_inf_time_mean_vec(i0) = mean(vivo_sys_c_inf_times);
	vivo_sys_c_inf_time_std_vec(i0) = std(vivo_sys_c_inf_times);
end

fprintf('\n\\begin{tabular}{lrrrr}\n');
fprintf('\\toprule\n');
fprintf('\\multicolumn{2}{c}{\\multirow{2}{*}{\\textbf{Trial Number}}} ');
fprintf('& \\multicolumn{3}{c}{\\textbf{Inference Time [ms]}} \\\\\n');
fprintf('& & \\multicolumn{1}{c}{DetectionNet} & \\multicolumn{1}{c}{WaveSegNet-1} & \\multicolumn{1}{c}{WaveSegNet-2} \\\\ \\midrule\n');
fprintf('\\multirow{5}{*}{Phantom} ');

for i0 = 1:length(phant_id_vec)
	fprintf(...
		'& %d & $%.1f \\pm %.1f$ & - & $%.1f \\pm %.1f$ \\\\\n', i0, ...
		1e3 * phant_sys_ab_inf_time_mean_vec(i0), 1e3 * phant_sys_ab_inf_time_std_vec(i0), ...
		1e3 * phant_sys_c_inf_time_mean_vec(i0), 1e3 * phant_sys_c_inf_time_std_vec(i0));
end

fprintf('\\midrule\n');
fprintf('\\multirow{4}{*}{\\textit{In Vivo}} ');

for i0 = 1:2
	fprintf(...
		'& %d & $%.1f \\pm %.1f$ & - & $%.1f \\pm %.1f$ \\\\\n', i0, ...
		1e3 * vivo_sys_ab_inf_time_mean_vec(i0), 1e3 * vivo_sys_ab_inf_time_std_vec(i0), ...
		1e3 * vivo_sys_c_inf_time_mean_vec(i0), 1e3 * vivo_sys_c_inf_time_std_vec(i0));
end

for i0 = 3:4
	fprintf(...
		'& %d & - & $%.1f \\pm %.1f$ & $%.1f \\pm %.1f$ \\\\\n', i0, ...
		1e3 * vivo_sys_ab_inf_time_mean_vec(i0), 1e3 * vivo_sys_ab_inf_time_std_vec(i0), ...
		1e3 * vivo_sys_c_inf_time_mean_vec(i0), 1e3 * vivo_sys_c_inf_time_std_vec(i0));
end

fprintf('\\bottomrule\n');
fprintf('\\end{tabular}\n\n');
