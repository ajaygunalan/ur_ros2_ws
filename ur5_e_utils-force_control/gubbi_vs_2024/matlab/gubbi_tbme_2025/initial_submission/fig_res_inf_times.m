addpath(genpath(fullfile(...
	'..', '..', '..', '..', '..', '..', '..', '..', 'beamforming_code', 'image_quality_metrics', 'utils')));

load_data;

set(...
	0, ...
	'defaultlinelinewidth', 3, ...
	'defaultaxesfontsize', 20, ...
	'defaultlinemarkersize', 7, ...
	'defaulterrorbarlinewidth', 3);

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

figure(1);
set(gcf, 'Position', [10, 10, 900, 500]);

tiledlayout(1, 2, 'Padding', 'loose', 'TileSpacing', 'compact');

nexttile(1);
color_order = get(gca, 'ColorOrder');

phant_fh = bar([phant_sys_ab_inf_time_mean_vec, phant_sys_c_inf_time_mean_vec], 'HandleVisibility', 'off');
phant_fh(2).FaceColor = color_order(3, :);
hold on

% Error bars for DetectionNet.
phant_sys_ab_bar_props = get(phant_fh(1));
phant_sys_ab_x = phant_sys_ab_bar_props.XEndPoints;
phant_sys_ab_y = phant_sys_ab_bar_props.YEndPoints;
errorbar(...
	phant_sys_ab_x, phant_sys_ab_y, phant_sys_ab_inf_time_std_vec, ...
	'Color', [0, 0, 0], 'LineStyle', 'none', 'HandleVisibility', 'off');

% Error bars for WaveSegNet-2.
phant_sys_c_bar_props = get(phant_fh(2));
phant_sys_c_x = phant_sys_c_bar_props.XEndPoints;
phant_sys_c_y = phant_sys_c_bar_props.YEndPoints;
errorbar(...
	phant_sys_c_x, phant_sys_c_y, phant_sys_c_inf_time_std_vec, ...
	'Color', [0, 0, 0], 'LineStyle', 'none', 'HandleVisibility', 'off');
hold off
xlim([0.5, 5.5]);
ylim([0, 2.0]);
xlabel({'Trial Number'; '(a)'});
% ylabel({'{\bfPhantom}'; 'Inference Time [s]'});
ylabel('Inference Time [s]');
title('Phantom');

nexttile(2);
vivo_sys_a_fh = bar(...
	1:2, [vivo_sys_ab_inf_time_mean_vec(1:2), vivo_sys_c_inf_time_mean_vec(1:2)], 'HandleVisibility', 'off');
vivo_sys_a_fh(2).FaceColor = color_order(3, :);
hold on
vivo_sys_b_fh = bar(...
	3:4, [vivo_sys_ab_inf_time_mean_vec(3:4), vivo_sys_c_inf_time_mean_vec(3:4)], 'HandleVisibility', 'off');
vivo_sys_b_fh(1).FaceColor = color_order(2, :);
vivo_sys_b_fh(2).FaceColor = color_order(3, :);

% Error bars for DetectionNet and WaveSegNet-2.
vivo_sys_a_bar_props = get(vivo_sys_a_fh(1));
vivo_sys_a_x = vivo_sys_a_bar_props.XEndPoints;
vivo_sys_a_y = vivo_sys_a_bar_props.YEndPoints;
errorbar(...
	vivo_sys_a_x, vivo_sys_a_y, vivo_sys_ab_inf_time_std_vec(1:2), ...
	'Color', [0, 0, 0], 'LineStyle', 'none', 'HandleVisibility', 'off');

vivo_sys_ac_bar_props = get(vivo_sys_a_fh(2));
vivo_sys_ac_x = vivo_sys_ac_bar_props.XEndPoints;
vivo_sys_ac_y = vivo_sys_ac_bar_props.YEndPoints;
errorbar(...
	vivo_sys_ac_x, vivo_sys_ac_y, vivo_sys_c_inf_time_std_vec(1:2), ...
	'Color', [0, 0, 0], 'LineStyle', 'none', 'HandleVisibility', 'off');

% Error bars for WaveSegNet-1 and WaveSegNet-2.
vivo_sys_b_bar_props = get(vivo_sys_b_fh(1));
vivo_sys_b_x = vivo_sys_b_bar_props.XEndPoints;
vivo_sys_b_y = vivo_sys_b_bar_props.YEndPoints;
errorbar(...
	vivo_sys_b_x, vivo_sys_b_y, vivo_sys_ab_inf_time_std_vec(3:4), ...
	'Color', [0, 0, 0], 'LineStyle', 'none', 'HandleVisibility', 'off');

vivo_sys_bc_bar_props = get(vivo_sys_b_fh(2));
vivo_sys_bc_x = vivo_sys_bc_bar_props.XEndPoints;
vivo_sys_bc_y = vivo_sys_bc_bar_props.YEndPoints;
errorbar(...
	vivo_sys_bc_x, vivo_sys_bc_y, vivo_sys_c_inf_time_std_vec(3:4), ...
	'Color', [0, 0, 0], 'LineStyle', 'none', 'HandleVisibility', 'off');

% Plot a dummy bar plot to get the colors for the legend.
legend_fh = bar(7, [1, 1, 1]);

for i0 = 1:3
	legend_fh(i0).FaceColor = color_order(i0, :);
end

hold off
legend('DetectionNet', 'WaveSegNet-1', 'WaveSegNet-2', 'location', 'northeast');
xlabel({'Trial Number'; '(b)'});
% ylabel({'{\bf{\itIn Vivo}}'; 'Inference Time [s]'});
xticks([1:length(vivo_id_vec)]);
yticklabels({});
title('{\itIn Vivo}');
xlim([0.5, 4.5]);
ylim([0, 2.0]);

output_img_dir_name = fullfile('.', 'img');
createMissingDirectories(output_img_dir_name);
output_img_file_name = fullfile(output_img_dir_name, 'res_inf_times.png');
fprintf(['Saving file ''', output_img_file_name, '''...\n']);
saveas(gcf, output_img_file_name);
