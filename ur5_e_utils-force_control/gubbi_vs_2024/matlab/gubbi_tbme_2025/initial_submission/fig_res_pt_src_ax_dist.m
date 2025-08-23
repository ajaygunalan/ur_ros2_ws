addpath(genpath(fullfile(...
	'..', '..', '..', '..', '..', '..', '..', '..', 'beamforming_code', 'image_quality_metrics', 'utils')));

load_data;

set(...
	0, ...
	'defaultlinelinewidth', 3, ...
	'defaultaxesfontsize', 18, ...
	'defaultlinemarkersize', 7, ...
	'defaulterrorbarlinewidth', 3);

output_img_dir_name = fullfile('.', 'img');
createMissingDirectories(output_img_dir_name);

phant_trial_id_vec = [1:5];
phant_sys_ab_pos_mean_vec = zeros(size(phant_id_vec));
phant_sys_ab_pos_std_vec = zeros(size(phant_id_vec));
phant_sys_c_pos_mean_vec = zeros(size(phant_id_vec));
phant_sys_c_pos_std_vec = zeros(size(phant_id_vec));
phant_bar_sep = 0.1;

for i0 = 1:length(phant_id_vec)
	p0 = phant_id_vec(i0);
	[phant_sys_ab_pos, phant_sys_c_pos] = computeAxialStatistics(...
		phant_sys_ab_struct(p0), phant_sys_c_struct(p0), phant_time_mat(i0, :));
	phant_sys_ab_pos_mean_vec(i0) = mean(phant_sys_ab_pos);
	phant_sys_ab_pos_std_vec(i0) = std(phant_sys_ab_pos);
	phant_sys_c_pos_mean_vec(i0) = mean(phant_sys_c_pos);
	phant_sys_c_pos_std_vec(i0) = std(phant_sys_c_pos);
end

vivo_sys_ab_pos_mean_vec = zeros(size(vivo_id_vec));
vivo_sys_ab_pos_std_vec = zeros(size(vivo_id_vec));
vivo_sys_c_pos_mean_vec = zeros(size(vivo_id_vec));
vivo_sys_c_pos_std_vec = zeros(size(vivo_id_vec));
vivo_bar_sep = 0.1;
vivo_sys_id_vec = [1, 1, 2, 2];
vivo_trial_id_vec = [1:4];

for i0 = 1:length(vivo_id_vec)
	v0 = vivo_id_vec(i0);
	[vivo_sys_ab_pos, vivo_sys_c_pos] = computeAxialStatistics(...
		vivo_sys_ab_struct(v0), vivo_sys_c_struct(v0), vivo_time_mat(i0, :), ...
		'ptSrcAxialLimits', vivo_pt_src_ax_limit_mat(i0, :));
	vivo_sys_ab_pos_mean_vec(i0) = mean(vivo_sys_ab_pos);
	vivo_sys_ab_pos_std_vec(i0) = std(vivo_sys_ab_pos);
	vivo_sys_c_pos_mean_vec(i0) = mean(vivo_sys_c_pos);
	vivo_sys_c_pos_std_vec(i0) = std(vivo_sys_c_pos);
end

figure(1);
set(gcf, 'Position', [800, 10, 475, 900]);

tiledlayout(2, 1, 'Padding', 'loose', 'TileSpacing', 'compact');
nexttile(1);

% Tweak color order because WaveSegNet-1 does not show up in this plot.
color_order = get(gca, 'ColorOrder');
color_order = color_order([1, 4, 2, 3, 5, 6, 7], :);
set(gca, 'ColorOrder', color_order, 'NextPlot', 'ReplaceChildren');

errorbar(phant_trial_id_vec - phant_bar_sep, 1e3 .* phant_sys_ab_pos_mean_vec, 1e3 .* phant_sys_ab_pos_std_vec, 'o');
hold on
errorbar(phant_trial_id_vec + phant_bar_sep, 1e3 .* phant_sys_c_pos_mean_vec, 1e3 .* phant_sys_c_pos_std_vec, 'o');
hold off
xlabel({'Trial'; '(a)'});
ylabel('Axial Position [mm]');
title('Phantom');
xlim([0.5, 5.5]);
xticks([1:5]);
% xticklabels({'1', '2', '3', '4', '5'});
ylim([-40, -20]);

nexttile(2);

% Tweak color order to make WaveSegNet-2 purple instead of yellow.
color_order = get(gca, 'ColorOrder');
color_order = color_order([1, 2, 4, 3, 5, 6, 7], :);
set(gca, 'ColorOrder', color_order, 'NextPlot', 'ReplaceChildren');

errorbar(...
	vivo_trial_id_vec(1:2) - vivo_bar_sep, ...
	1e3 .* vivo_sys_ab_pos_mean_vec(1:2), 1e3 .* vivo_sys_ab_pos_std_vec(1:2), 'o', ...
	'DisplayName', 'DetectionNet');
hold on
errorbar(...
	vivo_trial_id_vec(3:4) - vivo_bar_sep, ...
	1e3 .* vivo_sys_ab_pos_mean_vec(3:4), 1e3 .* vivo_sys_ab_pos_std_vec(3:4), 'o', ...
	'DisplayName', 'WaveSegNet-1');
errorbar(...
	vivo_trial_id_vec + vivo_bar_sep, ...
	1e3 .* vivo_sys_c_pos_mean_vec, 1e3 .* vivo_sys_c_pos_std_vec, 'o', 'DisplayName', 'WaveSegNet-2');
hold off
xlabel({'Trial'; '(b)'});
ylabel('Axial Position [mm]');
title('{\itIn Vivo}');
xlim([0.5, 4.5]);
xticks([1:4]);
% xticklabels({'1', '2', '3', '4'});
ylim([-100, -40]);
legend('location', 'southwest');

output_img_file_name = fullfile(output_img_dir_name, 'res_pt_src_ax_dist.png');
fprintf(['Saving file ''', output_img_file_name, '''...\n']);
saveas(gcf, output_img_file_name);
