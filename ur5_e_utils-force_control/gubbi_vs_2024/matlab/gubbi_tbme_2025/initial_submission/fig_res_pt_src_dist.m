addpath(genpath(fullfile(...
	'..', '..', '..', '..', '..', '..', '..', '..', 'beamforming_code', 'image_quality_metrics', 'utils')));

load_data;

y_lim_elev = [-0.5, 10.5];
y_lim_ax = [10, 110];

set(...
	0, ...
	'defaultlinelinewidth', 3, ...
	'defaultaxesfontsize', 18, ...
	'defaultlinemarkersize', 7, ...
	'defaulterrorbarlinewidth', 3);

output_img_dir_name = fullfile('.', 'img');
createMissingDirectories(output_img_dir_name);

phant_trial_id_vec = [1:5];
phant_sys_ab_pos_mat = [];
phant_sys_c_pos_mat = [];

for i0 = 1:length(phant_id_vec)
	p0 = phant_id_vec(i0);
	[phant_sys_ab_pos, phant_sys_c_pos] = computePointSourceStatisticsProbe(...
		phant_sys_ab_struct(p0), phant_sys_c_struct(p0), phant_time_mat(i0, :));
	phant_sys_ab_pos_mat = [...
		phant_sys_ab_pos_mat; ...
		i0 * ones(size(phant_sys_ab_pos, 1), 1), phant_sys_ab_pos, ones(size(phant_sys_ab_pos, 1), 1)];
	phant_sys_c_pos_mat = [...
		phant_sys_c_pos_mat; ...
		i0 * ones(size(phant_sys_c_pos, 1), 1), phant_sys_c_pos, 3 .* ones(size(phant_sys_c_pos, 1), 1)];

	phant_sys_ab_median = median(phant_sys_ab_pos, 1);
	[~, phant_sys_ab_iqr] = iqr(phant_sys_ab_pos, 1);
	phant_sys_c_median = median(phant_sys_c_pos, 1);
	[~, phant_sys_c_iqr] = iqr(phant_sys_c_pos, 1);

	fprintf('Trial %d:\n', i0);
	fprintf('\tSystem A/B:\n');
	fprintf(...
		'\t\tElevation: [%.1f, %.1f, %.1f] mm\n', ...
		1e3 * phant_sys_ab_iqr(1, 2), 1e3 * phant_sys_ab_median(2), 1e3 * phant_sys_ab_iqr(2, 2));
	fprintf('\t\t    Axial: [%.1f, %.1f, %.1f] mm\n', ...
		1e3 * phant_sys_ab_iqr(1, 3), 1e3 * phant_sys_ab_median(3), 1e3 * phant_sys_ab_iqr(2, 3));
	fprintf('\tSystem C:\n');
	fprintf(...
		'\t\tElevation: [%.1f, %.1f, %.1f] mm\n', ...
		1e3 * phant_sys_c_iqr(1, 2), 1e3 * phant_sys_c_median(2), 1e3 * phant_sys_c_iqr(2, 2));
	fprintf('\t\t    Axial: [%.1f, %.1f, %.1f] mm\n', ...
		1e3 * phant_sys_c_iqr(1, 3), 1e3 * phant_sys_c_median(3), 1e3 * phant_sys_c_iqr(2, 3));
end

vivo_trial_id_vec = [1:4];
vivo_sys_ab_pos_mat = [];
vivo_sys_c_pos_mat = [];

for i0 = 1:length(vivo_id_vec)
	v0 = vivo_id_vec(i0);
	[vivo_sys_ab_pos, vivo_sys_c_pos] = computePointSourceStatisticsProbe(...
		vivo_sys_ab_struct(v0), vivo_sys_c_struct(v0), vivo_time_mat(i0, :), ...
		'ptSrcAxialLimits', vivo_pt_src_ax_limit_mat(i0, :));

	if i0 <= 2
		vivo_sys_ab_id = 1;
	else
		vivo_sys_ab_id = 2;
	end

	vivo_sys_ab_pos_mat = [...
		vivo_sys_ab_pos_mat; ...
		i0 * ones(size(vivo_sys_ab_pos, 1), 1), vivo_sys_ab_pos, vivo_sys_ab_id .* ones(size(vivo_sys_ab_pos, 1), 1)];
	vivo_sys_c_pos_mat = [...
		vivo_sys_c_pos_mat; ...
		i0 * ones(size(vivo_sys_c_pos, 1), 1), vivo_sys_c_pos, 3 .* ones(size(vivo_sys_c_pos, 1), 1)];

	vivo_sys_ab_median = median(vivo_sys_ab_pos, 1);
	[~, vivo_sys_ab_iqr] = iqr(vivo_sys_ab_pos, 1);
	vivo_sys_c_median = median(vivo_sys_c_pos, 1);
	[~, vivo_sys_c_iqr] = iqr(vivo_sys_c_pos, 1);

	fprintf('Trial %d:\n', i0);
	fprintf('\tSystem A/B:\n');
	fprintf(...
		'\t\tElevation: [%.1f, %.1f, %.1f] mm\n', ...
		1e3 * vivo_sys_ab_iqr(1, 2), 1e3 * vivo_sys_ab_median(2), 1e3 * vivo_sys_ab_iqr(2, 2));
	fprintf('\t\t    Axial: [%.1f, %.1f, %.1f] mm\n', ...
		1e3 * vivo_sys_ab_iqr(1, 3), 1e3 * vivo_sys_ab_median(3), 1e3 * vivo_sys_ab_iqr(2, 3));
	fprintf('\tSystem C:\n');
	fprintf(...
		'\t\tElevation: [%.1f, %.1f, %.1f] mm\n', ...
		1e3 * vivo_sys_c_iqr(1, 2), 1e3 * vivo_sys_c_median(2), 1e3 * vivo_sys_c_iqr(2, 2));
	fprintf('\t\t    Axial: [%.1f, %.1f, %.1f] mm\n', ...
		1e3 * vivo_sys_c_iqr(1, 3), 1e3 * vivo_sys_c_median(3), 1e3 * vivo_sys_c_iqr(2, 3));
end

figure(1);
set(gcf, 'Position', [10, 10, 950, 850]);

tiledlayout(2, 2, 'Padding', 'tight', 'TileSpacing', 'compact');
sub_fig_id_list = {'a', 'b', 'c', 'd', 'e', 'f'};
y_label_list = {'Lateral Position [mm]', 'Elevation Position [mm]', 'Axial Position [mm]'};

y_lim_mat = [...
	-2, 12; ...
	-2, 12; ...
	20, 40; ...
	40, 110];

notch_req = 'on';
marker_style = '.';

nexttile(1);
color_order = get(gca, 'ColorOrder');
phant_elev_scale = 1.25;
phant_elev_h = boxchart(...
	phant_elev_scale .* [phant_sys_ab_pos_mat(:, 1); phant_sys_c_pos_mat(:, 1)], ...
	1e3 .* [phant_sys_ab_pos_mat(:, 3); phant_sys_c_pos_mat(:, 3)], ...
	'GroupByColor', [phant_sys_ab_pos_mat(:, 5); phant_sys_c_pos_mat(:, 5)], ...
	'Notch', notch_req, 'MarkerStyle', marker_style);
phant_elev_h(2).BoxEdgeColor = color_order(3, :);
phant_elev_h(2).BoxFaceColor = color_order(3, :);
phant_elev_h(2).BoxMedianLineColor = color_order(3, :);
phant_elev_h(2).MarkerColor = color_order(3, :);
xticks(phant_elev_scale .* [1:5]);
xticklabels({'1', '2', '3', '4', '5'});
ylim(y_lim_elev);
xlabel({'Trial Number'; '(a)'});
ylabel({'Elevation Position'; '[mm]'});
title('Phantom');

nexttile(2);
vivo_elev_scale = 1.25;
vivo_sys_a_id = (vivo_sys_ab_pos_mat(:, 1) <= 2);
vivo_sys_ac_id = (vivo_sys_c_pos_mat(:, 1) <= 2);
vivo_sys_a_elev_h = boxchart(...
	vivo_elev_scale .* [vivo_sys_ab_pos_mat(vivo_sys_a_id, 1); vivo_sys_c_pos_mat(vivo_sys_ac_id, 1)], ...
	1e3 .* [vivo_sys_ab_pos_mat(vivo_sys_a_id, 3); vivo_sys_c_pos_mat(vivo_sys_ac_id, 3)], ...
	'GroupByColor', [vivo_sys_ab_pos_mat(vivo_sys_a_id, 5); vivo_sys_c_pos_mat(vivo_sys_ac_id, 5)], ...
	'Notch', notch_req, 'MarkerStyle', marker_style);
vivo_sys_a_elev_h(2).BoxEdgeColor = color_order(3, :);
vivo_sys_a_elev_h(2).BoxFaceColor = color_order(3, :);
vivo_sys_a_elev_h(2).BoxMedianLineColor = color_order(3, :);
vivo_sys_a_elev_h(2).MarkerColor = color_order(3, :);
hold on
vivo_sys_b_elev_h = boxchart(...
	vivo_elev_scale .* [vivo_sys_ab_pos_mat(~vivo_sys_a_id, 1); vivo_sys_c_pos_mat(~vivo_sys_ac_id, 1)], ...
	1e3 .* [vivo_sys_ab_pos_mat(~vivo_sys_a_id, 3); vivo_sys_c_pos_mat(~vivo_sys_ac_id, 3)], ...
	'GroupByColor', [vivo_sys_ab_pos_mat(~vivo_sys_a_id, 5); vivo_sys_c_pos_mat(~vivo_sys_ac_id, 5)], ...
	'Notch', notch_req, 'MarkerStyle', marker_style);
vivo_sys_b_elev_h(1).BoxEdgeColor = color_order(2, :);
vivo_sys_b_elev_h(1).BoxFaceColor = color_order(2, :);
vivo_sys_b_elev_h(1).BoxMedianLineColor = color_order(2, :);
vivo_sys_b_elev_h(1).MarkerColor = color_order(2, :);
vivo_sys_b_elev_h(2).BoxEdgeColor = color_order(3, :);
vivo_sys_b_elev_h(2).BoxFaceColor = color_order(3, :);
vivo_sys_b_elev_h(2).BoxMedianLineColor = color_order(3, :);
vivo_sys_b_elev_h(2).MarkerColor = color_order(3, :);
xticks(vivo_elev_scale .* [1:4]);
xticklabels({'1', '2', '3', '4'});
yticklabels({});
ylim(y_lim_elev);
xlabel({'Trial Number'; '(b)'});
title('{\itIn Vivo}');

nexttile(3);
phant_ax_scale = 1.25;
phant_ax_h = boxchart(...
	phant_ax_scale .* [phant_sys_ab_pos_mat(:, 1); phant_sys_c_pos_mat(:, 1)], ...
	1e3 .* [phant_sys_ab_pos_mat(:, 4); phant_sys_c_pos_mat(:, 4)], ...
	'GroupByColor', [phant_sys_ab_pos_mat(:, 5); phant_sys_c_pos_mat(:, 5)], ...
	'Notch', notch_req, 'MarkerStyle', marker_style, 'HandleVisibility', 'off');
phant_ax_h(2).BoxEdgeColor = color_order(3, :);
phant_ax_h(2).BoxFaceColor = color_order(3, :);
phant_ax_h(2).BoxMedianLineColor = color_order(3, :);
phant_ax_h(2).MarkerColor = color_order(3, :);
hold on
x_lim = xlim();
legend_h = boxchart(...
	phant_ax_scale .* [7, 7, 7]', 1e3 .* [1, 1, 1]', 'GroupByColor', [1, 2, 3]');
legend_h(1).DisplayName = 'DetectionNet';
legend_h(2).DisplayName = 'WaveSegNet-1';
legend_h(3).DisplayName = 'WaveSegNet-2';

for i0 = 1:3
	legend_h(i0).BoxEdgeColor = color_order(i0, :);
	legend_h(i0).BoxFaceColor = color_order(i0, :);
	legend_h(i0).BoxMedianLineColor = color_order(i0, :);
	legend_h(i0).MarkerColor = color_order(i0, :);
end

hold off
xlim(x_lim);

xticks(phant_ax_scale .* [1:5]);
xticklabels({'1', '2', '3', '4', '5'});
ylim(y_lim_ax);
xlabel({'Trial Number'; '(c)'});
ylabel('Axial Position [mm]');
legend('location', 'northwest');

nexttile(4);
vivo_ax_scale = 1.25;
vivo_sys_a_id = (vivo_sys_ab_pos_mat(:, 1) <= 2);
vivo_sys_ac_id = (vivo_sys_c_pos_mat(:, 1) <= 2);
vivo_sys_a_ax_h = boxchart(...
	vivo_ax_scale .* [vivo_sys_ab_pos_mat(vivo_sys_a_id, 1); vivo_sys_c_pos_mat(vivo_sys_ac_id, 1)], ...
	1e3 .* [vivo_sys_ab_pos_mat(vivo_sys_a_id, 4); vivo_sys_c_pos_mat(vivo_sys_ac_id, 4)], ...
	'GroupByColor', [vivo_sys_ab_pos_mat(vivo_sys_a_id, 5); vivo_sys_c_pos_mat(vivo_sys_ac_id, 5)], ...
	'Notch', notch_req, 'MarkerStyle', marker_style);
vivo_sys_a_ax_h(2).BoxEdgeColor = color_order(3, :);
vivo_sys_a_ax_h(2).BoxFaceColor = color_order(3, :);
vivo_sys_a_ax_h(2).BoxMedianLineColor = color_order(3, :);
vivo_sys_a_ax_h(2).MarkerColor = color_order(3, :);
hold on
vivo_sys_b_ax_h = boxchart(...
	vivo_ax_scale .* [vivo_sys_ab_pos_mat(~vivo_sys_a_id, 1); vivo_sys_c_pos_mat(~vivo_sys_ac_id, 1)], ...
	1e3 .* [vivo_sys_ab_pos_mat(~vivo_sys_a_id, 4); vivo_sys_c_pos_mat(~vivo_sys_ac_id, 4)], ...
	'GroupByColor', [vivo_sys_ab_pos_mat(~vivo_sys_a_id, 5); vivo_sys_c_pos_mat(~vivo_sys_ac_id, 5)], ...
	'Notch', notch_req, 'MarkerStyle', marker_style);
vivo_sys_b_ax_h(1).BoxEdgeColor = color_order(2, :);
vivo_sys_b_ax_h(1).BoxFaceColor = color_order(2, :);
vivo_sys_b_ax_h(1).BoxMedianLineColor = color_order(2, :);
vivo_sys_b_ax_h(1).MarkerColor = color_order(2, :);
vivo_sys_b_ax_h(2).BoxEdgeColor = color_order(3, :);
vivo_sys_b_ax_h(2).BoxFaceColor = color_order(3, :);
vivo_sys_b_ax_h(2).BoxMedianLineColor = color_order(3, :);
vivo_sys_b_ax_h(2).MarkerColor = color_order(3, :);
xticks(vivo_ax_scale .* [1:4]);
xticklabels({'1', '2', '3', '4'});
yticklabels({});
ylim(y_lim_ax);
xlabel({'Trial Number'; '(d)'});

output_img_file_name = fullfile(output_img_dir_name, 'res_pt_src_dist.png');
fprintf(['Saving file ''', output_img_file_name, '''...\n']);
saveas(gcf, output_img_file_name);
