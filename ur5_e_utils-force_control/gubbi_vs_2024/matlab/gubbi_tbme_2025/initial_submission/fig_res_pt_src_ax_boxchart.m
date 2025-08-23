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

phant_pos = [];
phant_box_sep = 0.0;
phant_box_width = 0.5;
phant_box_scaling = 2.0;

for i0 = 1:length(phant_id_vec)
	p0 = phant_id_vec(i0);
	[phant_sys_ab_pos, phant_sys_c_pos] = computeAxialStatistics(...
		phant_sys_ab_struct(p0), phant_sys_c_struct(p0), phant_time_mat(i0, :));
	phant_pos = [...
		phant_pos; ...
		[phant_sys_ab_pos, (i0 - phant_box_sep) .* ones(size(phant_sys_ab_pos)), ones(size(phant_sys_ab_pos))]; ...
		[phant_sys_c_pos, (i0 + phant_box_sep) .* ones(size(phant_sys_c_pos)), 3 .* ones(size(phant_sys_c_pos))]];
end

vivo_pos = [];
vivo_box_sep = 0.0;
vivo_box_width = 0.5;
vivo_box_scaling = 2.5;
vivo_sys_id_vec = [1, 1, 2, 2];

for i0 = 1:length(vivo_id_vec)
	v0 = vivo_id_vec(i0);
	[vivo_sys_ab_pos, vivo_sys_c_pos] = computeAxialStatistics(...
		vivo_sys_ab_struct(v0), vivo_sys_c_struct(v0), vivo_time_mat(i0, :), ...
		'ptSrcAxialLimits', vivo_pt_src_ax_limit_mat(i0, :));
	vivo_pos = [...
		vivo_pos; ...
		[vivo_sys_ab_pos, (i0 - vivo_box_sep) .* ones(size(vivo_sys_ab_pos)), vivo_sys_id_vec(i0) .* ones(size(vivo_sys_ab_pos))]; ...
		[vivo_sys_c_pos, (i0 + vivo_box_sep) .* ones(size(vivo_sys_c_pos)), 3 .* ones(size(vivo_sys_c_pos))]];
end

figure(1);
set(gcf, 'Position', [800, 10, 650, 900]);

tiledlayout(2, 1, 'Padding', 'loose', 'TileSpacing', 'compact');
nexttile(1);
boxchart(...
	phant_box_scaling .* phant_pos(:, 2), 1e3 .* phant_pos(:, 1), ...
	'BoxWidth', phant_box_width, 'MarkerStyle', '.', 'GroupByColor', phant_pos(:, 3));
xlabel({'Trial'; '(a)'});
ylabel('Axial Position [mm]');
title('Phantom');
xlim(phant_box_scaling .* [0.5, 5.5]);
xticks(phant_box_scaling .* [1:5]);
xticklabels({'1', '2', '3', '4', '5'});

nexttile(2);
boxchart(...
	vivo_box_scaling .* vivo_pos(:, 2), 1e3 .* vivo_pos(:, 1), ...
	'BoxWidth', vivo_box_width, 'MarkerStyle', '.', 'GroupByColor', vivo_pos(:, 3));
xlabel({'Trial'; '(b)'});
ylabel('Axial Position [mm]');
title('{\itIn Vivo}');
xlim(vivo_box_scaling .* [0.5, 4.5]);
xticks(vivo_box_scaling .* [1:5]);
xticklabels({'1', '2', '3', '4'});

output_img_file_name = fullfile(output_img_dir_name, 'res_pt_src_ax_boxchart.png');
fprintf(['Saving file ''', output_img_file_name, '''...\n']);
saveas(gcf, output_img_file_name);
