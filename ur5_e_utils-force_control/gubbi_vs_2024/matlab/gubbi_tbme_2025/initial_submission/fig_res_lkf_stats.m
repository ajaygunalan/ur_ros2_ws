addpath(genpath(fullfile(...
	'..', '..', '..', '..', '..', '..', '..', '..', 'beamforming_code', 'image_quality_metrics', 'utils')));

load_data;

set(...
	0, ...
	'defaultlinelinewidth', 3, ...
	'defaultaxesfontsize', 20, ...
	'defaultlinemarkersize', 7, ...
	'defaulterrorbarlinewidth', 3);

figure(1);
set(gcf, 'Position', [10, 10, 900, 500]);

tiledlayout(1, 2, 'Padding', 'loose', 'TileSpacing', 'compact');
y_lim = [0, 120];

lkf_str_list = {'Tracking', 'Not Tracking'};

phant_lkf_validity_mat = zeros(length(phant_id_vec), 5);

for i0 = 1:length(phant_id_vec)
	p0 = phant_id_vec(i0);
	phant_lkf_validity_mat(i0, :) = computeLkfStatistics(...
		phant_sys_ab_struct(p0), phant_mtlkf_struct(p0), phant_time_mat(i0, :));
end

phant_lkf_validity_mat = 100.0 .* phant_lkf_validity_mat ./ (sum(phant_lkf_validity_mat, 2) + 1e-12);
phant_lkf_validity_mat = [sum(phant_lkf_validity_mat(:, 1:4), 2), phant_lkf_validity_mat(:, 5)];

vivo_lkf_validity_mat = zeros(length(vivo_id_vec), 5);

for i0 = 1:length(vivo_id_vec)
	v0 = vivo_id_vec(i0);
	vivo_lkf_validity_mat(i0, :) = computeLkfStatistics(...
		vivo_sys_ab_struct(v0), vivo_mtlkf_struct(v0), vivo_time_mat(i0, :));
end

vivo_lkf_validity_mat = 100.0 .* vivo_lkf_validity_mat ./ (sum(vivo_lkf_validity_mat, 2) + 1e-12);
vivo_lkf_validity_mat = [sum(vivo_lkf_validity_mat(:, 1:4), 2), vivo_lkf_validity_mat(:, 5)];

nexttile(1);
phant_h = bar(phant_lkf_validity_mat);

for i0 = 1:length(phant_h)
	phant_h(i0).DisplayName = lkf_str_list{i0};
end

color_order = get(gca, 'ColorOrder');
set(gca, 'ColorOrder', color_order, 'NextPlot', 'ReplaceChildren');

xlabel({'Trial Number'; '(a)'});
ylim(y_lim);
yticks([0:20:100]);
ylabel('MTLKF Tracking Rate [%]');
title('Phantom');

nexttile(2);
vivo_h = bar(vivo_lkf_validity_mat);

for i0 = 1:length(vivo_h)
	vivo_h(i0).DisplayName = lkf_str_list{i0};
end

color_order = get(gca, 'ColorOrder');
set(gca, 'ColorOrder', color_order, 'NextPlot', 'ReplaceChildren');

xlabel({'Trial Number'; '(b)'});
ylim(y_lim);
yticklabels({});
title('{\itIn Vivo}');
% legend('location', 'northeastoutside');
legend('location', 'northeast');

output_img_dir_name = fullfile('.', 'img');
createMissingDirectories(output_img_dir_name);
output_img_file_name = fullfile(output_img_dir_name, 'res_lkf_stats.png');
fprintf(['Saving file ''', output_img_file_name, '''...\n']);
saveas(gcf, output_img_file_name);
