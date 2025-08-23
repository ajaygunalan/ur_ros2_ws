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
phant_sys_ab_pos_mean_vec = zeros(size(phant_id_vec, 1), 3);
phant_sys_ab_pos_std_vec = zeros(size(phant_id_vec, 1), 3);
phant_sys_c_pos_mean_vec = zeros(size(phant_id_vec, 1), 3);
phant_sys_c_pos_std_vec = zeros(size(phant_id_vec, 1), 3);
phant_bar_sep = 0.1;

for i0 = 1:length(phant_id_vec)
	p0 = phant_id_vec(i0);
	[phant_sys_ab_pos, phant_sys_c_pos] = computePointSourceStatisticsProbe(...
		phant_sys_ab_struct(p0), phant_sys_c_struct(p0), phant_time_mat(i0, :));
	phant_sys_ab_pos_mean_vec(i0, :) = mean(phant_sys_ab_pos, 1);
	phant_sys_ab_pos_std_vec(i0, :) = std(phant_sys_ab_pos, 1);
	phant_sys_c_pos_mean_vec(i0, :) = mean(phant_sys_c_pos, 1);
	phant_sys_c_pos_std_vec(i0, :) = std(phant_sys_c_pos, 1);
end

vivo_sys_ab_pos_mean_vec = zeros(size(vivo_id_vec, 1), 3);
vivo_sys_ab_pos_std_vec = zeros(size(vivo_id_vec, 1), 3);
vivo_sys_c_pos_mean_vec = zeros(size(vivo_id_vec, 1), 3);
vivo_sys_c_pos_std_vec = zeros(size(vivo_id_vec, 1), 3);
vivo_bar_sep = 0.1;
vivo_sys_id_vec = [1, 1, 2, 2];
vivo_trial_id_vec = [1:4];

for i0 = 1:length(vivo_id_vec)
	v0 = vivo_id_vec(i0);
	[vivo_sys_ab_pos, vivo_sys_c_pos] = computePointSourceStatisticsProbe(...
		vivo_sys_ab_struct(v0), vivo_sys_c_struct(v0), vivo_time_mat(i0, :), ...
		'ptSrcAxialLimits', vivo_pt_src_ax_limit_mat(i0, :));
	vivo_sys_ab_pos_mean_vec(i0, :) = mean(vivo_sys_ab_pos, 1);
	vivo_sys_ab_pos_std_vec(i0, :) = std(vivo_sys_ab_pos, 1);
	vivo_sys_c_pos_mean_vec(i0, :) = mean(vivo_sys_c_pos, 1);
	vivo_sys_c_pos_std_vec(i0, :) = std(vivo_sys_c_pos, 1);
end

figure(1);
set(gcf, 'Position', [10, 10, 950, 850]);

tiledlayout(2, 2, 'Padding', 'loose', 'TileSpacing', 'compact');
sub_fig_id_list = {'a', 'b', 'c', 'd', 'e', 'f'};
y_label_list = {'Lateral Position [mm]', 'Elevation Position [mm]', 'Axial Position [mm]'};

y_lim_mat = [...
	-2, 12; ...
	-2, 12; ...
	20, 40; ...
	40, 110];

for i0 = 1:2
	nexttile((2 * i0) - 1);

	% Tweak color order because WaveSegNet-1 does not show up in this plot.
	color_order = get(gca, 'ColorOrder');
	color_order = color_order([1, 4, 2, 3, 5, 6, 7], :);
	set(gca, 'ColorOrder', color_order, 'NextPlot', 'ReplaceChildren');
	errorbar(...
		phant_trial_id_vec - phant_bar_sep, ...
		1e3 .* phant_sys_ab_pos_mean_vec(:, i0 + 1), 1e3 .* phant_sys_ab_pos_std_vec(:, i0 + 1), 'o');
	hold on
	errorbar(...
		phant_trial_id_vec + phant_bar_sep, ...
		1e3 .* phant_sys_c_pos_mean_vec(:, i0 + 1), 1e3 .* phant_sys_c_pos_std_vec(:, i0 + 1), 'o');
	hold off
	xlabel({'Trial'; ['(', sub_fig_id_list{(2 * i0) - 1}, ')']});
	ylabel(y_label_list{i0 + 1});

	if i0 == 1
		title('Phantom');
	end

	xlim([0.5, 5.5]);
	xticks([1:5]);
	ylim(y_lim_mat((2 * i0) - 1, :));

	nexttile(2 * i0);
	color_order = get(gca, 'ColorOrder');
	color_order = color_order([1, 2, 4, 3, 5, 6, 7], :);
	set(gca, 'ColorOrder', color_order, 'NextPlot', 'ReplaceChildren');
	errorbar(...
		vivo_trial_id_vec(1:2) - vivo_bar_sep, ...
		1e3 .* vivo_sys_ab_pos_mean_vec(1:2, i0 + 1), 1e3 .* vivo_sys_ab_pos_std_vec(1:2, i0 + 1), 'o', ...
		'DisplayName', 'DetectionNet');
	hold on
	errorbar(...
		vivo_trial_id_vec(3:4) - vivo_bar_sep, ...
		1e3 .* vivo_sys_ab_pos_mean_vec(3:4, i0 + 1), 1e3 .* vivo_sys_ab_pos_std_vec(3:4, i0 + 1), 'o', ...
		'DisplayName', 'WaveSegNet-1');
	errorbar(...
		vivo_trial_id_vec + vivo_bar_sep, ...
		1e3 .* vivo_sys_c_pos_mean_vec(:, i0 + 1), 1e3 .* vivo_sys_c_pos_std_vec(:, i0 + 1), 'o', ...
		'DisplayName', 'WaveSegNet-2');
	hold off
	xlabel({'Trial'; ['(', sub_fig_id_list{2 * i0}, ')']});
	ylabel(y_label_list{i0 + 1});

	if i0 == 1
		title('{\itIn Vivo}');
	else
		legend('location', 'northwest');
	end

	xlim([0.5, 4.5]);
	xticks([1:4]);
	ylim(y_lim_mat((2 * i0), :));
end

output_img_file_name = fullfile(output_img_dir_name, 'res_pt_src_dist.png');
fprintf(['Saving file ''', output_img_file_name, '''...\n']);
saveas(gcf, output_img_file_name);
