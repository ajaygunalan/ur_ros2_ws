addpath(genpath(fullfile(...
	'..', '..', '..', '..', '..', '..', '..', '..', 'beamforming_code', 'image_quality_metrics', 'utils')));

load_data;

set(...
	0, ...
	'defaultlinelinewidth', 3, ...
	'defaultaxesfontsize', 20, ...
	'defaultlinemarkersize', 7, ...
	'defaulterrorbarlinewidth', 3);


output_img_dir_name = fullfile('.', 'img');
createMissingDirectories(output_img_dir_name);

y_lim = [-2.15, 4.7];

phant_force_mat = [];
fprintf('Phantom:\n');

for i0 = 1:length(phant_id_vec)
	p0 = phant_id_vec(i0);

	netft_id_vec = find(...
		phant_netft_struct(p0).time >= phant_time_mat(i0, 1) & phant_netft_struct(p0).time <= phant_time_mat(i0, 2));

	phant_force_vec = -sort(phant_netft_struct(p0).proc_probe(netft_id_vec, 3));
	phant_median = median(phant_force_vec);
	phant_iqr = iqr(phant_force_vec);
	phant_outliers = sum(abs(phant_median - phant_force_vec) > 1.5 * phant_iqr);

	fprintf(...
		'Trial %d:\n\tNon-contact %.2f %%\n\tMedian %.2f N\n\tInter-quartile %.2f N\n\tOutliers %.2f%%\n', ...
		i0, 100.0 * sum(phant_force_vec <= 0) / length(netft_id_vec), ...
		phant_median, phant_iqr, 100.0 * phant_outliers / length(netft_id_vec));

	phant_force_mat = [phant_force_mat; [i0 * ones(length(netft_id_vec), 1), phant_force_vec]];
end

vivo_force_mat = [];

fprintf('In vivo:\n');

for i0 = 1:length(vivo_id_vec)
	v0 = vivo_id_vec(i0);

	netft_id_vec = find(...
		vivo_netft_struct(v0).time >= vivo_time_mat(i0, 1) & vivo_netft_struct(v0).time <= vivo_time_mat(i0, 2));
	vivo_force_vec = -sort(vivo_netft_struct(v0).proc_probe(netft_id_vec, 3));
	vivo_median = median(vivo_force_vec);
	vivo_iqr = iqr(vivo_force_vec);
	vivo_outliers = sum(abs(vivo_median - vivo_force_vec) > 1.5 * vivo_iqr);

	fprintf(...
		'Trial %d:\n\tNon-contact %.2f %%\n\tMedian %.2f N\n\tInter-quartile %.2f N\n\tOutliers %.2f%%\n', ...
		i0, 100.0 * sum(vivo_force_vec <= 0) / length(netft_id_vec), ...
		vivo_median, vivo_iqr, 100.0 * vivo_outliers / length(netft_id_vec));

	vivo_force_mat = [vivo_force_mat; [i0 * ones(length(netft_id_vec), 1), vivo_force_vec]];
end

figure(1);
set(gcf, 'Position', [800, 10, 900, 500]);

tiledlayout(1, 2, 'Padding', 'loose', 'TileSpacing', 'compact');

nexttile(1);
boxplot(phant_force_mat(:, 2), phant_force_mat(:, 1), 'Symbol', 'r.', 'Jitter', 0.2);
ylim(y_lim);
xlabel({'Trial Number'; '(a)'});
ylabel('Contact Force [N]');
title('Phantom');

nexttile(2);
boxplot(vivo_force_mat(:, 2), vivo_force_mat(:, 1), 'Symbol', 'r.', 'Jitter', 0.2);
ylim(y_lim);
xlabel({'Trial Number'; '(b)'});
yticklabels({});
title('{\itIn Vivo}');

output_img_file_name = fullfile(output_img_dir_name, 'res_contact_force_box.png');
fprintf(['Saving file ''', output_img_file_name, '''...\n']);
saveas(gcf, output_img_file_name);
