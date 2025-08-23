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

phant_force_mat = [];
fprintf('Phantom:\n');

for i0 = 1:length(phant_id_vec)
	p0 = phant_id_vec(i0);

	netft_id_vec = find(...
		phant_netft_struct(p0).time >= phant_time_mat(i0, 1) & phant_netft_struct(p0).time <= phant_time_mat(i0, 2));

	phant_force_mat = [...
		phant_force_mat; [i0 * ones(length(netft_id_vec), 1), phant_netft_struct(p0).proc_probe(netft_id_vec, 3)]];

	fprintf(...
		'Trial %d: %.2f %%\n', ...
		i0, 100.0 * sum(phant_netft_struct(p0).proc_probe(netft_id_vec, 3) >= 0) / length(netft_id_vec));
end

vivo_force_mat = [];

fprintf('In vivo:\n');

for i0 = 1:length(vivo_id_vec)
	v0 = vivo_id_vec(i0);

	netft_id_vec = find(...
		vivo_netft_struct(v0).time >= vivo_time_mat(i0, 1) & vivo_netft_struct(v0).time <= vivo_time_mat(i0, 2));

	vivo_force_mat = [...
		vivo_force_mat; [i0 * ones(length(netft_id_vec), 1), vivo_netft_struct(v0).proc_probe(netft_id_vec, 3)]];

	fprintf(...
		'Trial %d: %.2f %%\n', ...
		i0, 100.0 * sum(vivo_netft_struct(v0).proc_probe(netft_id_vec, 3) >= 0) / length(netft_id_vec));
end

figure(1);
set(gcf, 'Position', [800, 10, 500, 900]);

tiledlayout(2, 1, 'Padding', 'loose', 'TileSpacing', 'compact');

nexttile(1);
violinplot(-phant_force_mat(:, 2), phant_force_mat(:, 1), 'ShowData', false);
xlabel({'Trial Number'; '(a)'});
ylabel({'{\bfPhantom}'; 'Contact Force [N]'});
xlim([0.5, 5.5]);


nexttile(2);
violinplot(-vivo_force_mat(:, 2), vivo_force_mat(:, 1), 'ShowData', false);
xlabel({'Trial Number'; '(b)'});
ylabel({'{\bf{\itIn Vivo}}'; 'Contact Force [N]'});
xlim([0.5, 4.5]);

output_img_file_name = fullfile(output_img_dir_name, 'res_contact_force.png');
fprintf(['Saving file ''', output_img_file_name, '''...\n']);
saveas(gcf, output_img_file_name);
