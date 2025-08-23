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

figure(1);
set(gcf, 'Position', [10, 10, 1800, 600]);

tiledlayout(1, 3, 'Padding', 'loose', 'TileSpacing', 'compact');

nexttile(1);
phant_trial_id = 1;
plotCatheterBase(...
	phant_sys_ab_struct(phant_id_vec(phant_trial_id)), phant_sys_c_struct(phant_id_vec(phant_trial_id)), ...
	phant_time_mat(phant_trial_id, :), 'groundTruthDistance', 40e-3, ...
	'saveFile', false);
xlabel({'x [mm]'; '(a)'});
ylabel('z [mm]');
xlim([-20, 22]);
ylim([-40, 5]);
title(sprintf('Phantom Trial %d', phant_trial_id));

nexttile(2);
vivo_trial_id = 2;
plotCatheterBase(...
	vivo_sys_ab_struct(vivo_id_vec(vivo_trial_id)), vivo_sys_c_struct(vivo_id_vec(vivo_trial_id)), ...
	vivo_time_mat(vivo_trial_id, :), ...
	'ptSrcAxialLimits', vivo_pt_src_ax_limit_mat(vivo_trial_id, :), 'groundTruthDistance', 64e-3, ...
	'saveFile', false);
xlabel({'x [mm]'; '(b)'});
ylabel('z [mm]');
xlim([-35, 38]);
ylim([-100, 20]);
title(sprintf('{\\itIn Vivo} Trial %d', vivo_trial_id));
legend('location', 'east');

nexttile(3);
vivo_trial_id = 3;
plotCatheterBase(...
	vivo_sys_ab_struct(vivo_id_vec(vivo_trial_id)), vivo_sys_c_struct(vivo_id_vec(vivo_trial_id)), ...
	vivo_time_mat(vivo_trial_id, :), 'ptSrcAxialLimits', vivo_pt_src_ax_limit_mat(vivo_trial_id, :), ...
	'realTimeSys', 'WaveSegNet-1', 'groundTruthDistance', 38e-3, 'saveFile', false);
xlabel({'x [mm]'; '(c)'});
ylabel('z [mm]');
xlim([-20, 20]);
ylim([-100, 20]);
title(sprintf('{\\itIn Vivo} Trial %d', vivo_trial_id));

output_img_file_name = fullfile(output_img_dir_name, 'res_cath_tracks.png');
fprintf(['Saving file ''', output_img_file_name, '''...\n']);
saveas(gcf, output_img_file_name);
