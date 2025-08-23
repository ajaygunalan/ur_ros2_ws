addpath(genpath(fullfile(...
	'..', '..', '..', '..', '..', '..', '..', '..', 'beamforming_code', 'image_quality_metrics', 'utils')));

load_data;

close all
set(...
	0, ...
	'defaultlinelinewidth', 3, ...
	'defaultaxesfontsize', 18, ...
	'defaultlinemarkersize', 7, ...
	'defaulterrorbarlinewidth', 3);

output_img_dir_name = fullfile('.', 'img');
createMissingDirectories(output_img_dir_name);

fluoro_img_file_name = fullfile('.', 'dat', 'fluoro_trial_02.png');
fprintf(['Reading file ''', fluoro_img_file_name, '''...\n']);
fluoro_img = imread(fluoro_img_file_name);
fluoro_img = fluoro_img(1401:3400, 500:2800, :);
fluoro_img = fluoro_img(1:1600, :, :);

fluoro_x_dim = [37, 2.3e-3];
fluoro_y_dim = [710 - 90, 64e-3];

fluoro_x_axis = (fluoro_x_dim(2) / fluoro_x_dim(1)) .* [1:size(fluoro_img, 2)];
fluoro_y_axis = (fluoro_y_dim(2) / fluoro_y_dim(1)) .* [1:size(fluoro_img, 1)];

gen_cath_track_csv;
cath_track = csvread(fullfile('.', 'dat', 'cath_track.csv'));

figure(1);
% set(gcf, 'Position', [10, 10, 1800, 600]);

% tiledlayout(1, 3, 'Padding', 'loose', 'TileSpacing', 'compact');

vivo_trial_id = 2;
sys_c_pos_shifted = plotFluoroBase(...
	vivo_sys_ab_struct(vivo_id_vec(vivo_trial_id)), vivo_sys_c_struct(vivo_id_vec(vivo_trial_id)), ...
	vivo_time_mat(vivo_trial_id, :), ...
	'ptSrcAxialLimits', vivo_pt_src_ax_limit_mat(vivo_trial_id, :), 'groundTruthDistance', 64e-3, ...
	'saveFile', false);
xlabel('x [mm]');
ylabel('z [mm]');
% xlim([-35, 38]);
% ylim([-100, 20]);
title(sprintf('{\\itIn Vivo} Trial %d', vivo_trial_id));
% legend('location', 'east');

cath_track_x_full = [90:1930]';
cath_track_y_full = spline(low_mat(:, 1), low_mat(:, 2), cath_track_x_full);

% z = find(sys_c_pos_shifted(:, 1) > -0.003);
% sys_c_pos_shifted = sys_c_pos_shifted(z, 1:3);
sys_c_pos_shifted = sys_c_pos_shifted(:, 1:3);
% sys_c_pos_shifted(:, 3) = 0.0;
cath_track_x_trial = linspace(90, 710, size(sys_c_pos_shifted, 1))';
cath_track_y_trial = spline(low_mat(:, 1), low_mat(:, 2), cath_track_x_trial);

cath_track_m = flip([fluoro_x_axis(round(cath_track_y_trial))', fluoro_y_axis(round(cath_track_x_trial))', zeros(size(cath_track_y_trial, 1), 1)]);

[s, r, t] = absoluteOrientationQuaternion([sys_c_pos_shifted(1:10:end, 1:2), zeros(size(sys_c_pos_shifted(1:10:end, :), 1), 1)]', cath_track_m(1:10:end, :)', 0);
t(2) = t(2) + 5e-3;

sys_c_cath_track_m = ((s * r * sys_c_pos_shifted') + t)';
sys_c_cath_track_m(:, 3) = sys_c_cath_track_m(:, 3) - mean(sys_c_cath_track_m(:, 3));
sys_c_cath_track = zeros(size(sys_c_cath_track_m));
[~, sys_c_cath_track(:, 1)] = min(abs(fluoro_x_axis - sys_c_cath_track_m(:, 1)), [], 2);
[~, sys_c_cath_track(:, 2)] = min(abs(fluoro_y_axis - sys_c_cath_track_m(:, 2)), [], 2);

% fluoro_img = insertShape(...
% 	fluoro_img, 'line', [cath_track_y_trial, cath_track_x_trial], 'LineWidth', 20, 'ShapeColor', 'red');
% fluoro_img = insertShape(...
% 	fluoro_img, 'line', [cath_track_y_full, cath_track_x_full], 'LineWidth', 10, 'ShapeColor', 'red');
fluoro_img = insertShape(...
	fluoro_img, 'circle', [sys_c_cath_track(:, 1:2), 10 .* ones(size(sys_c_cath_track, 1), 1)], ...
	'ShapeColor', 'yellow');

fluoro_img = fluoro_img(1:1400, :, :);
fluoro_img = fluoro_img(:, 101:end, :);
fluoro_img = fluoro_img(:, 1:1935, :);

% Fluoro axes - 90-710 vertical corresponds to 64 mm
fluoro_z_axis = (1:size(fluoro_img, 1)) .* (64 / (710-90));
fluoro_z_axis = fluoro_z_axis - mean(fluoro_z_axis);

fluoro_x_axis = (1:size(fluoro_img, 2)) .* (64 / (710-90));
fluoro_x_axis = fluoro_x_axis - mean(fluoro_x_axis);

figure(2);
imagesc(fluoro_x_axis, fluoro_z_axis, fluoro_img);
shading interp
xlabel('y [mm]');
ylabel('x [mm]');

output_img_file_name = fullfile(output_img_dir_name, 'res_fluoro_superpose_imagesc.png');
fprintf(['Saving file ''', output_img_file_name, '''...\n']);
saveas(gcf, output_img_file_name);

output_img_file_name = fullfile(output_img_dir_name, 'res_fluoro_superpose_imwrite.png');
fprintf(['Saving file ''', output_img_file_name, '''...\n']);
imwrite(fluoro_img, output_img_file_name);

min_dist_vec = Inf(size(sys_c_cath_track_m, 1), 1);

for i0 = 1:size(sys_c_cath_track_m, 1)
	min_dist_vec(i0) = min(sqrt(sum((sys_c_cath_track_m(i0, 1:2) - cath_track_m(:, 1:2)) .^ 2, 2)));
end

mean_dist_err = mean(min_dist_vec);
rmse_dist_err = sqrt(mean(min_dist_vec .^ 2));
median_dist_err = median(min_dist_vec);
max_dist_err = max(min_dist_vec);
min_dist_err = min(min_dist_vec);
fprintf('Mean Euclidean distance error = %.2f mm\n', 1e3 * mean_dist_err);
fprintf('RMSE Euclidean distance error = %.2f mm\n', 1e3 * rmse_dist_err);
fprintf('Median Euclidean distance error = %.2f mm\n', 1e3 * median_dist_err);
fprintf('Range Euclidean distance error = %.2f-%.2f mm\n', 1e3 * min_dist_err, 1e3 * max_dist_err);

% imshow(fluoro_img);
% axis image
