% This script implements a 1D linear kalman filter.
addpath(genpath(fullfile('..', '..', '..', '..', 'beamforming_code', 'image_quality_metrics', 'utils')));
close all

num_steps = 1000;
time_step = 0.01;

t_vec = zeros(1, num_steps);
u_vec = zeros(2, num_steps);
x_vec = zeros(2, num_steps);
z_vec = zeros(2, num_steps);
mu_vec = zeros(2, num_steps);
sigma_vec = zeros(2, 2, num_steps);

set(...
	0, ...
	'defaultlinelinewidth', 3, ...
	'defaultaxesfontsize', 14, ...
	'defaultlinemarkersize', 7, ...
	'defaulterrorbarlinewidth', 3);

color_mat = getDefaultColorOrder();
color_mat = color_mat.RGB;

figure(1);
set(gcf, 'Position', [800, 10, 1000, 800]);
tiledlayout(2, 2, 'Padding', 'loose', 'TileSpacing', 'compact');

x_lim = [-2, 2];
y_lim = [-2, 2];

for i0 = 1:num_steps
	if i0 == 1
		x_prev = [1, 0]';
		mu_prev = zeros(size(x_vec, 1), 1);
		sigma_prev = eye(size(x_vec, 1));
	else
		x_prev = x_vec(:, i0 - 1);
		mu_prev = mu_vec(:, i0 - 1);
		sigma_prev = squeeze(sigma_vec(:, :, i0 - 1));
	end

	t_vec(i0) = time_step * i0;
	omega_val = pi / 4.0;
	u_vec(1, i0) = -sin(omega_val * t_vec(i0)) * omega_val;
	u_vec(2, i0) = cos(omega_val * t_vec(i0)) * omega_val;

	% Propagate the state.
	prop_state_args = {'B', time_step * eye(size(x_vec, 1)), 'R', 0.001 * eye(size(x_vec, 1))};
	x_vec(:, i0) = linearPropagateState(x_prev, u_vec(:, i0), prop_state_args{:});

	% Measure the state.
	meas_state_args = {'Q', 0.1 * eye(size(x_vec, 1))};
	z_vec(:, i0) = linearMeasureState(x_vec(:, i0), meas_state_args{:});

	% Execute the LKF.
	[mu_vec(:, i0), sigma_vec(:, :, i0)] = linearKalmanFilter(...
		mu_prev, sigma_prev, zeros(size(u_vec, 1), 1), z_vec(:, i0), ...
		'B', time_step * eye(size(x_vec, 1)), ...
		'R', 0.1 * eye(size(x_vec, 1)), ...
		'Q', 0.2 * eye(size(x_vec, 1)));

	if mod(i0, 100) == 0
		plot_linear_kalman_filter_2d;
	end
end

fig_id = 1;
line_width = 1;

for i0 = 1:size(x_vec, 1)
	figure(i0 + fig_id);
	plot(t_vec, x_vec(i0, :), 'LineWidth', line_width, 'DisplayName', 'Position');
	hold on
	plot(t_vec, u_vec(i0, :), 'LineWidth', line_width, 'DisplayName', 'Velocity Command');
	plot(t_vec, z_vec(i0, :), 'LineWidth', line_width, 'DisplayName', 'Measurement');
	plot(t_vec, mu_vec(i0, :), 'LineWidth', line_width, 'DisplayName', 'Estimate');
	hold off
	legend();
	xlabel('Time [s]');
	ylabel('Position [mm]');
end

output_img_dir_name = fullfile('.', 'img');
createMissingDirectories(output_img_dir_name);

figure(1);
output_img_file_name = fullfile(output_img_dir_name, 'lkf_2d_track.png');
fprintf(['Saving file ''', output_img_file_name, '''...\n']);
saveas(gcf, output_img_file_name);

figure(2);
output_img_file_name = fullfile(output_img_dir_name, 'lkf_2d_x.png');
fprintf(['Saving file ''', output_img_file_name, '''...\n']);
saveas(gcf, output_img_file_name);

figure(3);
output_img_file_name = fullfile(output_img_dir_name, 'lkf_2d_y.png');
fprintf(['Saving file ''', output_img_file_name, '''...\n']);
saveas(gcf, output_img_file_name);
