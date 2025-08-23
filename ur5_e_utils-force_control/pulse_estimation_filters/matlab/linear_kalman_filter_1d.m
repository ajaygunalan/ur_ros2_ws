% This script implements a 1D linear kalman filter.
addpath(genpath(fullfile('..', '..', '..', '..', 'beamforming_code', 'image_quality_metrics', 'utils')));
close all

num_steps = 1000;
time_step = 0.01;

t_vec = zeros(1, num_steps);
u_vec = zeros(1, num_steps);
x_vec = zeros(1, num_steps);
z_vec = zeros(1, num_steps);
mu_vec = zeros(1, num_steps);
sigma_vec = zeros(1, 1, num_steps);

for i0 = 1:num_steps
	if i0 == 1
		x_prev = zeros(size(x_vec, 1), 1);
		mu_prev = zeros(size(x_vec, 1), 1);
		sigma_prev = eye(size(x_vec, 1));
	else
		x_prev = x_vec(:, i0 - 1);
		mu_prev = mu_vec(:, i0 - 1);
		sigma_prev = squeeze(sigma_vec(:, :, i0 - 1));
	end

	t_vec(i0) = time_step * i0;
	u_vec(:, i0) = sin(pi * t_vec(i0) / 4.0);

	prop_state_args = {'B', time_step};
	x_vec(:, i0) = linearPropagateState(x_prev, u_vec(:, i0), prop_state_args{:});

	meas_state_args = {};
	z_vec(:, i0) = linearMeasureState(x_vec(:, i0), meas_state_args{:});
	[mu_vec(:, i0), sigma_vec(:, :, i0)] = linearKalmanFilter(...
		mu_prev, sigma_prev, zeros(size(u_vec, 1), 1), z_vec(:, i0), prop_state_args{:}, meas_state_args{:});
end

plot(t_vec, x_vec, 'DisplayName', 'Position');
hold on
plot(t_vec, u_vec, 'DisplayName', 'Velocity Command');
plot(t_vec, z_vec, 'DisplayName', 'Measurement');
plot(t_vec, mu_vec, 'DisplayName', 'Estimate');
hold off
legend();
xlabel('Time [s]');
ylabel('Position [mm]');

output_img_dir_name = fullfile('.', 'img');
createMissingDirectories(output_img_dir_name);
output_img_file_name = fullfile(output_img_dir_name, 'lkf_1d.png');
fprintf(['Saving file ''', output_img_file_name, '''...\n']);
saveas(gcf, output_img_file_name);
