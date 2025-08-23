x_actual = zeros(3, 1);
v_actual = zeros(3, 1);
v_actual(1) = 1e-3;

t_samp = 100e-3;

x_est_curr = 1e-3 .* [0, 0, 0, 0, 0, 0]';
p_curr = zeros(6, 6);

proc_noise_cov_mat = 1e-6 .* eye(6);
a_mat = [...
	1, 0, 0, t_samp, 0, 0; ...
	0, 1, 0, 0, t_samp, 0; ...
	0, 0, 1, 0, 0, t_samp; ...
	0, 0, 0, 1, 0, 0; ...
	0, 0, 0, 0, 1, 0; ...
	0, 0, 0, 0, 0, 1];

h_mat = [...
	1, 0, 0, 0, 0, 0; ...
	0, 1, 0, 0, 0, 0; ...
	0, 0, 1, 0, 0, 0];
meas_noise_cov_mat = 4e-6 .* eye(3);

n = 128;
gt_vec = zeros(6, n);
dt_vec = zeros(6, n + 1);
dt_vec(:, 1) = x_est_curr;

for i0 = 1:n
	% Ground truth, assuming perfect model.
	x_actual = x_actual + (t_samp .* v_actual) + (1e-3 .* randn(3, 1));
	v_actual = v_actual + (1e-3 .* randn(3, 1));
	% x_actual = x_actual + (t_samp .* v_actual);
	z_actual = x_actual + (2e-3 .* randn(3, 1));
	gt_vec(:, i0) = [x_actual; v_actual];

	% Linear Kalman Filter
	x_est_prev = x_est_curr;
	x_pred = a_mat * x_est_curr;
	p_pred = (a_mat * p_curr * a_mat') + proc_noise_cov_mat;
	k_gain = p_pred * h_mat' * inv((h_mat * p_pred * h_mat') + meas_noise_cov_mat);
	x_est_curr = x_pred + (k_gain * (z_actual - (h_mat * x_pred)));
	p_curr = p_pred - (k_gain * h_mat * p_pred);

	% Manually update velocity because it is not observed in the measurements.
	dt_vec(:, i0 + 1) = x_est_curr;
end

y_label_list = {'x [mm]', 'y [mm]', 'z [mm]', 'v_x [mm/s]', 'v_y [mm/s]', 'v_z [mm/s]'};
title_list = {...
	'Lateral Position', 'Elevation Position', 'Axial Position', ...
	'Lateral Velocity', 'Elevation Velocity', 'Axial Velocity'};


figure();
tiledlayout(2, 3);

for i0 = 1:6
	nexttile(i0);
	plot(t_samp .* 1:n, 1e3 .* gt_vec(i0, :), 'DisplayName', 'Ground Truth');
	hold on
	plot(t_samp .* 0:n, 1e3 .* dt_vec(i0, :), 'DisplayName', 'Filtered');
	hold off;
	xlabel('Iteration');
	ylabel(y_label_list{i0});
	title(title_list{i0});
end
