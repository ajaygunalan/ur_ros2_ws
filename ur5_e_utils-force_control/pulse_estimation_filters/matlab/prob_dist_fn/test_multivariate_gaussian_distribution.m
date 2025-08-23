n = 65536;
x_vec = ((2 * 85 * rand(n, 1)) - 85) * 1e-3;
y_vec = ((2 * 20 * rand(n, 1)) - 20) * 1e-3;
z_vec = ((80 * rand(n, 1)) + 20) * 1e-3;

pos_mat = [x_vec, y_vec, z_vec];
x_c_1 = 1e-3 .* [5, 10, 60];
x_c_2 = 1e-3 .* [5, -10, 60];

sigma_val = 2e-3;
cov_mat_inv = eye(3) / (sigma_val ^ 2);
cov_mat = (sigma_val ^ 2) * eye(3);

prob_vec = zeros(n, 1);
gauss_scaling = 1.0 / sqrt(8.0 * pi * pi * pi * det(cov_mat));

for i0 = 1:n
	err_vec_1 = pos_mat(i0, :) - x_c_1;
	prob_vec(i0) = 0.5 * gauss_scaling * exp(-0.5 * err_vec_1 * cov_mat_inv * err_vec_1');

	err_vec_2 = pos_mat(i0, :) - x_c_2;
	prob_vec(i0) = prob_vec(i0) + (0.5 * gauss_scaling * exp(-0.5 * err_vec_2 * cov_mat_inv * err_vec_2'));
end

figure(1);
tiledlayout(1, 3);

nexttile(1);
scatter(1e3 .* pos_mat(:, 1), prob_vec, '.');
xlabel('x [mm]');
ylabel('p_X(x)');
xlim([-85, 85]);

nexttile(2);
scatter(1e3 .* pos_mat(:, 2), prob_vec, '.');
xlabel('y [mm]');
ylabel('p_Y(y)');
xlim([-20, 20]);

nexttile(3);
scatter(1e3 .* pos_mat(:, 3), prob_vec, '.');
xlabel('z [mm]');
ylabel('p_Z(z)');
xlim([20, 100]);

