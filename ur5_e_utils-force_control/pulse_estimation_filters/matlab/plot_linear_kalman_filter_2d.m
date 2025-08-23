n_start = 1;
nexttile(1);
scatter(z_vec(1, n_start:i0), z_vec(2, n_start:i0), [], color_mat(3, :), 'DisplayName', 'Measurement');
hold on
scatter(x_vec(1, n_start:i0), x_vec(2, n_start:i0), [], color_mat(1, :), 'DisplayName', 'Position');
hold off
legend('location', 'southeast');
xlim(x_lim);
ylim(y_lim);
xlabel('x [mm]');
ylabel('y [mm]');
title('Position and Measurement');

nexttile(2);
scatter(z_vec(1, n_start:i0), z_vec(2, n_start:i0), [], color_mat(3, :), 'DisplayName', 'Measurement');
hold on
scatter(mu_vec(1, n_start:i0), mu_vec(2, n_start:i0), [], color_mat(4, :), 'DisplayName', 'Filter');
hold off
legend('location', 'southeast');
xlim(x_lim);
ylim(y_lim);
xlabel('x [mm]');
ylabel('y [mm]');
title('Measurement and Filter Output');

nexttile(3);
scatter(mu_vec(1, n_start:i0), mu_vec(2, n_start:i0), [], color_mat(4, :), 'DisplayName', 'Filter');
hold on
scatter(x_vec(1, n_start:i0), x_vec(2, n_start:i0), [], color_mat(1, :), 'DisplayName', 'Position');
hold off
legend('location', 'southeast');
xlim(x_lim);
ylim(y_lim);
xlabel('x [mm]');
ylabel('y [mm]');
title('Position and Filter Output');

nexttile(4);
scatter(x_vec(1, :) - mu_vec(1, :), x_vec(2, :) - mu_vec(2, :), [], color_mat(2, :), 'DisplayName', 'Error');
legend('location', 'southeast');
xlim(x_lim);
ylim(y_lim);
xlabel('x [mm]');
ylabel('y [mm]');
title('Filter Error');

drawnow();
pause(0.01);
