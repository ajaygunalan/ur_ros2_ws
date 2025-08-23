max_num_dof = 1000;
x_vec = [1:max_num_dof]';
chi_squared_vec = zeros(max_num_dof, 1);
alpha_val = 0.01;

for i0 = 1:max_num_dof
	chi_squared_vec(i0) = chi2inv(1 - alpha_val, i0);
end

lin_coeffs = polyfit(x_vec, chi_squared_vec, 2);

plot(x_vec, chi_squared_vec);
hold on
plot(x_vec, (lin_coeffs(1) .* (x_vec .^ 2)) + (lin_coeffs(2) .* x_vec) + lin_coeffs(3));
hold off
xlabel('Degrees of Freedom');
ylabel(sprintf('Chi-Squared for alpha=%.3f', alpha_val));
