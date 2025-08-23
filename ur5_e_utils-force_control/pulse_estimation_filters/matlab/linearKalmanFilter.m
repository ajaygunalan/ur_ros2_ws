function [mu_curr, sigma_curr] = linearKalmanFilter(mu_prev, sigma_prev, u_curr, z_curr, varargin)
	p = inputParser();

	addParameter(p, 'A', eye(size(mu_prev, 1)), @isnumeric);
	addParameter(p, 'B', eye(size(mu_prev, 1)), @isnumeric);
	addParameter(p, 'R', 0.1 * eye(size(mu_prev, 1)), @isnumeric);

	addParameter(p, 'C', eye(size(mu_prev, 1)), @isnumeric);
	addParameter(p, 'Q', 0.1 * eye(size(mu_prev, 1)), @isnumeric);

	p.KeepUnmatched = true;
	parse(p, varargin{:});

	a_mat = p.Results.A;
	c_mat = p.Results.C;

	mu_prior = (a_mat * mu_prev) + (p.Results.B * u_curr);
	sigma_prior = (a_mat * sigma_prev * a_mat') + p.Results.R;

	k_mat = sigma_prior * c_mat' * inv(c_mat * sigma_prior * c_mat' + p.Results.Q);
	mu_curr = mu_prior + (k_mat * (z_curr - (c_mat * mu_prior)));
	sigma_curr = (eye(size(mu_prev)) - (k_mat * c_mat')) * sigma_prior;
end
