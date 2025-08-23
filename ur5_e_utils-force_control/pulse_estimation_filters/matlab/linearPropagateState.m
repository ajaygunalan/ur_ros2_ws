function x_curr = linearPropagateState(x_prev, u_curr, varargin)
	p = inputParser();

	addParameter(p, 'A', eye(size(x_prev, 1)), @isnumeric);
	addParameter(p, 'B', eye(size(x_prev, 1)), @isnumeric);
	addParameter(p, 'R', 0.05 * eye(size(x_prev, 1)), @isnumeric);

	p.KeepUnmatched = true;
	parse(p, varargin{:});

	% Propagate the state.
	process_noise = p.Results.R * randn(size(x_prev, 1), size(x_prev, 2));
	x_curr = (p.Results.A * x_prev) + (p.Results.B * u_curr) + process_noise;
end
