function z_curr = linearMeasureState(x_curr, varargin)
	p = inputParser();

	addParameter(p, 'C', eye(size(x_curr, 1)), @isnumeric);
	addParameter(p, 'Q', 0.1 * eye(size(x_curr, 1)), @isnumeric);

	p.KeepUnmatched = true;
	parse(p, varargin{:});

	measurement_noise = p.Results.Q * randn(size(x_curr, 1), size(x_curr, 2));
	z_curr = (p.Results.C * x_curr) + measurement_noise;
end
