function [lin_reg_transform] = computeLinRegTransform(input_data)
	y = input_data(:, 2);
	x = [ones(size(input_data, 1), 1), input_data(:, 1)];
	b = x \ y;
	m = b(2);

	lin_reg_rot_mat = [1, -m; m, 1] ./ sqrt(1 + (m ^ 2));
	lin_reg_rot_mat = [lin_reg_rot_mat', zeros(2, 1); zeros(1, 2), 1];
	lin_reg_trans_vec = -[mean(input_data(:, 1:2), 1), min(input_data(:, 3))] * lin_reg_rot_mat';
	lin_reg_transform = [lin_reg_rot_mat, lin_reg_trans_vec'; 0, 0, 0, 1];
end
