function x_mat = hatOperator(x_vec)
	x_mat = zeros(3);
	x_mat(1, 2) = -x_vec(3);
	x_mat(1, 3) = x_vec(2);
	x_mat(2, 3) = -x_vec(1);
	x_mat = x_mat - x_mat';
end

