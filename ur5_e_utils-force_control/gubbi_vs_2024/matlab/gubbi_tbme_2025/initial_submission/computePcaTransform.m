function [pca_transform] = computePcaTransform(input_data)
	pca_rot_mat = pca(input_data(:, 1:2));
	pca_rot_mat = [pca_rot_mat', zeros(2, 1); zeros(1, 2), 1];
	pca_trans_vec = -[mean(input_data(:, 1:2), 1), min(input_data(:, 3))] * pca_rot_mat';
	pca_transform = [pca_rot_mat, pca_trans_vec'; 0, 0, 0, 1];
end
