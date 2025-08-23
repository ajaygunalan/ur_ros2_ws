function [src_file_name, src_file_id] = getIntegratedSensorDataFileName(metadata, src_dir_name, x_id, y, z)
	[~, y_id] = min(abs(abs(y) - metadata.img.src.pos.unique_y));
	[~, z_id] = min(abs(z - metadata.img.src.pos.unique_z));
	src_file_id = x_id + ((y_id - 1) * size(metadata.img.src.pos.unique_x, 1)) ...
		+ ((z_id - 1) * size(metadata.img.src.pos.unique_x, 1) * size(metadata.img.src.pos.unique_y, 1));
	src_file_name = fullfile(src_dir_name, sprintf('%06d.mat', src_file_id));
end
