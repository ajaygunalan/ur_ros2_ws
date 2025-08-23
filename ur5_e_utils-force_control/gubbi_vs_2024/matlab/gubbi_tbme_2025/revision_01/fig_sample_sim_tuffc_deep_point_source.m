bf_code_dir_name = fullfile('..', '..', '..', '..', '..', '..', '..', '..', 'beamforming_code');
addpath(fullfile(bf_code_dir_name, 'beamforming_functions'));
addpath(genpath(fullfile(bf_code_dir_name, 'image_quality_metrics', 'utils')));

set(...
	0, ...
	'defaultlinelinewidth', 3, ...
	'defaultaxesfontsize', 16, ...
	'defaultlinemarkersize', 7, ...
	'defaulterrorbarlinewidth', 3);

% input_img_dir_name = fullfile(...
% 	'..', '..', '..', '..', '..', '..', '..', '..', '..', 'Datasets', 'paimdb', 'images', 'p42v_12cm_spheres', ...
% 	'p42v_12cm_1s1r_500um_20240903_bp_109p4');

input_img_dir_name = fullfile(...
	'..', '..', '..', '..', '..', '..', '..', '..', '..', 'Datasets', 'paimdb', 'images', ...
	'p42v_12cm_1s1r_100um_20241017');

metadata_mat_file_name = fullfile(input_img_dir_name, 'metadata.mat');
fprintf(['Loading file ''', metadata_mat_file_name, '''...\n']);
metadata = load(metadata_mat_file_name);

z = find(...
	(metadata.img.ref.num.vec == 0) ...
	& (abs(metadata.img.src.pos.vec(:, 1, 1) - 5e-3) <= 1e-3) ...
	& (abs(metadata.img.src.pos.vec(:, 1, 2)) <= 1e-3) ...
	& (abs(metadata.img.src.pos.vec(:, 1, 3) - 88e-3) <= 5e-3) ...
	& (abs(metadata.img.sos.vec - 1540) <= 30));

fprintf('Found %d images satisfying criteria.\n', numel(z));
img_id = z(1);
input_img_file_name = fullfile(input_img_dir_name, 'Images', sprintf('%06d.jpg', img_id));
fprintf(['Loading file ''', input_img_file_name, '''...\n']);
input_img = imread(input_img_file_name);

output_mat_dir_name = fullfile('.', 'dat');
output_img_dir_name = fullfile('.', 'img');
createMissingDirectories(output_mat_dir_name, output_img_dir_name);
output_mat_file_name = fullfile(output_mat_dir_name, sprintf('sim_%06d.mat', img_id));

kwave_metadata = struct(...
	'fs', 4.0 * 2.97, ...
	'f0', 2.97, ...
	'kerf', 50e-6, ...
	'elementWidth', 250e-6, ...
	'axialLength', 100e-3, ...
	'phantomDepth', 30e-3, ...
	'FOV', 90, ...
	'st_ang', -45, ...
	'radius', 0);

if ~exist(output_mat_file_name, 'file')
	fprintf('Delaying data...\n');
	[das_pa_delay_data, das_pa_metadata] = delay_PA_phased(...
		'is_kwave', 1, ...
		'kwave_data', input_img', 'kwave_metadata', kwave_metadata, ...
		'sound_speed', 1540);

	fprintf('Beamforming data...\n');
	[das_pa.PA_img, ~, ~, das_pa.x_axis, das_pa.z_axis] = ...
		beamformer_DAS_PA_phased(das_pa_delay_data, das_pa_metadata);

	das_pa.x_axis = das_pa.x_axis - mean(das_pa.x_axis);
	fprintf(['Saving file ''', output_mat_file_name, '''...\n']);
	save(output_mat_file_name, '-struct', 'das_pa');
else
	fprintf(['Loading file ''', output_mat_file_name, '''...\n']);
	das_pa = load(output_mat_file_name);
end

fprintf('Source location = [%.2f, %.2f, %.2f] mm\n', 1e3 .* squeeze(metadata.img.src.pos.vec(img_id, 1, :)));
fprintf('Sound speed = %.1f m/s\n', metadata.img.sos.vec(img_id));

img_depth = kwave_metadata.axialLength;
x_axis = 1e3 .* linspace(-img_depth / sqrt(2), img_depth / sqrt(2), size(input_img, 2));
z_axis = 1e3 .* linspace(0, img_depth, size(input_img, 1));

pa_db_img = db(das_pa.PA_img ./ max(das_pa.PA_img(:)));
imagesc(x_axis, z_axis, pa_db_img, [-10, 0]);
colormap gray
xlim([-5, 15]);
ylim([78, 98]);
colorbar
xlabel({'Lateral [mm]'; '(a)'});
ylabel('Axial [mm]');
title('Simulated');

output_img_file_name = fullfile(output_img_dir_name, sprintf('das_pa_sim_%06d.png', img_id));
fprintf(['Saving file ''', output_img_file_name, '''...\n']);
saveas(gcf, output_img_file_name);
