bf_code_dir_name = fullfile('..', '..', '..', '..', '..', '..', '..', '..', 'beamforming_code');
addpath(fullfile(bf_code_dir_name, 'beamforming_functions'));
addpath(genpath(fullfile(bf_code_dir_name, 'image_quality_metrics', 'utils')));

set(...
	0, ...
	'defaultlinelinewidth', 3, ...
	'defaultaxesfontsize', 16, ...
	'defaultlinemarkersize', 7, ...
	'defaulterrorbarlinewidth', 3);

img_id = 1;
input_img_file_name= fullfile(...
	'..', '..', '..', '..', '..', '..', '..', '..', '..', 'Datasets', 'paimdb', 'images', 'p42v_12cm_cath', ...
	'p42v_12cm_cath_phant_20240905', 'Images', sprintf('%06d.jpg', img_id));

fprintf(['Reading file ''', input_img_file_name, '''...\n']);
input_img = imread(input_img_file_name);

output_mat_dir_name = fullfile('.', 'dat');
createMissingDirectories(output_mat_dir_name);
output_mat_file_name = fullfile(output_mat_dir_name, sprintf('expt_%06d.mat', img_id));

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

img_depth = kwave_metadata.axialLength;
x_axis = 1e3 .* linspace(-img_depth / sqrt(2), img_depth / sqrt(2), size(input_img, 2));
z_axis = 1e3 .* linspace(0, img_depth, size(input_img, 1));

pa_db_img = db(das_pa.PA_img ./ max(das_pa.PA_img(:)));
imagesc(x_axis, z_axis, pa_db_img, [-10, 0]);
colormap gray
xlim([-5, 5]);
ylim([21, 31]);
colorbar
xlabel({'Lateral [mm]'; '(b)'});
ylabel('Axial [mm]');
title('Phantom');

output_img_file_name = fullfile(output_img_dir_name, sprintf('das_pa_expt_%06d.png', img_id));
fprintf(['Saving file ''', output_img_file_name, '''...\n']);
saveas(gcf, output_img_file_name);
