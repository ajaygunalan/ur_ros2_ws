bf_code_dir_name = fullfile('..', '..', '..', '..', '..', '..', '..', '..', 'beamforming_code');
addpath(fullfile(bf_code_dir_name, 'beamforming_functions'));
addpath(genpath(fullfile(bf_code_dir_name, 'image_quality_metrics', 'utils')));

set(...
	0, ...
	'defaultlinelinewidth', 3, ...
	'defaultaxesfontsize', 16, ...
	'defaultlinemarkersize', 7, ...
	'defaulterrorbarlinewidth', 3);

% input_mat_file_name = fullfile(...
% 	'..', '..', 'datasets', 'AnimalStudy20241106', 'das_pa_p42v', 'chdat_20241106_132059_019.mat');

input_mat_file_name = fullfile(...
	'..', '..', 'datasets', 'AnimalStudy20241106', 'das_pa_p42v', 'chdat_20241106_160422_017.mat');

output_img_dir_name = fullfile('.', 'img');
createMissingDirectories(output_img_dir_name);

fprintf(['Loading file ''', input_mat_file_name, '''...\n']);
das_pa = load(input_mat_file_name);

pa_db_img = db(das_pa.pa_img ./ max(das_pa.pa_img(:)));
imagesc(das_pa.x_axis, das_pa.z_axis, pa_db_img, [-6, 0]);
colormap gray
xlim([-10, 10]);
ylim([60, 90]);
colorbar
xlabel({'Lateral [mm]'; '(b)'});
ylabel('Axial [mm]');
title('{\itIn Vivo}');

[~, input_file_base_name, ~] = fileparts(input_mat_file_name);

output_img_file_name = fullfile(output_img_dir_name, [strrep(input_file_base_name, 'chdat', 'das_pa'), '.png']);
fprintf(['Saving file ''', output_img_file_name, '''...\n']);
saveas(gcf, output_img_file_name);
