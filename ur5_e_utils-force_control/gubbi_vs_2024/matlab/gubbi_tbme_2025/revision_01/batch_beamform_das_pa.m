bf_code_dir_name = fullfile('..', '..', '..', '..', '..', '..', '..', '..', 'beamforming_code');
addpath(fullfile(bf_code_dir_name, 'beamforming_functions'));
addpath(genpath(fullfile(bf_code_dir_name, 'image_quality_metrics', 'utils')));

input_chdat_dir_name = fullfile('..', '..', 'datasets', 'AnimalStudy20241106', 'raw_data_p42v');
output_das_pa_dir_name = fullfile(fileparts(input_chdat_dir_name), 'das_pa_p42v');
createMissingDirectories(output_das_pa_dir_name);

input_chdat_mat_file_list = dir(fullfile(input_chdat_dir_name, 'chdat*.mat'));
pa_chdat_length = 926;

% These images were acquired with 7 US transmit angles followed by a PA acquisition, so Receive(8) contains PA
% parameters.
pa_rcv_id = 8;

kwave_metadata = struct(... %kwave metadata structure for P4-2v and parameters used in code
	'speedOfSoundMps', 1540,...
	'fs', 4 * 2.97, ...
	'f0', 2.97, ...
	'kerf', 50e-6, ...
	'elementWidth', 250e-6, ...
	'axialLength', 100e-3, ...
	'phantomDepth', 100e-3, ...
	'FOV', 90.0, ...
	'st_ang', -45.0, ...
	'radius', 0.0);

dyn_range = 10;

set(...
	0, ...
	'defaultlinelinewidth', 3, ...
	'defaultaxesfontsize', 16, ...
	'defaultlinemarkersize', 7, ...
	'defaulterrorbarlinewidth', 3);

for i0 = 1:length(input_chdat_mat_file_list)
	input_chdat_file_name = fullfile(input_chdat_mat_file_list(i0).folder, input_chdat_mat_file_list(i0).name);
	fprintf(['Loading file ''', input_chdat_file_name, '''...\n']);
	bpai_ch_data = load(fullfile(input_chdat_file_name));
	num_frames = size(bpai_ch_data.RcvData{1}, 3);

	pa_start_sample = bpai_ch_data.Receive(pa_rcv_id).startSample;
	pa_end_sample = pa_start_sample + pa_chdat_length - 1;

	[~, input_file_base_name, ~] = fileparts(input_chdat_file_name);

	for i1 = 1:num_frames
		output_file_base_name = [input_file_base_name, sprintf('_%03d', i1)];
		output_das_pa_mat_file_name = fullfile(output_das_pa_dir_name, [output_file_base_name, '.mat']);

		if ~exist(output_das_pa_mat_file_name, 'file')
			pa_ch_data = bpai_ch_data.RcvData{1}(pa_start_sample:pa_end_sample, 33:96, i1);
			das_pa = struct();

			fprintf(['Delaying data...\n']);
			[delay_data, pa_metadata] = delay_PA_phased(...
				'is_kwave', 1, 'kwave_data', pa_ch_data', 'kwave_metadata', kwave_metadata);
			fprintf(['Beamforming data...\n']);
			[das_pa.pa_img, ~, metadata, das_pa.x_axis, das_pa.z_axis] = beamformer_DAS_PA_phased(...
				delay_data, pa_metadata);

			das_pa.x_axis = das_pa.x_axis - mean(das_pa.x_axis);

			fprintf(['Saving file ''', output_das_pa_mat_file_name, '''...\n']);
			save(output_das_pa_mat_file_name, '-struct', 'das_pa');
		else
			fprintf(['Loading file ''', output_das_pa_mat_file_name, '''...\n']);
			das_pa = load(output_das_pa_mat_file_name);
		end

		output_das_pa_img_file_name = fullfile(output_das_pa_dir_name, [output_file_base_name, '.png']);

		if ~exist(output_das_pa_img_file_name, 'file')
			das_pa_db = db(das_pa.pa_img ./ max(das_pa.pa_img(:)));
			imagesc(das_pa.x_axis, das_pa.z_axis, das_pa_db, [-dyn_range, 0]);
			colormap gray
			colorbar
			xlabel('Lateral [mm]');
			ylabel('Axial [mm]');

			fprintf(['Saving file ''', output_das_pa_img_file_name, '''...\n']);
			saveas(gcf, output_das_pa_img_file_name);
		end
	end
end
