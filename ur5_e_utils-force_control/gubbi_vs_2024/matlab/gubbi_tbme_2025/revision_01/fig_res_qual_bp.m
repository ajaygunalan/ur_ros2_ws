bf_code_dir_name = fullfile('..', '..', '..', '..', '..', '..', '..', '..', 'beamforming_code');
addpath(fullfile(bf_code_dir_name, 'beamforming_functions'));
addpath(genpath(fullfile(bf_code_dir_name, 'image_quality_metrics', 'utils')));

close all
set(...
	0, ...
	'defaultlinelinewidth', 3, ...
	'defaultaxesfontsize', 16, ...
	'defaultlinemarkersize', 7, ...
	'defaulterrorbarlinewidth', 3);
figure(1);
set(gcf, 'Position', [100, 100, 1000, 800]);
tiledlayout(2, 2, 'TileSpacing', 'compact');

sim_img_dir_name = fullfile(...
	'..', '..', '..', '..', '..', '..', '..', '..', '..', 'Datasets', 'paimdb', 'images', ...
	'p42v_12cm_1s1r_500um_20240903_bp_109p4');

metadata_mat_file_name = fullfile(sim_img_dir_name, 'metadata.mat');
fprintf(['Loading file ''', metadata_mat_file_name, '''...\n']);
metadata = load(metadata_mat_file_name);

z = find(...
	(metadata.img.ref.num.vec == 0) ...
	& (abs(metadata.img.src.pos.vec(:, 1, 1) + 1e-3) <= 1e-3) ...
	& (abs(metadata.img.src.pos.vec(:, 1, 2)) <= 1e-3) ...
	& (abs(metadata.img.src.pos.vec(:, 1, 3) - 73e-3) <= 5e-3) ...
	& (abs(metadata.img.sos.vec - 1540) <= 30));

fprintf('Found %d images satisfying criteria.\n', numel(z));
img_id = z(1);
sim_img_file_name = fullfile(sim_img_dir_name, 'Images', sprintf('%06d.jpg', img_id));
fprintf(['Loading file ''', sim_img_file_name, '''...\n']);
sim_img = imread(sim_img_file_name);

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

sim_output_mat_dir_name = fullfile('.', 'dat');
sim_output_img_dir_name = fullfile('.', 'img');
createMissingDirectories(sim_output_mat_dir_name, sim_output_img_dir_name);
sim_output_mat_file_name = fullfile(sim_output_mat_dir_name, sprintf('bp_sim_%06d.mat', img_id));

if ~exist(sim_output_mat_file_name, 'file')
	fprintf('Delaying data...\n');
	[das_pa_delay_data, das_pa_metadata] = delay_PA_phased(...
		'is_kwave', 1, ...
		'kwave_data', sim_img', 'kwave_metadata', kwave_metadata, ...
		'sound_speed', 1540);

	fprintf('Beamforming data...\n');
	[das_pa.PA_img, ~, ~, das_pa.x_axis, das_pa.z_axis] = ...
		beamformer_DAS_PA_phased(das_pa_delay_data, das_pa_metadata);

	das_pa.x_axis = das_pa.x_axis - mean(das_pa.x_axis);
	fprintf(['Saving file ''', sim_output_mat_file_name, '''...\n']);
	save(sim_output_mat_file_name, '-struct', 'das_pa');
else
	fprintf(['Loading file ''', sim_output_mat_file_name, '''...\n']);
	das_pa = load(sim_output_mat_file_name);
end

nexttile(1);

pa_db_img = db(das_pa.PA_img ./ max(das_pa.PA_img(:)));
imagesc(das_pa.x_axis, das_pa.z_axis, pa_db_img, [-6, 0]);
colormap gray
xlim([-10, 10]);
ylim([60, 90]);
% colorbar
xlabel({'Lateral [mm]'; '(a)'});
ylabel({'{\bfDelay-and-Sum}'; 'Axial [mm]'});
title('Simulated');

vivo_chdat_file_name = fullfile(...
	'..', '..', 'datasets', 'AnimalStudy20241106', 'raw_data_p42v', 'chdat_20241106_160422.mat');
vivo_frame_id = 17;
vivo_mat_file_name = fullfile(...
	'..', '..', 'datasets', 'AnimalStudy20241106', 'das_pa_p42v', ...
	sprintf('chdat_20241106_160422_%03d.mat', vivo_frame_id));

fprintf(['Loading file ''', vivo_mat_file_name, '''...\n']);
das_pa = load(vivo_mat_file_name);

nexttile(3);
sim_img = double(sim_img);
sim_img = sim_img - min(sim_img(:));
sim_img = sim_img ./ max(sim_img(:));
ch_x_axis = linspace(-19.2/2.0, 19.2/2.0, size(sim_img, 2));
ch_z_axis = linspace(0, 120, size(sim_img, 1));
imagesc(ch_x_axis, ch_z_axis, sim_img);
colormap gray
xlabel({'Lateral [mm]'; '(c)'});
% ylabel('Axial [mm]');
ylabel({'{\bfChannel Data}'; 'Axial [mm]'});
ylim([60, 90]);

nexttile(4);
bpai_ch_data = load(vivo_chdat_file_name);
pa_chdat_length = 926;
pa_rcv_id = 8;
pa_start_sample = bpai_ch_data.Receive(pa_rcv_id).startSample;
pa_end_sample = pa_start_sample + pa_chdat_length - 1;
expt_ch_data = bpai_ch_data.RcvData{1}(pa_start_sample:pa_end_sample, 33:96, vivo_frame_id);

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

fprintf(['Delaying data...\n']);
[delay_data, pa_metadata] = delay_PA_phased(...
	'is_kwave', 1, 'kwave_data', expt_ch_data', 'kwave_metadata', kwave_metadata);
fprintf(['Beamforming data...\n']);
[das_pa.pa_img, ~, metadata, das_pa.x_axis, das_pa.z_axis] = beamformer_DAS_PA_phased(...
	delay_data, pa_metadata);
das_pa.x_axis = das_pa.x_axis - mean(das_pa.x_axis);

expt_ch_data = double(expt_ch_data);
expt_ch_data = expt_ch_data - min(expt_ch_data(:));
expt_ch_data = expt_ch_data ./ max(expt_ch_data(:));
imagesc(ch_x_axis, ch_z_axis, expt_ch_data);
colormap gray
xlabel({'Lateral [mm]'; '(d)'});
yticklabels({});
ylim([60, 90]);
colorbar();

nexttile(2);
pa_db_img = db(das_pa.pa_img ./ max(das_pa.pa_img(:)));
imagesc(das_pa.x_axis, das_pa.z_axis, pa_db_img, [-6, 0]);
colormap gray
xlim([-10, 10]);
ylim([60, 90]);
c_bar = colorbar();
ylabel(c_bar, '(dB)');
% colorbar('TickLabels', {'-6 dB', '-5 dB', '-4 dB', '-3 dB', '-2 dB', '-1 dB', ' 0 dB'});
xlabel({'Lateral [mm]'; '(b)'});
% ylabel('Axial [mm]');
title('{\itIn Vivo}');
yticklabels({});

output_img_dir_name = fullfile('.', 'img');
createMissingDirectories(output_img_dir_name);
output_img_file_name = fullfile(output_img_dir_name, 'res_qual_bp.png');
fprintf(['Saving file ''', output_img_file_name, '''...\n']);
saveas(gcf, output_img_file_name);
