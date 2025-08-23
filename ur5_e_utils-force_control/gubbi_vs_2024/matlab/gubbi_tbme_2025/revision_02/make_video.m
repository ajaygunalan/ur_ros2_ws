bf_code_dir_name = fullfile('..', '..', '..', '..', '..', '..', '..', '..', 'beamforming_code');
addpath(fullfile(bf_code_dir_name, 'beamforming_functions'));
addpath(genpath(fullfile(bf_code_dir_name, 'image_quality_metrics', 'utils')));

set(...
	0, ...
	'defaultlinelinewidth', 3, ...
	'defaultaxesfontsize', 16, ...
	'defaultlinemarkersize', 7, ...
	'defaulterrorbarlinewidth', 3);

close all
figure();
% set(gcf, 'Position', [10, 10, 320, 720]);
set(gcf, 'Position', [10, 10, 560, 720]);
pause(0.1);

% makeVideo(3, 72.0, 163.0);
makeVideo(4, 212.0, 253.0, 'dynamicRange', 15);

function makeVideo(trial_id, t_start, t_end, varargin)
	p = inputParser();

	addParameter(p, 'dynamicRange', 20, @isnumeric);
	addParameter(p, 'liveVideoStart', 34, @isnumeric);
	addParameter(p, 'liveVideoEnd', 59, @isnumeric);
	addParameter(p, 'robotLeft', 1, @isnumeric);
	addParameter(p, 'catheterLeft', 801, @isnumeric);
	addParameter(p, 'robotWidth', 400, @isnumeric);
	addParameter(p, 'catheterWidth', 400, @isnumeric);

	parse(p, varargin{:});

	dyn_range = p.Results.dynamicRange;

	chdat_start_id = 0;
	chdat_end_id = 2803;

	live_vid_start = p.Results.liveVideoStart;
	live_vid_end = p.Results.liveVideoEnd;

	robot_left = p.Results.robotLeft;
	robot_width = p.Results.robotWidth;
	robot_right = robot_left + robot_width - 1;
	catheter_left = p.Results.catheterLeft;
	catheter_width = p.Results.catheterWidth;
	catheter_right = catheter_left + catheter_width - 1;

	% input_schematic_file_name = fullfile('.', 'dat', 'video_schematic.png');
	% input_schematic = double(imread(input_schematic_file_name));
	% input_schematic = input_schematic(:, 1:3200, :);
	% input_schematic = imresize(input_schematic, [223, 399]);
	% input_schematic = imresize(input_schematic, 380.0 / size(input_schematic, 2));

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

	input_dir_name = fullfile(...
		'..', '..', '..', '..', '..', '..', 'bag', '20241106_animal_study', 'reindexed', ...
		'sys_d_2024-11-06-16-08-59_02');

	robot_poses_file_name = fullfile(input_dir_name, 'robot_poses.h5');
	sys_c_time = h5read(robot_poses_file_name, '/sys_c/time');

	output_vid_dir_name = fullfile('.', 'vid');
	createMissingDirectories(output_vid_dir_name);
	output_vid_file_name = fullfile(output_vid_dir_name, sprintf('trial_%02d_02.avi', trial_id));
	output_vh = VideoWriter(output_vid_file_name);
	output_vh.Quality = 95;

	ch_x_axis = linspace(-9.6, 9.6, 256);
	ch_z_axis = linspace(0, 120, 926);

	live_vid_file_name = fullfile(...
		'..', '..', '..', '..', '..', '..', '..', '..', '..', 'Datasets', 'AnimalStudy20241106', 'Photos', ...
		'20241106_191429000_iOS.MOV');
	live_vh = VideoReader(live_vid_file_name);
	live_vh.CurrentTime = live_vid_start;
	output_vh.FrameRate = live_vh.FrameRate;
	open(output_vh);

	while live_vh.CurrentTime < live_vid_end
		live_vid_t_curr = live_vh.CurrentTime - live_vid_start;
		fprintf('%.2f s\n', live_vid_t_curr);
		live_frame = readFrame(live_vh);
		robot_frame = live_frame(:, robot_left:robot_right, :);
		catheter_frame = live_frame(:, catheter_left:catheter_right, :);

		z = find(sys_c_time - t_start <= live_vid_t_curr);

		if isempty(z)
			% Off-by-one correction.
			z = chdat_start_id + 1;
		end

		% Channel data file names start from zero.
		i0 = z(end) - 1;

		input_chdat_file_name = fullfile(input_dir_name, sprintf('chdat_%06d.h5', i0));
		fprintf(['Reading file ''', input_chdat_file_name, '''...\n']);
		ch_data = permute(h5read(input_chdat_file_name, '/chdat'), [3, 2, 1]);

		output_das_pa_file_name = fullfile(input_dir_name, sprintf('das_pa_%06d.mat', i0));

		if ~exist(output_das_pa_file_name, 'file')
			das_pa = struct();
			fprintf(['Delaying data...\n']);
			ch_data_resized = imresize(double(rgb2gray(ch_data)), [926, 64]);
			[delay_data, pa_metadata] = delay_PA_phased(...
				'is_kwave', 1, 'kwave_data', ch_data_resized', 'kwave_metadata', kwave_metadata);
			fprintf(['Beamforming data...\n']);
			[das_pa.pa_img, ~, metadata, das_pa.x_axis, das_pa.z_axis] = beamformer_DAS_PA_phased(...
				delay_data, pa_metadata);

			das_pa.x_axis = das_pa.x_axis - mean(das_pa.x_axis);

			fprintf(['Saving file ''', output_das_pa_file_name, '''...\n']);
			save(output_das_pa_file_name, '-struct', 'das_pa');
		else
			fprintf(['Loading file ''', output_das_pa_file_name, '''...\n']);
			das_pa = load(output_das_pa_file_name);
		end

		input_seg_mask_file_name = fullfile(input_dir_name, sprintf('seg_mask_c_%06d.h5', i0));

		if exist(input_seg_mask_file_name, 'file')
			seg_mask = h5read(input_seg_mask_file_name, '/sys_c/src_mask_01');
			ch_data = labeloverlay(ch_data, seg_mask');
		else
			fprintf('Skipping segmentation mask overlay...\n');
		end

		imagesc(ch_x_axis, ch_z_axis, ch_data);
		colormap gray
		colorbar
		ylim([0, 100]);
		xlabel('Lateral [mm]');
		ylabel('Axial [mm]');
		title('Channel Data');

		ch_data_frame = getframe(gcf);

		pa_db_img = db(das_pa.pa_img ./ max(das_pa.pa_img(:)));
		imagesc(das_pa.x_axis, das_pa.z_axis, pa_db_img, [-dyn_range, 0]);
		% imagesc(das_pa.x_axis, das_pa.z_axis, das_pa.pa_img);
		colormap gray
		colorbar

		input_sys_c_file_name = fullfile(input_dir_name, sprintf('sys_c_%06d.h5', i0));

		if exist(input_sys_c_file_name, 'file')
			sys_c_loc_est = h5read(input_sys_c_file_name, '/pt_src/p42v_link1');
			sys_c_loc_est = sys_c_loc_est(:, end);
			sys_c_sos_est = h5read(input_sys_c_file_name, '/pt_src/sound_speed');
			sys_c_sos_est = sys_c_sos_est(end);
			hold on
			plot(...
				1e3 .* sys_c_loc_est(1), ...
				1e3 .* sys_c_loc_est(3) * 1540.0 / sys_c_sos_est, ...
				'Marker', '+', 'MarkerSize', 15);
			hold off
		end

		xlabel('Lateral [mm]');
		ylabel('Axial [mm]');
		title('Delay-and-Sum');
		das_pa_frame = getframe(gcf);

		cat_frame = [fliplr(catheter_frame), fliplr(robot_frame), ch_data_frame.cdata, das_pa_frame.cdata];

		% schematic_row_start = size(cat_frame, 1) - size(input_schematic, 1) - 9;
		% schematic_row_end = schematic_row_start + size(input_schematic, 1) - 1;
		% schematic_col_start = 11;
		% schematic_col_end = schematic_col_start + size(input_schematic, 2) - 1;
		% cat_frame(schematic_row_start:schematic_row_end, schematic_col_start:schematic_col_end, :) = ...
		% 	uint8(input_schematic(:, :, [1, 1, 1]));

		zero_pad = zeros(180, size(cat_frame, 2), size(cat_frame, 3), 'uint8');
		output_frame = [zero_pad; cat_frame; zero_pad];
		% output_frame = cat_frame;
		fprintf('Writing frame of size %d x %d...\n', size(output_frame, 1), size(output_frame, 2));
		writeVideo(output_vh, output_frame);
	end

	close(output_vh);
end
