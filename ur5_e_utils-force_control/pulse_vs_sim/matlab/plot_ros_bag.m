addpath(genpath(fullfile('..', '..', '..', '..', '..', '..', 'beamforming_code', 'image_quality_metrics', 'utils')));
clearvars
close all

vision_msg_path = fullfile('..', '..', 'matlab_msg_gen_ros1', 'glnxa64', 'install', 'm');

if ~exist(vision_msg_path, 'dir')
	error(['Could not find directory ''', vision_msg_path, ''', please run script `update_ros_msg.m`.']);
else
	addpath(vision_msg_path);
end

% Open the ROS bag file.
% bag_file_name = fullfile('../bag/test_s3a_0_2024-07-15-15-08-55.bag');

if ~exist('bag_file_name', 'var')
	bag_file_name = get_file_name(fullfile('..', 'bag', '*.bag'), 'Select ROS bag file.');
end

[output_vid_dir_name, output_vid_file_base_name, ~] = fileparts(bag_file_name);
output_vid_file_name = fullfile(output_vid_dir_name, [output_vid_file_base_name, '.avi']);

overwrite_video = true;

if ~exist('overwrite_video', 'var')
	overwrite_video = false;
end

x_lim = [-100, 100];
% x_lim = [-40, 40];

y_lim = [-100, 100];
% y_lim = [-30, 30];

z_lim = [-10, 120];

if((overwrite_video) || (~exist(output_vid_file_name, 'file')))
	fprintf(['Opening file ''', bag_file_name, '''...\n']);
	bag_fh = rosbag(bag_file_name);
	bag_info = rosbag('info', bag_file_name);

	fprintf('Loading TF messages from ROS bag file...\n');
	tf_sel = select(bag_fh, 'Topic', '/tf');
	tf_list = readMessages(tf_sel, 'DataFormat', 'struct');
	tf_id = 1;

	fprintf('Loading PA channel data messages from ROS bag file...\n');
	pa_ch_data_img_sel = select(bag_fh, 'Topic', '/verasonics/channel_data');
	pa_ch_data_img_list = readMessages(pa_ch_data_img_sel, 'DataFormat', 'struct');
	pa_ch_data_img_id = 1;

	fprintf('Loading network detection messages from ROS bag file...\n');
	pt_src_loc_det_sel = select(bag_fh, 'Topic', '/pt_src_loc/detections');
	pt_src_loc_det_list = readMessages(pt_src_loc_det_sel);
	pt_src_loc_det_id = 1;

	fprintf('Loading visualized network detection messages from ROS bag file...\n');
	pt_src_loc_det_img_sel = select(bag_fh, 'Topic', '/pt_src_loc/vis_detections');
	pt_src_loc_det_img_list = readMessages(pt_src_loc_det_img_sel, 'DataFormat', 'struct');
	pt_src_loc_det_img_id = 1;

	fprintf('Loading target pose messages from ROS bag file...\n');
	filter_tgt_pose_sel = select(bag_fh, 'Topic', '/filter/target_pose');
	filter_tgt_pose_list = readMessages(filter_tgt_pose_sel, 'DataFormat', 'struct');
	filter_tgt_pose_id = 1;

	fprintf('Loading FSM state messages from ROS bag file...\n');
	guidance_fsm_state_sel = select(bag_fh, 'Topic', '/guidance/fsm_state');
	guidance_fsm_state_list = readMessages(guidance_fsm_state_sel);
	guidance_fsm_state_id = 1;

	guidance_fsm_state_vec = [];
	guidance_fsm_t_vec = [];

	fprintf('Loading commanded probe pose messages from ROS bag file...\n');
	guidance_probe_cmd_pose_sel = select(bag_fh, 'Topic', '/guidance/probe_cmd_pose');
	guidance_probe_cmd_pose_list = readMessages(guidance_probe_cmd_pose_sel);
	guidance_probe_cmd_pose_id = 1;

	t_start = bag_fh.StartTime;
	t_curr = 0;

	pt_src_pose_base = eye(4);
	probe_cmd_pose_base = eye(4);
	probe_pose_base = eye(4);
	probe_pose_base_pt_src_loc = eye(4);
	particles_pose_base = eye(4);

	pa_ch_data_x_axis = linspace(-9.6, 9.6, 256);
	pa_ch_data_z_axis = linspace(0, 120, 926);
	pa_ch_data_img = uint8(zeros(926, 256));

	[~, bag_file_base_name, ~] = fileparts(bag_file_name);

	if startsWith(bag_file_base_name, 'test_s2')
		pt_src_loc_det_x_axis = linspace(-120 / sqrt(2), 120 / sqrt(2), 1130);
		pt_src_loc_det_z_axis = linspace(0, 120, 926);
		pt_src_loc_det_img = uint8(zeros(926, 1130));
	elseif startsWith(bag_file_base_name, 'test_s3a')
		pt_src_loc_det_x_axis = linspace(-120 / sqrt(2), 120 / sqrt(2), 565);
		pt_src_loc_det_z_axis = linspace(0, 120, 926);
		pt_src_loc_det_img = uint8(zeros(926, 565));
	else
		pt_src_loc_det_x_axis = linspace(-9.6, 9.6, 256);
		pt_src_loc_det_z_axis = linspace(0, 120, 926);
		pt_src_loc_det_img = uint8(zeros(926, 256));
	end

	guidance_fsm_state_vec = zeros(length(guidance_fsm_state_list), 1);
	guidance_fsm_t_vec = zeros(size(guidance_fsm_state_vec));

	% Initialize the video writer.
	fprintf(['Opening file ''', output_vid_file_name, ''' for writing...\n']);
	output_vh = VideoWriter(output_vid_file_name);
	output_vh.FrameRate = 100;
	t_samp = 1.0 / output_vh.FrameRate;
	open(output_vh);

	three_panel_fig_setup;
	t_prev = 0.0;

	while(1)
		t_vec = Inf(8, 1);

		if tf_id <= length(tf_list)
			t_vec(1) = double(tf_list{tf_id}.Transforms.Header.Stamp.Sec) ...
				+ (1e-9 * double(tf_list{tf_id}.Transforms.Header.Stamp.Nsec)) - t_start;
		end

		if pa_ch_data_img_id <= length(pa_ch_data_img_list)
			t_vec(2) = double(pa_ch_data_img_list{pa_ch_data_img_id}.Header.Stamp.Sec) ...
				+ (1e-9 * double(pa_ch_data_img_list{pa_ch_data_img_id}.Header.Stamp.Nsec)) - t_start;
		end

		if pt_src_loc_det_id <= length(pt_src_loc_det_list)
			t_vec(3) = double(pt_src_loc_det_list{pt_src_loc_det_id}.Header.Stamp.Sec) ...
				+ (1e-9 * double(pt_src_loc_det_list{pt_src_loc_det_id}.Header.Stamp.Nsec)) - t_start;
		end

		if pt_src_loc_det_img_id <= length(pt_src_loc_det_img_list)
			t_vec(4) = double(pt_src_loc_det_img_list{pt_src_loc_det_img_id}.Header.Stamp.Sec) ...
				+ (1e-9 * double(pt_src_loc_det_img_list{pt_src_loc_det_img_id}.Header.Stamp.Nsec)) - t_start;
		end

		if filter_tgt_pose_id <= length(filter_tgt_pose_list)
			t_vec(5) = double(filter_tgt_pose_list{filter_tgt_pose_id}.Header.Stamp.Sec) ...
				+ (1e-9 * double(filter_tgt_pose_list{filter_tgt_pose_id}.Header.Stamp.Nsec)) - t_start;
		end

		if guidance_fsm_state_id <= length(guidance_fsm_state_list)
			t_vec(6) = double(guidance_fsm_state_list{guidance_fsm_state_id}.Header.Stamp.Sec) ...
				+ (1e-9 * double(guidance_fsm_state_list{guidance_fsm_state_id}.Header.Stamp.Nsec)) - t_start;
		end

		if guidance_probe_cmd_pose_id <= length(guidance_probe_cmd_pose_list)
			t_vec(7) = double(guidance_probe_cmd_pose_list{guidance_probe_cmd_pose_id}.Header.Stamp.Sec) ...
				+ (1e-9 * double(guidance_probe_cmd_pose_list{guidance_probe_cmd_pose_id}.Header.Stamp.Nsec)) - t_start;
		end

		if any(isfinite(t_vec))
			fprintf('Determining index of message with current lowest time stamp...\n');
			[t_min, min_id] = min(t_vec);
			plot_flag = true;

			while t_min > t_prev + t_samp
				drawnow();
				vid_frame = getframe(gcf);
				writeVideo(output_vh, vid_frame.cdata);
				t_prev = t_prev + t_samp;

				if plot_flag
					if exist('pt_src_pose_base', 'var')
						plotSource(pt_src_pose_base);
					end

					if exist('pt_src_loc_det_msg', 'var')
						plotDetections(pt_src_loc_det_msg, probe_pose_base_pt_src_loc);
					end

					if exist('filter_tgt_pose_msg', 'var')
						plotFilterMsg(filter_tgt_pose_msg);
					end

					if exist('probe_pose_base', 'var')
						plotProbe(probe_pose_base);
					end

					if exist('probe_cmd_pose_base', 'var')
						plotProbe(probe_cmd_pose_base, 'probeColor', [1, 0, 0]);
					end

					nexttile(1);
					hold off
					xlabel('x [mm]');
					ylabel('z [mm]');
					xlim(x_lim);
					ylim(z_lim);

					nexttile(2);
					hold off
					xlabel('x [mm]');
					ylabel('y [mm]');
					xlim(x_lim);
					ylim(y_lim);

					plot_flag = false;
				end
			end

			if min_id == 1
				% We received a new TF frame, so parse it and update the plot.
				tf_msg = tf_list{tf_id};
				tf_id = tf_id + 1;

				if strcmp(tf_msg.Transforms.ChildFrameId, 'pt_src')
					fprintf('%.6f s: TF: pt_src_pose_base\n', t_min);
					pt_src_pose_base = rosReadTransform(tf_msg.Transforms, 'OutputOption', 'single');
				elseif strcmp(tf_msg.Transforms.ChildFrameId, 'probe')
					fprintf('%.6f s: TF: probe_pose_base\n', t_min);
					probe_pose_base = rosReadTransform(tf_msg.Transforms, 'OutputOption', 'single');
				else
					error(['Unknown child frame ID ''', tf_msg.Transforms.ChildFrameId, '''.']);
				end
			elseif min_id == 2
				fprintf('%.6f s: Simulation: Raw channel data\n', t_min);
				pa_ch_data_img = rosReadImage(pa_ch_data_img_list{pa_ch_data_img_id});
				pa_ch_data_img_id = pa_ch_data_img_id + 1;

				nexttile(3);
				imagesc(pa_ch_data_x_axis, pa_ch_data_z_axis, pa_ch_data_img);
				colormap gray
				xlabel('x [mm]');
				ylabel('z [mm]');
			elseif min_id == 3
				fprintf('%.6f s: Localization: Detections from network\n', t_min);
				pt_src_loc_det_msg = pt_src_loc_det_list{pt_src_loc_det_id};
				pt_src_loc_det_id = pt_src_loc_det_id + 1;

				% Store the current transform from the probe frame to the base frame for plotting purposes.
				probe_pose_base_pt_src_loc = probe_pose_base;
			elseif min_id == 4
				fprintf('%.6f s: Localization: Visualized network outputs\n', t_min);
				pt_src_loc_det_img = rosReadImage(pt_src_loc_det_img_list{pt_src_loc_det_img_id});
				pt_src_loc_det_img_id = pt_src_loc_det_img_id + 1;

				nexttile(3);
				imagesc(pt_src_loc_det_x_axis, pt_src_loc_det_z_axis, pt_src_loc_det_img);
				colormap gray
				xlabel('x [mm]');
				ylabel('z [mm]');
			elseif min_id == 5
				fprintf('%.6f s: Filter: Target pose\n', t_min);
				filter_tgt_pose_msg = filter_tgt_pose_list{filter_tgt_pose_id};
				filter_tgt_pose_id = filter_tgt_pose_id + 1;
			elseif min_id == 6
				fprintf('%.6f s: Guidance: FSM state\n', t_min);
				% guidance_fsm_state_msg = guidance_fsm_state_list{guidance_fsm_state_id};
				% guidance_fsm_state_vec(guidance_fsm_state_id) = guidance_fsm_state_msg.Data;
				% guidance_fsm_t_vec(guidance_fsm_state_id) = t_min;
				guidance_fsm_state_id = guidance_fsm_state_id + 1;
			elseif min_id == 7
				fprintf('%.6f s: Guidance: Commanded probe pose\n', t_min);
				guidance_probe_cmd_pose_msg = guidance_probe_cmd_pose_list{guidance_probe_cmd_pose_id};
				probe_cmd_pose_base(1:3, 1:3) = quat2rotm([...
					guidance_probe_cmd_pose_msg.Transform.Rotation.W, ...
					guidance_probe_cmd_pose_msg.Transform.Rotation.X, ...
					guidance_probe_cmd_pose_msg.Transform.Rotation.Y, ...
					guidance_probe_cmd_pose_msg.Transform.Rotation.Z]);
				probe_cmd_pose_base(1:3, 4) = [...
					guidance_probe_cmd_pose_msg.Transform.Translation.X, ...
					guidance_probe_cmd_pose_msg.Transform.Translation.Y, ...
					guidance_probe_cmd_pose_msg.Transform.Translation.Z]';
				guidance_probe_cmd_pose_id = guidance_probe_cmd_pose_id + 1;
			else
				error('Unknown value of `min_id` = %d.', min_id);
			end
		end

		if ~any(isfinite(t_vec))
			break;
		end
	end

	fprintf(['Closing file ''', output_vid_file_name, '''...\n']);
	close(output_vh);
end
