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

[bag_dir_name, bag_file_base_name, ~] = fileparts(bag_file_name);
pa_ch_data_dir_name = fullfile(bag_dir_name, bag_file_base_name, 'channel_data');
detections_dir_name = fullfile(bag_dir_name, bag_file_base_name, 'detections');
vis_detections_dir_name = fullfile(bag_dir_name, bag_file_base_name, 'vis_detections');
createMissingDirectories(pa_ch_data_dir_name, detections_dir_name, vis_detections_dir_name);

bag_fh = rosbag(bag_file_name);

pa_ch_data_x_axis = linspace(-9.6, 9.6, 256);
pa_ch_data_z_axis = linspace(0, 120, 926);
pa_ch_data_img = uint8(zeros(926, 256));

pa_ch_data_img_sel = select(bag_fh, 'Topic', '/verasonics/channel_data');
pa_ch_data_img_list = readMessages(pa_ch_data_img_sel, 'DataFormat', 'struct');

for i0 = 1:length(pa_ch_data_img_list)
	pa_ch_data_img = rosReadImage(pa_ch_data_img_list{i0});
	imagesc(pa_ch_data_x_axis, pa_ch_data_z_axis, pa_ch_data_img);
	colormap gray
	xlabel('x [mm]');
	ylabel('z [mm]');

	output_img_file_name = fullfile(pa_ch_data_dir_name, sprintf('%06d.jpg', pa_ch_data_img_list{i0}.Header.Seq));
	fprintf(['Saving file, ''', output_img_file_name, '''...\n']);
	saveas(gcf, output_img_file_name);
end

if startsWith(bag_file_base_name, 'test_s2')
	detection_x_axis = linspace(-120 / sqrt(2), 120 / sqrt(2), 1130);
	detection_z_axis = linspace(0, 120, 926);
	detection_img = uint8(zeros(926, 1130));
	img_resized = [926, 128];
	zero_pad_left = zeros(926, 501);
	zero_pad_right = zeros(926, 501);
elseif startsWith(bag_file_base_name, 'test_s3a')
	detection_x_axis = linspace(-120 / sqrt(2), 120 / sqrt(2), 565);
	detection_z_axis = linspace(0, 120, 926);
	detection_img = uint8(zeros(926, 565));
	img_resized = [926, 64];
	zero_pad_left = zeros(926, 250);
	zero_pad_right = zeros(926, 251);
else
	detection_x_axis = linspace(-9.6, 9.6, 256);
	detection_z_axis = linspace(0, 120, 926);
	detection_img = uint8(zeros(926, 256));
	img_resized = [926, 256];
	zero_pad_left = [];
	zero_pad_right = [];
end

detection_sel = select(bag_fh, 'Topic', '/pt_src_loc/detections');
detection_list = readMessages(detection_sel, 'DataFormat', 'struct');

for i0 = 1:length(detection_list)
	detection_msg = detection_list{i0};

	if ~isempty(detection_msg.Detections)
		detection_img = rosReadImage(detection_msg.Detections(1).SourceImg);
		detection_img = [zero_pad_left, imresize(detection_img, img_resized), zero_pad_right];

		imagesc(detection_x_axis, detection_z_axis, detection_img);
		colormap gray;
		hold on

		for i1 = 1:length(detection_msg.Detections)
			if detection_msg.Detections(i1).Results.Id == 0
				color_vec = [0, 0.4470, 0.7410];
			else
				color_vec = [0.8500, 0.3250, 0.0980];
			end

			scatter(...
				1e3 * detection_msg.Detections(i1).Results.Pose.Pose.Position.X, ...
				1e3 * detection_msg.Detections(i1).Results.Pose.Pose.Position.Z, 'x', 'CData', color_vec);

			bbox_center_vec_px = [...
				detection_msg.Detections(i1).Bbox.Center.X, detection_msg.Detections(i1).Bbox.Center.Y];
			bbox_center_vec_px = round(bbox_center_vec_px);
			bbox_center_vec_px(1) = max(bbox_center_vec_px(1), 1);
			bbox_center_vec_px(1) = min(bbox_center_vec_px(1), length(detection_x_axis));
			bbox_center_vec_px(2) = max(bbox_center_vec_px(2), 1);
			bbox_center_vec_px(2) = min(bbox_center_vec_px(2), length(detection_z_axis));
			bbox_center_vec = [detection_x_axis(bbox_center_vec_px(1)), detection_z_axis(bbox_center_vec_px(2))];

			bbox_dim_vec_px = [detection_msg.Detections(i1).Bbox.SizeX, detection_msg.Detections(i1).Bbox.SizeY];
			bbox_dim_vec = [...
				bbox_dim_vec_px(1) * (detection_x_axis(2) - detection_x_axis(1)), ...
				bbox_dim_vec_px(2) * (detection_z_axis(2) - detection_z_axis(1))];
			rectangle('Position', [bbox_center_vec - (bbox_dim_vec ./ 2.0), bbox_dim_vec], 'EdgeColor', color_vec);
		end

		hold off
		xlabel('x [mm]');
		ylabel('z [mm]');

		output_img_file_name = fullfile(...
			detections_dir_name, sprintf('%06d.jpg', detection_msg.Detections(1).SourceImg.Header.Seq));
		fprintf(['Saving file, ''', output_img_file_name, '''...\n']);
		saveas(gcf, output_img_file_name);
	end
end

detection_img_sel = select(bag_fh, 'Topic', '/pt_src_loc/vis_detections');
detection_img_list = readMessages(detection_img_sel, 'DataFormat', 'struct');

for i0 = 1:length(detection_img_list)
	detection_img = rosReadImage(detection_img_list{i0});
	imagesc(detection_x_axis, detection_z_axis, detection_img);
	colormap gray
	xlabel('x [mm]');
	ylabel('z [mm]');

	output_img_file_name = fullfile(vis_detections_dir_name, sprintf('%06d.jpg', detection_img_list{i0}.Header.Seq));
	fprintf(['Saving file, ''', output_img_file_name, '''...\n']);
	saveas(gcf, output_img_file_name);
end
