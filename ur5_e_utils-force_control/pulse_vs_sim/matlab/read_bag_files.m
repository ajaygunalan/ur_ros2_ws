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

bag_fh = rosbag(bag_file_name);
bag_info = rosbag('info', bag_file_name);
