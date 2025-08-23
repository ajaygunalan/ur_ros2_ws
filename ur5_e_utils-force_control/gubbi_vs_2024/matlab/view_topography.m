addpath(genpath(fullfile('..', '..', '..', '..', '..', '..', 'beamforming_code', 'image_quality_metrics', 'utils')));
addpath(genpath(fullfile(...
	'..', '..', '..', '..', '..', '..', 'deep_learning_projects', 'point_source_localization', 'theory_3d', ...
	'functions')));
addpath(genpath(fullfile('..', '..', 'matlab_msg_gen_ros1')));

input_dir_name = fullfile('..', '..', '..', '..', 'bag');

if ~exist('input_bag_file_name', 'var')
	input_bag_file_name = '';
end

input_bag_file_name = get_file_name(...
	fullfile(input_dir_name, '*.bag'), 'Select bag file to process.', 'inputFileName', input_bag_file_name);
fprintf(['Reading file ''', input_bag_file_name, '''...\n']);
bag_fh = rosbag(input_bag_file_name);
bag_info = rosbag('info', input_bag_file_name);

topic_name = '/pt_src_loc/detections';
fprintf(['Reading messages from topic ''', topic_name, '''...\n']);
pt_src_loc_det_sel = select(bag_fh, 'Topic', topic_name);
pt_src_loc_det_list = readMessages(pt_src_loc_det_sel, 'DataFormat', 'struct');

figure(1);
set(gcf, 'Position', [8, 13, 1212, 890]);
tiledlayout(2, 5);

for i0 = 1:size(pt_src_loc_det_list, 1)
	input_img = rosReadImage(pt_src_loc_det_list{i0}.Segmentations.SourceImg);
	seg_mask = rosReadImage(pt_src_loc_det_list{i0}.Segmentations.SegMaskImg);
	nexttile(1);
	imagesc(seg_mask);
	colormap gray
	xlabel('x [px]');
	ylabel('z [px]');

	[seg_point_mat, num_non_zero_cols] = convertMaskToPoints(seg_mask);
	seg_point_mat = seg_point_mat';
	fprintf('%d non-zero cols\n', num_non_zero_cols);


