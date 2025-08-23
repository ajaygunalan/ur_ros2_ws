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

	dt_src_loc_sos_mat = pt_src_loc_det_list{i0}.Segmentations.Results.GaussNewtOutputs.Data;
	dt_src_loc_sos_dim = [pt_src_loc_det_list{i0}.Segmentations.Results.GaussNewtOutputs.Layout.Dim(:).Size];
	dt_src_loc_sos_mat = reshape(dt_src_loc_sos_mat, dt_src_loc_sos_dim(2), dt_src_loc_sos_dim(1));
	hyp_iou_vec = Inf(size(dt_src_loc_sos_mat, 2), 1);
	seg_res_vec = Inf(size(dt_src_loc_sos_mat, 2), 1);
	hyp_res_vec = Inf(size(dt_src_loc_sos_mat, 2), 1);
	seg_hyp_dist_vec = Inf(size(dt_src_loc_sos_mat, 2), 1);
	x_axis = [1:size(dt_src_loc_sos_mat, 2)]';

	for i1 = 1:size(dt_src_loc_sos_mat, 2)
		hyp_param_mat = computeHypParameters(...
			dt_src_loc_sos_mat(1:3, i1), ...
			'speedOfSound', dt_src_loc_sos_mat(4, i1), ...
			'ampWaveWidth', dt_src_loc_sos_mat(5, i1));
		hyp_point_mat = convertParametersToPoints(hyp_param_mat);
		hyp_mask = convertPointsToMask(hyp_point_mat);

		hyp_intersect = hyp_mask & seg_mask;
		hyp_union = hyp_mask | seg_mask;
		hyp_iou_vec(i1) = sum(hyp_intersect(:)) ./ sum(hyp_union(:));
		seg_res_vec(i1) = mean(computeResiduals(seg_point_mat, hyp_param_mat), 'all');
		hyp_res_vec(i1) = sum(computeResiduals(hyp_point_mat, hyp_param_mat));

		hyp_point_mat = hyp_point_mat - 1;
		seg_hyp_dist_vec(i1) = mean(...
			abs(seg_point_mat(1:num_non_zero_cols, :) - hyp_point_mat(1:num_non_zero_cols, :)), 'all');

		if i1 == 1
			nexttile(2);
			imagesc(hyp_mask);
			colormap gray
			xlabel('x [px]');
			ylabel('z [px]');
			title('Initial Guess');
		end

		nexttile(3);
		imagesc(hyp_mask);
		colormap gray
		xlabel('x [px]');
		ylabel('z [px]');
		title('Current Guess');

		nexttile(4);
		plot(x_axis, hyp_iou_vec);
		xlabel('x [px]');
		ylabel('IoU');
		title('Segmentation IoU');

		nexttile(5);
		plot(x_axis, seg_res_vec);
		hold on
		plot(x_axis, hyp_res_vec);
		plot(x_axis, seg_hyp_dist_vec);
		xlabel('x [px]');
		ylabel('Residuals');
		title('Residuals and Distances');
		hold off

		nexttile(6);
		plot(x_axis, 1.0e3 * dt_src_loc_sos_mat(1, :)');
		xlabel('Iteration');
		ylabel('x [mm]');

		nexttile(7);
		plot(x_axis, 1.0e3 * dt_src_loc_sos_mat(2, :)');
		xlabel('Iteration');
		ylabel('y [mm]');

		nexttile(8);
		plot(x_axis, 1.0e3 * dt_src_loc_sos_mat(3, :)');
		xlabel('Iteration');
		ylabel('z [mm]');

		nexttile(9);
		plot(x_axis, dt_src_loc_sos_mat(4, :)');
		xlabel('Iteration');
		ylabel('c [m/s]');

		nexttile(10);
		plot(x_axis, 1.0e6 * dt_src_loc_sos_mat(5, :)');
		xlabel('Iteration');
		ylabel('r [um]');

		drawnow();
		pause(0.1);

		if i1 > 1
			fprintf(...
				'|x| = %.2f mm, |c| = %.1f m/s, |r| = %.2f um\n', ...
				1.0e3 * norm(dt_src_loc_sos_mat(1:3, i1) - dt_src_loc_sos_mat(1:3, i1 - 1)), ...
				abs(dt_src_loc_sos_mat(4, i1) - dt_src_loc_sos_mat(4, i1 - 1)), ...
				1.0e6 * abs(dt_src_loc_sos_mat(5, i1) - dt_src_loc_sos_mat(5, i1 - 1)));
		end
	end

	break;
end
