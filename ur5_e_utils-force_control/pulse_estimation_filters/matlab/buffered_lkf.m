addpath(genpath(fullfile('..', '..', '..', '..', '..', '..', 'beamforming_code', 'image_quality_metrics', 'utils')));
clearvars
% close all

vision_msg_path = fullfile('..', '..', 'matlab_msg_gen_ros1', 'glnxa64', 'install', 'm');

if ~exist(vision_msg_path, 'dir')
	rosgenmsg(fullfile('..', '..'));
	addpath(vision_msg_path);
	clear classes
	rehash toolboxcache
else
	addpath(vision_msg_path);
end

input_bag_file_name = fullfile('.', 'bag', 'sa_p0_pm_00_2024-07-25-14-29-26.bag');

bag_fh = rosbag(input_bag_file_name);
bag_info = rosbag('info', input_bag_file_name);

[output_vid_dir_name, input_file_base_name, ~] = fileparts(input_bag_file_name);
output_vid_file_name = fullfile(output_vid_dir_name, [input_file_base_name, '.avi']);
output_vh = VideoWriter(output_vid_file_name);
output_vh.FrameRate = 30;
t_vid_curr = 0.0;
t_vid_step = 1.0 / output_vh.FrameRate;
open(output_vh);

fprintf('Loading network detection messages from ROS bag file...\n');
detection_sel = select(bag_fh, 'Topic', '/pt_src_loc/detections');
detection_list = readMessages(detection_sel);
detection_id = 1;

detection_t_vec = zeros(length(detection_list), 1);
t_start = double(detection_list{1}.Header.Stamp.Sec) + (1e-9 * double(detection_list{1}.Header.Stamp.Nsec));

num_src_vec = zeros(length(detection_list), 1);
num_lkf_vec = zeros(length(detection_list), 1);

num_lkf = 0.0;

% TODO: Change when applicable.
delta_t = 0.5;
% proc_a_mat = [...
% 	1, 0, 0, delta_t, 0, 0, delta_t * delta_t / 2.0, 0, 0; ...
% 	0, 1, 0, 0, delta_t, 0, 0, delta_t * delta_t / 2.0, 0; ...
% 	0, 0, 1, 0, 0, delta_t, 0, 0, delta_t * delta_t / 2.0; ...
% 	0, 0, 0, 1, 0, 0, delta_t, 0, 0; ...
% 	0, 0, 0, 0, 1, 0, 0, delta_t, 0; ...
% 	0, 0, 0, 0, 0, 1, 0, 0, delta_t; ...
% 	0, 0, 0, 0, 0, 0, 1, 0, 0; ...
% 	0, 0, 0, 0, 0, 0, 0, 1, 0; ...
% 	0, 0, 0, 0, 0, 0, 0, 0, 1];
proc_a_mat = eye(3);

% proc_noise_cov_mat = (4e-6 .* eye(9)) + (4e-8 .* (1 - eye(9)));
proc_noise_cov_mat = (4e-6 .* eye(3)) + (4e-8 .* (1 - eye(3)));

% meas_h_mat = [eye(3); zeros(6, 3)];
meas_h_mat = eye(3);

meas_noise_std_vec = 1e-3 .* [0.21, 0.23, 1.42]';
meas_noise_cov_mat = 4.0 .* meas_noise_std_vec * meas_noise_std_vec';

% LKF related variables. These are ugly, but they get the job done.
lkf_state_vec = [];
x_curr_mat = [];
p_curr_mat = [];
z_curr_mat = [];
z_pred_mat = [];
s_pred_mat = [];
x_pred_mat = [];
p_pred_mat = [];
k_gain_mat = [];

num_det_vec = zeros(length(detection_list), 1);
num_lkf_vec = zeros(length(detection_list), 1);

% Chi-squared table corresponding to 99% for 3 degrees of freedom.
gate_thresh = 11.34;

figure(1);

for i0 = 1:length(detection_list)
	d0 = detection_list{i0};
	detection_t_vec(i0) = double(d0.Header.Stamp.Sec) + (1e-9 * double(d0.Header.Stamp.Nsec)) - t_start;
	src_det_id_vec = [];
	src_det_pos_mat = [];

	% Fetch the number of detections corresponding to the source class in the current message.
	for i1 = 1:length(d0.Detections)
		if(d0.Detections(i1).Results(1).Id == 0)
			src_det_id_vec = [src_det_id_vec; i1; i1];
			src_det_pos_msg = d0.Detections(i1).Results(1).Pose.Pose.Position;
			src_det_pos_mat = [...
				src_det_pos_mat, ...
				[src_det_pos_msg.X, src_det_pos_msg.Y, src_det_pos_msg.Z]', ...
				[src_det_pos_msg.X, -src_det_pos_msg.Y, src_det_pos_msg.Z]'];
		end
	end

	num_det = size(src_det_pos_mat, 2);

	for i1 = 1:num_lkf
		% Predict based on the LKF state.
		if lkf_state_vec(i1) == 0
			z_pred_mat(:, i1) = meas_h_mat * x_curr_mat(:, i1);
			s_pred_mat(:, :, i1) = (meas_h_mat * squeeze(p_curr_mat(:, :, i1)) * meas_h_mat') ...
				+ meas_noise_cov_mat;
		else
			x_pred_mat(:, i1) = proc_a_mat * x_curr_mat(:, i1);
			p_pred_mat(:, :, i1) = (proc_a_mat * squeeze(p_curr_mat(:, :, i1)) * proc_a_mat') ...
				+ proc_noise_cov_mat;
			z_pred_mat(:, i1) = meas_h_mat * x_pred_mat(:, i1);
			s_pred_mat(:, :, i1) = (meas_h_mat * squeeze(p_pred_mat(:, :, i1)) * meas_h_mat') ...
				+ meas_noise_cov_mat;
		end
	end

	if num_det > 0
		if num_lkf > 0
			% Current tracks exist, so attempt to map the detections to the tracks.
			lkf_src_det_map = Inf(num_lkf, num_det);

			for i1 = 1:num_lkf
				for i2 = 1:size(src_det_id_vec, 1)
					% Gate based on the innovation.
					innovation_vec = src_det_pos_mat(:, i2) - z_pred_mat(:, i1);
					src_dist = abs(innovation_vec' * inv(squeeze(s_pred_mat(:, :, i1))) * innovation_vec);

					% Single step gating for now, to be replaced later.
					if src_dist <= gate_thresh
						lkf_src_det_map(i1, i2) = src_dist;
					end
				end
			end

			% Iterate row-wise through the map, filtering out to a single detection (i.e., column) for each track
			% (i.e., row).
			for i1 = 1:size(lkf_src_det_map, 1)
				[~, min_i2] = min(lkf_src_det_map(i1, :));
				lkf_src_det_map(i1, (1:size(lkf_src_det_map, 2)) ~= min_i2) = Inf;
			end

			lkf_src_det_id_map = zeros(num_lkf, 1);

			% Iterate column-wise through the map, merging tracks as required.
			for i1 = 1:size(lkf_src_det_map, 2)
				% Determine the track closest to the current measurement.
				[~, min_i2] = min(lkf_src_det_map(:, i1));

				% Associate the current measurement with the selected track.
				lkf_src_det_id_map(min_i2) = i1;

				% The remaining tracks for which the current measurement is the closest can be marked for deletion.
				lkf_state_vec(isfinite(lkf_src_det_map(:, i1)) & ((1:size(lkf_src_det_map, 1))' ~= min_i2)) = -1;
			end

			% Delete the tracks marked for deletion.
			z0 = find((lkf_state_vec == -1) | (lkf_state_vec == 3));
			lkf_state_vec(z0) = [];
			x_curr_mat(:, z0) = [];
			p_curr_mat(:, :, z0) = [];
			x_pred_mat(:, z0) = [];
			p_pred_mat(:, :, z0) = [];
			z_pred_mat(:, z0) = [];
			s_pred_mat(:, :, z0) = [];
			k_gain_mat(:, :, z0) = [];

			lkf_src_det_map(z0, :) = [];
			lkf_src_det_id_map(z0) = [];

			num_lkf = numel(lkf_state_vec);
		end

		if num_lkf > 0
			for i1 = 1:num_lkf
				k_gain_mat(:, :, i1) = p_pred_mat(:, :, i1) * meas_h_mat' * inv(s_pred_mat(:, :, i1));

				if lkf_src_det_id_map(i1) > 0
					% Update with measurement.
					z_curr_mat(:, i1) = src_det_pos_mat(:, lkf_src_det_id_map(i1));
					innovation_vec = z_curr_mat(:, i1) - z_pred_mat(:, i1);
					x_curr_mat(:, i1) = x_pred_mat(:, i1) + (k_gain_mat(:, :, i1) * innovation_vec);

					if lkf_state_vec(i1) > 0
						p_curr_mat(:, :, i1) = p_pred_mat(:, :, i1) ...
							- (k_gain_mat(:, :, i1) * meas_h_mat * p_pred_mat(:, :, i1));
					else
						p_curr_mat(:, :, i1) = proc_noise_cov_mat;
					end

					lkf_state_vec(i1) = 1;
				else
					% Update without measurement
					z_curr_mat(:, i1) = z_pred_mat(:, i1);
					x_curr_mat(:, i1) = x_pred_mat(:, i1);
					p_curr_mat(:, :, i1) = p_pred_mat(:, :, i1);
					lkf_state_vec(i1) = lkf_state_vec(i1) + 1;
				end
			end
		else
			% No tracks exist, so no detection is close to any track.
			lkf_src_det_map = Inf(1, num_det);
		end

		% Iterate through the distance map between detections and tracks generated above.
		for i1 = 1:num_det
			if ~any(isfinite(lkf_src_det_map(:, i1)))
				% The current detection is not associated with any track, so initialize a new track.
				lkf_state_vec = [lkf_state_vec; 0];
				x_curr_mat = [x_curr_mat, src_det_pos_mat(:, i1)];

				% TODO: Tune this.
				p_curr_mat = cat(3, p_curr_mat, eye(3));
				x_pred_mat = [x_pred_mat, src_det_pos_mat(:, i1)];
				p_pred_mat = cat(3, p_pred_mat, proc_noise_cov_mat);
				z_pred_mat = [z_pred_mat, src_det_pos_mat(:, i1)];
				s_pred_mat = cat(3, s_pred_mat, eye(3));
				k_gain_mat = cat(3, k_gain_mat, ones(3));
			end
		end
	elseif num_lkf > 0
		% No detections were found, so update the existing tracks if any accordingly.
		z0 = find((lkf_state_vec == -1) | (lkf_state_vec == 0) | (lkf_state_vec == 3));
		lkf_state_vec(z0) = [];
		x_curr_mat(:, z0) = [];
		p_curr_mat(:, :, z0) = [];
		x_pred_mat(:, z0) = [];
		p_pred_mat(:, :, z0) = [];
		z_pred_mat(:, z0) = [];
		s_pred_mat(:, :, z0) = [];
		k_gain_mat(:, :, z0) = [];

		num_lkf = numel(lkf_state_vec);

		for i1 = 1:num_lkf
			% Update the remaining tracks without measurement
			z_curr_mat(:, i1) = z_pred_mat(:, i1);
			x_curr_mat(:, i1) = x_pred_mat(:, i1);
			p_curr_mat(:, :, i1) = p_pred_mat(:, :, i1);
			lkf_state_vec(i1) = lkf_state_vec(i1) + 1;
		end
	end

	num_lkf = numel(lkf_state_vec);
	num_lkf_vec(i0) = num_lkf;
	num_det_vec(i0) = num_det / 2;

	while detection_t_vec(i0) - t_vid_curr >= t_vid_step
		drawnow();
		vid_frame = getframe(gcf);
		writeVideo(output_vh, vid_frame.cdata);
		t_vid_curr = t_vid_curr + t_vid_step;
	end

	if ~isempty(src_det_pos_mat)
		scatter(1e3 .* src_det_pos_mat(1, :), 1e3 .* src_det_pos_mat(2, :), 'o');
	end

	hold on

	if ~isempty(x_curr_mat)
		scatter(1e3 .* x_curr_mat(1, :), 1e3 .* x_curr_mat(2, :), 'x');
	end

	hold off
	xlabel('x [mm]');
	ylabel('y [mm]');
	xlim([-80, 80]);
	ylim([-25, 25]);
end

fprintf(['Closing file ''', output_vid_file_name, '''...\n']);
close(output_vh);

figure(2);
plot(num_det_vec, 'LineWidth', 3, 'DisplayName', 'Detections');
hold on;
plot(num_lkf_vec, 'DisplayName', 'Tracks');
hold off
xlabel('Iteration');
ylabel('Count');
legend();
