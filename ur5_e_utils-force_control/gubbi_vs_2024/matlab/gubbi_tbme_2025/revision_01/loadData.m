function [sys_ab_struct, sys_c_struct, mtlkf_struct, netft_struct] = loadData(input_h5_dir_name, varargin)
	p = inputParser();

	addParameter(p, 'realTimeSystem', 'a', @ischar);

	p.KeepUnmatched = true;
	parse(p, varargin{:});

	real_time_system = p.Results.realTimeSystem;

	fprintf(['Loading directory ''', input_h5_dir_name, '''...\n']);
	input_file_list = dir(input_h5_dir_name);
	sys_ab_h5_file_list = {};
	sys_ab_h5_file_id = 1;
	sys_c_h5_file_list = {};
	sys_c_h5_file_id = 1;

	for i0 = 1:length(input_file_list)
		if startsWith(input_file_list(i0).name, 'robot_poses')
			robot_poses_h5_file_name = fullfile(input_h5_dir_name, input_file_list(i0).name);
		elseif startsWith(input_file_list(i0).name, 'sys_ab')
			sys_ab_h5_file_list{sys_ab_h5_file_id} = fullfile(input_h5_dir_name, input_file_list(i0).name);
			sys_ab_h5_file_id = sys_ab_h5_file_id + 1;
		elseif startsWith(input_file_list(i0).name, 'sys_c')
			sys_c_h5_file_list{sys_c_h5_file_id} = fullfile(input_h5_dir_name, input_file_list(i0).name);
			sys_c_h5_file_id = sys_c_h5_file_id + 1;
		end
	end

	sys_ab_h5_file_list = natsortfiles(sys_ab_h5_file_list);
	sys_c_h5_file_list = natsortfiles(sys_c_h5_file_list);

	sys_ab_struct = struct(...
		'time', h5read(robot_poses_h5_file_name, '/sys_ab/time')', ...
		'robot_program_running', h5read(robot_poses_h5_file_name, '/sys_ab/robot_program_running')', ...
		'num_detections', h5read(robot_poses_h5_file_name, '/sys_ab/num_detections')', ...
		'tf_trans', h5read(robot_poses_h5_file_name, '/sys_ab/tf/p42v_link1/trans')', ...
		'tf_rot', h5read(robot_poses_h5_file_name, '/sys_ab/tf/p42v_link1/rot')');

	sys_ab_struct.pt_src_time = [];
	sys_ab_struct.pt_src_p42v_link1 = [];
	sys_ab_struct.pt_src_base_link = [];

	for i0 = 1:length(sys_ab_h5_file_list)
		sys_ab_h5_file_name = sys_ab_h5_file_list{i0};

		pt_src_p42v_link1 = h5read(sys_ab_h5_file_name, '/pt_src/p42v_link1');

		sys_ab_struct.pt_src_p42v_link1 = [sys_ab_struct.pt_src_p42v_link1; pt_src_p42v_link1'];

		time_val = h5read(sys_ab_h5_file_name, '/pt_src/time');
		time_vec = repelem(time_val, size(pt_src_p42v_link1, 2), 1);
		sys_ab_struct.pt_src_time = [sys_ab_struct.pt_src_time; time_vec];

		tf_probe_base = h5read(sys_ab_h5_file_name, '/tf/p42v_link1')';
		pt_src_base_link = tf_probe_base * [pt_src_p42v_link1; ones(1, size(pt_src_p42v_link1, 2))];
		sys_ab_struct.pt_src_base_link = [sys_ab_struct.pt_src_base_link; pt_src_base_link(1:3, :)'];
	end

	sys_c_struct = struct(...
		'time', h5read(robot_poses_h5_file_name, '/sys_c/time')', ...
		'robot_program_running', h5read(robot_poses_h5_file_name, '/sys_c/robot_program_running')', ...
		'num_detections', h5read(robot_poses_h5_file_name, '/sys_c/num_detections')', ...
		'exe_time', h5read(robot_poses_h5_file_name, '/sys_c/exe_time')', ...
		'tf_trans', h5read(robot_poses_h5_file_name, '/sys_c/tf/p42v_link1/trans')', ...
		'tf_rot', h5read(robot_poses_h5_file_name, '/sys_c/tf/p42v_link1/rot')');

	sys_c_struct.pt_src_time = [];
	sys_c_struct.pt_src_p42v_link1 = [];
	sys_c_struct.pt_src_base_link = [];

	for i0 = 1:length(sys_c_h5_file_list)
		sys_c_h5_file_name = sys_c_h5_file_list{i0};
		pt_src_p42v_link1 = permute(h5read(sys_c_h5_file_name, '/pt_src/p42v_link1'), [1, 3, 2]);
		sys_c_struct.pt_src_p42v_link1 = [sys_c_struct.pt_src_p42v_link1; permute(pt_src_p42v_link1, [2, 1, 3])];

		time_val = h5read(sys_c_h5_file_name, '/pt_src/time');
		time_vec = repelem(time_val, size(pt_src_p42v_link1, 2), 1);
		sys_c_struct.pt_src_time = [sys_c_struct.pt_src_time; time_vec];

		tf_probe_base = h5read(sys_c_h5_file_name, '/tf/p42v_link1')';
		pt_src_base_link = tf_probe_base * [pt_src_p42v_link1(:, :, end); ones(1, size(pt_src_p42v_link1, 2))];
		sys_c_struct.pt_src_base_link = [sys_c_struct.pt_src_base_link; pt_src_base_link(1:3, :)'];
	end

	mtlkf_struct = struct(...
		'time', h5read(robot_poses_h5_file_name, '/multi_track_lkf/time')', ...
		'num_filters', h5read(robot_poses_h5_file_name, '/multi_track_lkf/num_filters')', ...
		'validity', h5read(robot_poses_h5_file_name, '/multi_track_lkf/validity')');
	netft_struct = struct(...
		'time', h5read(robot_poses_h5_file_name, '/netft/time')', ...
		'proc_probe', h5read(robot_poses_h5_file_name, '/netft/proc_probe')');
end
