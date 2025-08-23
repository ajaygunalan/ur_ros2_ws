function [sys_ab_robot_dist_max, sys_ab_robot_vert_max] = computeRobotDistances(sys_ab_struct, input_time_vec)
	sys_ab_id_vec = find(sys_ab_struct.time >= input_time_vec(1) & sys_ab_struct.time <= input_time_vec(2));
	sys_ab_robot_dist_vec = sqrt(...
		sum((sys_ab_struct.tf_trans(sys_ab_id_vec, 1:2) - sys_ab_struct.tf_trans(sys_ab_id_vec(1), 1:2)) .^ 2, 2));

	sys_ab_robot_dist_max = max(sys_ab_robot_dist_vec);

	sys_ab_robot_vert_vec = (sys_ab_struct.tf_trans(sys_ab_id_vec, 3) - sys_ab_struct.tf_trans(sys_ab_id_vec(1), 3));
	sys_ab_robot_vert_max = max(sys_ab_robot_vert_vec) - min(sys_ab_robot_vert_vec);
end
