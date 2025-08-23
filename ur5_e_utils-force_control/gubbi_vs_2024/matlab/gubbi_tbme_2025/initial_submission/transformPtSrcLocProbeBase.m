function [sys_ab_struct, sys_c_struct] = transformPtSrcLocProbeBase(sys_ab_struct, sys_c_struct, input_time_vec)
	sys_ab_struct.pt_src_base_link = zeros(size(sys_ab_struct.pt_src_p42v_link1));
	sys_ab_id_vec = find(...
		sys_ab_struct.pt_src_time >= input_time_vec(1) & sys_ab_struct.pt_src_time <= input_time_vec(2));

	for i0 = 1:length(sys_ab_id_vec)
		s0 = sys_ab_id_vec(i0);
		t_pt_src = sys_ab_struct.pt_src_time(s0);

		[~, t_robot_id] = min(abs(sys_ab_struct.time - t_pt_src));

		tf_quaternion = sys_ab_struct.tf_rot(t_robot_id, :);
		tf_quaternion = tf_quaternion ./ (norm(tf_quaternion) + 1e-12);
		tf_rot_mat = quat2rotm(tf_quaternion);
		tf_trans_vec = sys_ab_struct.tf_trans(t_robot_id, :);

		tf_p42v_link1_base_link = [tf_rot_mat, tf_trans_vec'; 0, 0, 0, 1];
		pt_src_p42v_link1 = [sys_ab_struct.pt_src_p42v_link1(s0, :)'; 1];
		pt_src_base_link = tf_p42v_link1_base_link * pt_src_p42v_link1;
		sys_ab_struct.pt_src_base_link(s0, :) = pt_src_base_link(1:3);
	end

	sys_c_struct.pt_src_base_link = zeros(...
		size(sys_c_struct.pt_src_p42v_link1, 1), size(sys_c_struct.pt_src_p42v_link1, 2));
	sys_c_id_vec = find(...
		sys_c_struct.pt_src_time >= input_time_vec(1) & sys_c_struct.pt_src_time <= input_time_vec(2));

	for i0 = 1:length(sys_c_id_vec)
		s0 = sys_c_id_vec(i0);
		t_pt_src = sys_c_struct.pt_src_time(s0);

		[~, t_robot_id] = min(abs(sys_c_struct.time - t_pt_src));

		tf_quaternion = sys_c_struct.tf_rot(t_robot_id, :);
		tf_quaternion = tf_quaternion ./ (norm(tf_quaternion) + 1e-12);
		tf_rot_mat = quat2rotm(tf_quaternion);
		tf_trans_vec = sys_c_struct.tf_trans(t_robot_id, :);

		tf_p42v_link1_base_link = [tf_rot_mat, tf_trans_vec'; 0, 0, 0, 1];
		pt_src_p42v_link1 = [sys_c_struct.pt_src_p42v_link1(s0, :, end)'; 1];
		pt_src_base_link = tf_p42v_link1_base_link * pt_src_p42v_link1;
		sys_c_struct.pt_src_base_link(s0, :) = pt_src_base_link(1:3);
	end
end
