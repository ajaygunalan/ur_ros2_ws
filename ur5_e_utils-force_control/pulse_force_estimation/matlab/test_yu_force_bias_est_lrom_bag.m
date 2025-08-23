input_file_list = {...
	fullfile('..', 'dat', 'probe_2023-10-09-20-37-31.bag'), ...
	fullfile('..', 'dat', 'holder_2023-10-10-10-14-17.bag'), ...
	fullfile('..', 'dat', 'no_holder_2023-10-10-10-06-26.bag'), ...
	fullfile('..', 'dat', 'probe_2023-10-10-13-11-29.bag'), ...
	fullfile('..', 'dat', 'holder_2023-10-10-12-26-09.bag'), ...
	fullfile('..', 'dat', 'no_holder_2023-10-10-13-02-02.bag')};
%  input_file_list = {...
% 	fullfile('..', 'dat', 'probe_2023-10-11-15-41-00.bag')};

for i0 = 1:size(input_file_list, 2)
	input_bag_file_name = input_file_list{i0};
	fprintf(['Loading file ''', input_bag_file_name, '''...\n']);
	input_bag = rosbag(input_bag_file_name);
	ft_readings_sel = select(input_bag, 'Topic', '/yu_bias_est/ft_readings');
	robot_poses_sel = select(input_bag, 'Topic', '/yu_bias_est/robot_poses');
	ft_readings_vec = readMessages(ft_readings_sel, 'DataFormat', 'struct');
	robot_poses_vec = readMessages(robot_poses_sel, 'DataFormat', 'struct');

	num_msg = size(robot_poses_vec, 1);

	% Store all force and torque readings into matrices.
	f_s_mat = zeros(3, num_msg);
	t_s_mat = zeros(3, num_msg);

	% Initialize matrices required for bias compensation and gravity estimation.
	a9_mat = zeros(3 * num_msg, 9);
	a6_mat = zeros(3 * num_msg, 6);
	sigma_r_mat = (sqrt(3.0) / 3.0) .* eye(9);
	sigma_r_inv_mat = inv(sigma_r_mat);
	rot_e_b_mat = zeros(3, 3, num_msg);

	for i1 = 1:num_msg
		n1 = (3 * (i1 - 1)) + 1;

		% Populate the matrices of force and torque readings.
		f_s_mat(:, i1) = [ft_readings_vec{i1}.Force.X, ft_readings_vec{i1}.Force.Y, ft_readings_vec{i1}.Force.Z];
		t_s_mat(:, i1) = [ft_readings_vec{i1}.Torque.X, ft_readings_vec{i1}.Torque.Y, ft_readings_vec{i1}.Torque.Z];

		% Populate the matrix containing kinematic transformations.
		% Quaternions in MATLAB require the scalar component to be provided first, followed by the vector component.
		q0 = [...
			robot_poses_vec{i1}.Rotation.W, ...
			robot_poses_vec{i1}.Rotation.X, robot_poses_vec{i1}.Rotation.Y, robot_poses_vec{i1}.Rotation.Z];
		q0 = q0 ./ norm(q0);
		rot_e_b_mat(:, :, i1) = quat2rotm(q0);

		% Populate the A_6 and A_9 matrices required for gravity estimation.
		a9_mat(n1, 1:3) = f_s_mat(:, i1)';
		a9_mat(n1 + 1, 4:6) = f_s_mat(:, i1)';
		a9_mat(n1 + 2, 7:9) = f_s_mat(:, i1)';

		a6_mat(n1:(n1 + 2), 1:3) = -rot_e_b_mat(:, :, i1);
		a6_mat(n1:(n1 + 2), 4:6) = -eye(3);
	end

	h_mat = (eye(3 * num_msg) - (a6_mat * inv(a6_mat' * a6_mat) * a6_mat')) * a9_mat * sigma_r_inv_mat;
	hth_mat = h_mat' * h_mat;
	[v, w] = eig(hth_mat);

	for i1 = 1:size(w, 1)
		if w(i1, i1) > 0
			y_opt = v(:, i1);

			break;
		end
	end

	x9_vec = sigma_r_inv_mat * y_opt;
	rot_e_s = [x9_vec(1:3), x9_vec(4:6), x9_vec(7:9)];
	x6_vec = -inv(a6_mat' * a6_mat) * a6_mat' * a9_mat * sigma_r_inv_mat * y_opt;
	f_grav_b = x6_vec(1:3);
	f_grav_b = -f_grav_b .* sign(f_grav_b(3));
	f_bias_e = x6_vec(4:6);
	f_bias_s = rot_e_s' * f_bias_e;

	f_s_avg = mean(f_s_mat, 2);
	rot_e_b_avg = mean(rot_e_b_mat, 3);
	dt_mat = zeros(3, 3);

	for i1 = 1:num_msg
		dt_mat = dt_mat + ((squeeze(rot_e_b_mat(:, :, i1)) - rot_e_b_avg) * f_grav_b * (f_s_mat(:, i1) - f_s_avg)');
	end

	[u, s, v] = svd(dt_mat);

	if(rank(dt_mat) > 1)
		s_new = eye(3);
		s_new(3, 3) = det(u) * det(v);
		rot_s_e = u * s_new * v';
	else
		error('No solution for rotation from end effector frame to sensor frame');
	end

	% Compute the force bias in the sensor frame `S`.
	f_bias_s = f_s_avg - (rot_s_e * rot_e_b_avg * f_grav_b);

	% Compute center of gravity of tool and torque bias, both in sensor frame `S`.
	c_mat = zeros(3 * num_msg, 6);
	b_vec = zeros(3 * num_msg, 1);

	for i1 = 1:num_msg
		n1 = (3 * (i1 - 1)) + 1;
		c_mat(n1:(n1 + 2), 1:3) = -hatOperator(f_s_mat(:, i1) - f_bias_s);
		c_mat(n1:(n1 + 2), 4:6) = eye(3);
		b_vec(n1:(n1 + 2)) = t_s_mat(:, i1);
	end

	y_opt = inv(c_mat' * c_mat) * c_mat' * b_vec;
	p_grav_s = y_opt(1:3);
	t_bias_s = y_opt(4:6);

	% Estimate the contact force and torque for each F/T reading.
	f_contact_s_mat = zeros(3, num_msg);
	t_contact_s_mat = zeros(3, num_msg);

	for i1 = 1:num_msg
		f_contact_s_mat(:, i1) = f_s_mat(:, i1) - (rot_s_e * rot_e_b_mat(:, :, i1) * f_grav_b) - f_bias_s;
		t_contact_s_mat(:, i1) = ...
			t_s_mat(:, i1) - (hatOperator(p_grav_s) * rot_s_e * rot_e_b_mat(:, :, i1) * f_grav_b) - t_bias_s;
	end

	f_contact_mae = mean(abs(f_contact_s_mat), 2);
	t_contact_mae = mean(abs(t_contact_s_mat), 2);

	fprintf('Force bias in sensor frame: [%.3f, %.3f, %.3f] N\n', f_bias_s);
	fprintf('Torque bias in sensor frame: [%.3f, %.3f, %.3f] Nm\n', t_bias_s);
	fprintf('Gravity in robot base frame: [%.3f, %.3f, %.3f] N\n', f_grav_b);
	fprintf('Tool center of gravity in sensor frame: [%.3f, %.3f, %.3f] m\n', p_grav_s);
	fprintf(['rot_s_e: [\n% .2f, % .2f, % .2f\n% .2f, % .2f, % .2f\n', '% .2f, % .2f, % .2f]\n'], rot_s_e);
	fprintf('Contact force MAE: [% .2f, % .2f, % .2f]\n', f_contact_mae);
	fprintf('Contact torque MAE: [% .2f, % .2f, % .2f]\n', t_contact_mae);

	% c_mat' * c_mat
	% c_mat' * b_vec
	fprintf('\n');
end
