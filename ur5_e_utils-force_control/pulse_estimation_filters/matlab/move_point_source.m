% This script moves the point source relative to a fixed frame.
close all

num_steps = 1000;
time_step = 0.01;

t_vec = zeros(1, num_steps);
move_cmd_vec = zeros(3, num_steps);
move_freq_vec = [3, 2, 1]';
move_amp_vec = [16, 8, 30]';

pt_src_pos_w_vec = zeros(3, num_steps);
probe_pos_w_vec = zeros(3, num_steps);

figure(1);
set(gcf, 'Position', [800, 10, 1000, 800]);
tiledlayout(2, 2, 'Padding', 'loose', 'TileSpacing', 'compact');

t_lim = [-1, 11];
x_lim = [-20, 20];
y_lim = [-10, 10];
z_lim = [20, 100];

rosshutdown
rosinit

for i0 = 1:num_steps
	if i0 == 1
		x_prev = [0, 0, 60]';
	else
		x_prev = pt_src_pos_w_vec(:, i0 - 1);
	end

	t_vec(i0) = time_step * i0;

	for i1 = 1:3
		move_cmd_vec(i1, i0) = ...
			move_amp_vec(i1) * (move_freq_vec(i1) * pi / 4.0) * cos(move_freq_vec(i1) * pi * t_vec(i0) / 4.0);
	end

	prop_state_args = {'B', time_step};
	pt_src_pos_w_vec(:, i0) = linearPropagateState(x_prev, move_cmd_vec(:, i0), prop_state_args{:});

	if mod(i0, 10) == 0
		% i_start = max(1, i0 - 10);
		i_start = 1;

		nexttile(1);
		scatter(t_vec(1, i_start:i0), pt_src_pos_w_vec(1, i_start:i0));
		xlim(t_lim);
		ylim(x_lim);
		title(sprintf('Time Instance [%4d/%4d]', i0, num_steps));
		xlabel('t [s]');
		ylabel('x [mm]');

		nexttile(2);
		scatter(t_vec(1, i_start:i0), pt_src_pos_w_vec(2, i_start:i0));
		xlim(t_lim);
		ylim(y_lim);
		xlabel('t [s]');
		ylabel('y [mm]');

		nexttile(3);
		scatter(t_vec(1, i_start:i0), pt_src_pos_w_vec(3, i_start:i0));
		xlim(t_lim);
		ylim(z_lim);
		xlabel('t [s]');
		ylabel('z [mm]');

		drawnow();
		pause(0.01);
	end
end
