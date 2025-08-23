function [sys_ab_inference_times, sys_c_inference_times] = computeInferenceTimes(...
		sys_ab_struct, sys_c_struct, input_time_vec)
	sys_ab_id_vec = find(sys_ab_struct.time >= input_time_vec(1) & sys_ab_struct.time <= input_time_vec(2));
	sys_c_id_vec = find(sys_c_struct.time >= input_time_vec(1) & sys_c_struct.time <= input_time_vec(2));

	sys_ab_time_vec = sys_ab_struct.time(sys_ab_id_vec);
	sys_ab_inference_times = sys_ab_time_vec(2:end) - sys_ab_time_vec(1:(end-1));
	sys_c_inference_times = sys_c_struct.exe_time(sys_c_id_vec);
end
