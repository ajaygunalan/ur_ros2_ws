function [mtlkf_time_vec, mtlkf_state_vec] = matchLkfTimes(sys_ab_struct, mtlkf_struct, input_time_vec)
	mtlkf_state_vec = Inf(size(sys_ab_struct.time));
	sys_ab_time_id = 1;

	for i0 = 1:length(mtlkf_struct.time)
		t0 = mtlkf_struct.time(i0);

		if sys_ab_time_id >= length(sys_ab_struct.time)
			break;
        elseif sys_ab_struct.time(sys_ab_time_id) > t0
            continue;
		else
			while sys_ab_time_id < length(sys_ab_struct.time)
				if sys_ab_struct.time(sys_ab_time_id) <= t0 && sys_ab_struct.time(sys_ab_time_id + 1) > t0
					break;
				end

				sys_ab_time_id = sys_ab_time_id + 1;
			end

			mtlkf_state_vec(sys_ab_time_id) = mtlkf_struct.validity(i0);
		end
	end

	sys_ab_id_vec = find(sys_ab_struct.time >= input_time_vec(1) & sys_ab_struct.time <= input_time_vec(2));
	mtlkf_state_vec = mtlkf_state_vec(sys_ab_id_vec);
	mtlkf_time_vec = sys_ab_struct.time(sys_ab_id_vec);
end
