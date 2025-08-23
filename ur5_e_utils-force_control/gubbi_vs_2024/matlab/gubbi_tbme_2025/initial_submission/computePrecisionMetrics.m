function [sys_ab_precision, sys_ab_recall, sys_ab_f1_score, sys_c_precision, sys_c_recall, sys_c_f1_score] = ...
		computePrecisionMetrics(sys_ab_struct, sys_c_struct, input_time_vec)
	sys_ab_id_vec = find(sys_ab_struct.time >= input_time_vec(1) & sys_ab_struct.time <= input_time_vec(2));
	sys_c_id_vec = find(sys_c_struct.time >= input_time_vec(1) & sys_c_struct.time <= input_time_vec(2));

	sys_ab_num_src_vec = sys_ab_struct.num_detections(sys_ab_id_vec, 1);
	sys_ab_precision = sum(sys_ab_num_src_vec > 0) / sum(sys_ab_num_src_vec);
	sys_ab_recall = sum(sys_ab_num_src_vec > 0) / length(sys_ab_num_src_vec);
	sys_ab_f1_score = 2.0 * sys_ab_precision * sys_ab_recall / (sys_ab_precision + sys_ab_recall);

	sys_c_num_src_vec = sys_c_struct.num_detections(sys_c_id_vec, 1);
	sys_c_precision = sum(sys_c_num_src_vec > 0) / sum(sys_c_num_src_vec);
	sys_c_recall = sum(sys_c_num_src_vec > 0) / length(sys_c_num_src_vec);
	sys_c_f1_score = 2.0 * sys_c_precision * sys_c_recall / (sys_c_precision + sys_c_recall);
end
