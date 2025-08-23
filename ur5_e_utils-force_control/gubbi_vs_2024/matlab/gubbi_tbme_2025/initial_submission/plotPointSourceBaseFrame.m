function plotPointSourceProbeFrame(sys_ab_struct, sys_c_struct, input_time_vec, output_img_file_base_name, varargin)
	p = inputParser();

	addParameter(p, 'outputImgDirName', fullfile('.', 'img'), @ischar);

	parse(p, varargin{:});

	output_img_dir_name = p.Results.outputImgDirName;

	sys_ab_id_vec = find(sys_ab_struct.pt_src_time >= input_time_vec(1) & sys_ab_struct.pt_src_time <= input_time_vec(2));
	sys_c_id_vec = find(sys_c_struct.pt_src_time >= input_time_vec(1) & sys_c_struct.pt_src_time <= input_time_vec(2));

	pt_src_coords_str_list = {'x', 'y', 'z'};

	for i0 = 1:length(pt_src_coords_str_list)
		scatter(...
			sys_ab_struct.pt_src_time(sys_ab_id_vec), 1e3 .* sys_ab_struct.pt_src_base_link(sys_ab_id_vec, i0), ...
			'.', 'DisplayName', 'System A/B');
		hold on
		scatter(...
			sys_c_struct.pt_src_time(sys_c_id_vec), 1e3 .* sys_c_struct.pt_src_base_link(sys_c_id_vec, i0), ...
			'.', 'DisplayName', 'System C');
		hold off
		legend('location', 'southeast');

		xlabel('Time [s]');
		ylabel('Point Source Position [mm]');

		output_img_file_name = fullfile(...
			output_img_dir_name, ['raw_pt_src_', pt_src_coords_str_list{i0},  output_img_file_base_name]);
		fprintf(['Saving file ''', output_img_file_name, '''...\n']);
		saveas(gcf, output_img_file_name);
	end
end

