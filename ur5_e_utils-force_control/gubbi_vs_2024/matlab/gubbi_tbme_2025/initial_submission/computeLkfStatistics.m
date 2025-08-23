function [lkf_validity_histcounts] = computeLkfStatistics(sys_ab_struct, mtlkf_struct, input_time_vec, varargin)
	p = inputParser();

	addParameter(p, 'lkfHistBinEdges', [-0.5:1:5.5], @ischar);

	parse(p, varargin{:});

	% lkf_id_vec = find(mtlkf_struct.time >= input_time_vec(1) & mtlkf_struct.time <= input_time_vec(2));

	[lkf_time_vec, lkf_state_vec] = matchLkfTimes(sys_ab_struct, mtlkf_struct, input_time_vec);
	lkf_state_vec(~isfinite(lkf_state_vec)) = 5;
	% lkf_validity_histcounts = histcounts(mtlkf_struct.validity(lkf_id_vec), p.Results.lkfHistBinEdges);
	lkf_validity_histcounts = histcounts(lkf_state_vec, p.Results.lkfHistBinEdges);

	lkf_validity_histcounts(5) = sum(lkf_validity_histcounts(5:6));
	lkf_validity_histcounts = lkf_validity_histcounts([4, 1, 2, 3, 5]);
end
