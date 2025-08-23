function x_id = getLateralIndex(x, trans_pitch, sim_grid_res)
	x_id = mod(round((x + 2e-4) / sim_grid_res), round(trans_pitch / sim_grid_res)) + 1;
end
