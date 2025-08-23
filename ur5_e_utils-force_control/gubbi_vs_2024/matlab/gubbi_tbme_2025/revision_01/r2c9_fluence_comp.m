laser_energy = [2.67e-3, 2e-3, 608.5e-6, 322e-6, 169e-6];
fiber_diameter = 1e-3;

laser_fluence = laser_energy ./ (pi .* (fiber_diameter .^ 2) / 4.0);

for i0 = 1:length(laser_energy)
	fprintf('Energy = %.2f uJ, fluence = %.2f mJ/cm^2\n', 1e6 * laser_energy(i0), 0.1 * laser_fluence(i0));
end

fprintf('\n');

laser_fluence = [184, 495];
fiber_diameter = 600e-6;
laser_energy = laser_fluence .* (pi .* (fiber_diameter .^ 2) / 4.0);

for i0 = 1:length(laser_energy)
	fprintf('Fluence = %.2f mJ/cm^2, energy = %.2f uJ\n', 0.1 *  laser_fluence(i0), 1e6 * laser_energy(i0));
end

fprintf('\n');

laser_energy = [15e-3, 68e-3];
fiber_diameter = 5e-3;

laser_fluence = laser_energy ./ (pi .* (fiber_diameter .^ 2) / 4.0);

for i0 = 1:length(laser_energy)
	fprintf('Energy = %.2f uJ, fluence = %.2f mJ/cm^2\n', 1e6 * laser_energy(i0), 0.1 * laser_fluence(i0));
end

fprintf('\n');
