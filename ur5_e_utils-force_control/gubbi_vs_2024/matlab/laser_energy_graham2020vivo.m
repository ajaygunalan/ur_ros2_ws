fiber_diameter = 1e-3;
laser_energy = 2.8e-3;

fiber_area = (pi * (fiber_diameter ^ 2)) / 4.0;
laser_fluence = laser_energy / fiber_area;

fprintf(...
	'Laser energy = %.2f mJ\nFiber diameter = %.2f mm\nLaser fluence = %.2f mJ/cm^2\n', ...
	1.0e3 * laser_energy, 1.0e3 * fiber_diameter, 1.0e-1 * laser_fluence);

new_fiber_diameter = 600e-6;
new_fiber_area = (pi * (new_fiber_diameter ^ 2)) / 4.0;
new_laser_energy = laser_fluence * new_fiber_area;

fprintf(...
	'New fiber diameter = %.2f mm\nNew laser energy = %.2f mJ\n', ...
	1.0e3 * new_fiber_diameter, 1.0e3 * new_laser_energy);
