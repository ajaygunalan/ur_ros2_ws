laser_fluence = 25.2 * 1e-3 / ((1e-2)^2); % J/m^2
fiber_diameter = 1e-3;

laser_energy = laser_fluence * pi * (fiber_diameter ^ 2) / 4.0;
fprintf('Fluence = %.2f mJ/cm^2, Energy = %.2f uJ\n', 0.1 * laser_fluence, 1e6 * laser_energy);
