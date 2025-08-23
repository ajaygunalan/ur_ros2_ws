% Visualization of Transmit Delays in Phased Array Beamforming
clear; clc; close all;

%% Parameters
c = 1540; % Speed of sound in m/s
fs = 40e6; % Sampling frequency (Hz)
array_pitch = 0.3e-3; % Element spacing (m)
num_elements = 128; % Number of elements
image_width = num_elements * array_pitch; % Imaging width (m)
image_depth = 40e-3; % Imaging depth (m)

% Define Transmit Angle
tx_angle_vec = (pi / 180) * linspace(-10, 10, 7);

for i0 = 1:length(tx_angle_vec)
	theta_t = tx_angle_vec(i0);

	%% Define Image Grid
	dx = 0.1e-3; % Lateral pixel spacing (m)
	dz = 0.1e-3; % Axial pixel spacing (m)
	x_grid = linspace(-image_width/2, image_width/2, num_elements); % Lateral positions
	z_grid = 0:dz:image_depth; % Axial positions
	[X, Z] = meshgrid(x_grid, z_grid);

	%% Compute Virtual Source Position
	X_tx = Z .* tan(theta_t); % Virtual source lateral positions

	%% Compute Transmit Delays
	T_tx = sqrt((X - X_tx).^2 + Z.^2) / c; % Transmit delays (s)

	%% Visualization
	% figure;
	imagesc(x_grid * 1e3, z_grid * 1e3, T_tx * 1e6); % Convert to microseconds
	colormap(jet);
	colorbar;
	xlabel('Lateral Position (mm)');
	ylabel('Depth (mm)');
	title(sprintf('Transmit Delays for %.1f° Steering', theta_t * 180 / pi));
	axis image;
	drawnow();
	pause(0.5);
end
