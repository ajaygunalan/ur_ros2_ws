% Visualization of Plane Waves vs. Diverging Waves in Ultrasound Beamforming
clear; clc; close all;

%% Parameters
c = 1540; % Speed of sound in m/s
fs = 40e6; % Sampling frequency (Hz)
array_pitch = 0.3e-3; % Element spacing (m)
num_elements = 128; % Number of elements
image_width = num_elements * array_pitch; % Imaging width (m)
image_depth = 40e-3; % Imaging depth (m)

% Define Transmit Angle
theta_t = 10 * pi / 180; % 10 degrees in radians

%% Define Image Grid
dx = 0.1e-3; % Lateral pixel spacing (m)
dz = 0.1e-3; % Axial pixel spacing (m)
x_grid = linspace(-image_width/2, image_width/2, num_elements); % Lateral positions
z_grid = 0:dz:image_depth; % Axial positions
[X, Z] = meshgrid(x_grid, z_grid);

%% Compute Transmit Delays

% (1) Diverging Wave Model (Virtual Source Approximation)
X_tx = Z .* tan(theta_t); % Virtual source lateral positions
T_tx_diverging = sqrt((X - X_tx).^2 + Z.^2) / c; % Transmit delays (s)

% (2) Plane Wave Model
T_tx_plane = (X * sin(theta_t) + Z * cos(theta_t)) / c; % Linear delay approximation

%% Visualization

figure;

% Plot 1: Diverging Wave Delay Map
subplot(1,2,1);
imagesc(x_grid * 1e3, z_grid * 1e3, T_tx_diverging * 1e6);
colormap(jet);
colorbar;
xlabel('Lateral Position (mm)');
ylabel('Depth (mm)');
title(sprintf('Diverging Wavefront Delays (%.1f° Steering)', theta_t * 180/pi));
axis image;
hold on;

% Overlay Diverging Wavefronts
wavefront_intervals = 2e-6; % 2 microsecond spacing
wavefront_times = 0:wavefront_intervals:max(T_tx_diverging(:));
for t = wavefront_times
    contour(x_grid * 1e3, z_grid * 1e3, T_tx_diverging, [t t], 'w', 'LineWidth', 1.5);
end
hold off;

% Plot 2: Plane Wave Delay Map
subplot(1,2,2);
imagesc(x_grid * 1e3, z_grid * 1e3, T_tx_plane * 1e6);
colormap(jet);
colorbar;
xlabel('Lateral Position (mm)');
ylabel('Depth (mm)');
title(sprintf('Plane Wavefront Delays (%.1f° Steering)', theta_t * 180/pi));
axis image;
hold on;

% Overlay Plane Wavefronts
for t = wavefront_times
    contour(x_grid * 1e3, z_grid * 1e3, T_tx_plane, [t t], 'w', 'LineWidth', 1.5);
end
hold off;

