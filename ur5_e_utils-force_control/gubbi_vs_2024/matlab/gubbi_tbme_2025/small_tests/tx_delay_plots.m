load('20250205_022316.mat');

for i0 = 1:(length(TX) - 1)
	plot(TX(i0).Delay);
	ylim([-12, 12]);
	drawnow();
	pause(0.5);
end
