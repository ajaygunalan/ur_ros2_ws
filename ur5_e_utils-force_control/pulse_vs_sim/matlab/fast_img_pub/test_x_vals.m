x = 1e-3 .* [-18.2:0.1:18.2];
f_x = getLateralIndex(x);

plot(x, f_x);
xlim([-6e-4, 6e-4]);
