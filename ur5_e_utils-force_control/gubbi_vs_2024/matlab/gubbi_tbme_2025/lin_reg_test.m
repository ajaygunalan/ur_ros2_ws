x = linspace(-10, 10, 101)';
a = 1;
b = 2;
c = 3;
% ax + by + c = 0
y = ((-c - (a .* x)) ./ b) + 0.1 * randn(size(x));

lin_reg_input = [x, y];
lin_reg_output = ones(size(x));

params = lin_reg_input \ lin_reg_output;
