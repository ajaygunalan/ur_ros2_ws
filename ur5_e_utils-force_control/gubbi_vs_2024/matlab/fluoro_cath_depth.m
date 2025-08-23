clamp_tip = [488, 1509];
cath_tip = [1513, 1637];
bowl_left = [266, 2332];
bowl_right = [1599, 2342];

bowl_diameter = 3.25 * 25.4e-3;

cath_depth = (cath_tip(1) - clamp_tip(1)) * bowl_diameter / (bowl_right(1) - bowl_left(1));
fprintf('Fluoroscopy-based estimate of catheter depth = %.2f mm\n', 1e3 * cath_depth);
