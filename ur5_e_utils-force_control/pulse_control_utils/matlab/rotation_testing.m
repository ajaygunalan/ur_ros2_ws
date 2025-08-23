q1 = 0.5 * ones(1, 4);
r1 = quat2rotm(q1);
q2 = sqrt(0.5) * [1, -1, 0, 0];
q3 = quatmultiply(q2, q1);
