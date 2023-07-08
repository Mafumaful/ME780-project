v1_max = omega_max/beta_dif/beta_1*R_tire
v1_min = omega_min/beta_dif/beta_1*R_tire
x = v1_min:0.1:v1_max;
v2_max = omega_max/beta_dif/beta_2*R_tire
v2_min = omega_min/beta_dif/beta_2*R_tire
v3_max = omega_max/beta_dif/beta_3*R_tire
v3_min = omega_min/beta_dif/beta_3*R_tire
v4_max = omega_max/beta_dif/beta_4*R_tire
v4_min = omega_min/beta_dif/beta_4*R_tire
v5_max = omega_max/beta_dif/beta_5*R_tire
v5_min = omega_min/beta_dif/beta_5*R_tire
v6_max = omega_max/beta_dif/beta_6*R_tire
v6_min = omega_min/beta_dif/beta_6*R_tire

figure(2)
% calculate the torque of each gear
y1 = beta_dif * beta_1 * (p(1) * x1 * x1' + p(2) * x1 + p(3));
y2 = beta_dif * beta_2 * (p(1) * x2 * x2' + p(2) * x2 + p(3));
y3 = beta_dif * beta_3 * (p(1) * x3 * x3' + p(2) * x3 + p(3));
y4 = beta_dif * beta_4 * (p(1) * x4 * x4' + p(2) * x4 + p(3));
y5 = beta_dif * beta_5 * (p(1) * x5 * x5' + p(2) * x5 + p(3));
y6 = beta_dif * beta_6 * (p(1) * x6 * x6' + p(2) * x6 + p(3));

x1 = map_engine_spped / beta_dif / beta_1;
y1 = beta_dif * beta_1 / R_tire * polyval(p, map_engine_spped, ErrorEst);
x2 = map_engine_spped / beta_dif / beta_2;
y2 = beta_dif * beta_2 / R_tire * polyval(p, map_engine_spped, ErrorEst);
x3 = map_engine_spped / beta_dif / beta_3;
y3 = beta_dif * beta_3 / R_tire * polyval(p, map_engine_spped, ErrorEst);
x4 = map_engine_spped / beta_dif / beta_4;
y4 = beta_dif * beta_4 / R_tire * polyval(p, map_engine_spped, ErrorEst);
x5 = map_engine_spped / beta_dif / beta_5;
y5 = beta_dif * beta_5 / R_tire * polyval(p, map_engine_spped, ErrorEst);
x6 = map_engine_spped / beta_dif / beta_6;
y6 = beta_dif * beta_6 / R_tire * polyval(p, map_engine_spped, ErrorEst);


plot(x1, y1, 'r', x2, y2, 'g', x3, y3, 'b', x4, y4, 'c', x5, y5, 'm', x6, y6, 'k');
legend('1st gear', '2nd gear', '3rd gear', '4th gear', '5th gear', '6th gear');
title('Torque vs. Speed');
xlabel('wheel speed (rpm)');
ylabel('Torque (Nm)');

y21 = B0 + B1*x11.^2;
y22 = B0 + B1*x12.^2;
y23 = B0 + B1*x13.^2;
y24 = B0 + B1*x14.^2;
y25 = B0 + B1*x15.^2;
y26 = B0 + B1*x16.^2;

plot(x11, y21, 'r', x12, y22, 'g', x13, y23, 'b', x14, y24, 'c', x15, y25, 'm', x16, y26, 'k');
