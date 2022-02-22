figure



Zb = [[0.8,0.85,0.95,0.95];[0.95,0.95,0.95,0.95];[0.95,0.95,0.94,00.99];[0.94,0.94,0.95,0.91]];
Zs = [[0.4,0.35,0.5,0.85];[0.45,0.4,0.55,0.61];[0.55,0.59,0.58,0.59];[0.91,0.99,0.8,0.7]];
% Zb = [[0.9,0.9,0.95,0.95];[0.95,0.95,0.95,0.95];[0.95,0.95,0.94,00.99];[0.94,0.94,0.95,0.99]];
% Zs = [[0.8,0.79,0.85,0.95];[0.85,0.7,0.8,0.75];[0.90,0.95,0.9,0.85];[0.82,0.81,0.85,0.89]];

bar3(Zb-Zs);
zlim([-1,1]);
title("Sensor 1, pike 3");
xlabel("sensor blocks along x, normalized data, units");
ylabel("sensor blocks along y, normalized data, units");
zlabel("Difference between Futek and Velostat data, normalized, unit")
