% Units are meters
% Obstacle 1: plane
obs{1}.rho0 = 0.15;
obs{1}.h = 0.032;
obs{1}.type = 'pla';
% Obstacle 2: Cylinder
obs{2}.R = 0.1;
obs{2}.c = [0.62;0];
obs{2}.rho0 = 0.15;
obs{2}.h = 0.572;
obs{2}.type = 'cyl';
% Obstacle 3: Cylinder
obs{3}.R = 0.1;
obs{3}.c = [0.62;-0.44];
obs{3}.rho0 = 0.15;
obs{3}.h = 0.572;
obs{3}.type = 'cyl';