%% ece470: robot modelling and control
%  lab4: motion planning for the KUKA robot
%  authors: Duo Li, Pranshu Malik, and Varun Sampat
%  date: 25 March 2022

clc;
close all;
clearvars;

%% lab init

% DH Matrix for the KUKA, forward and inverse kinematics
% in order of theta_i, d_i, a_i, alpha_i

% note values are SI units (rad, meters)
link_1 = [0  0.400    0.025      pi/2 ];
link_2 = [0  0        0.315      0    ];
link_3 = [0  0        0.035      pi/2 ];
link_4 = [0  0.365    0          -pi/2];
link_5 = [0  0        0          pi/2 ];
link_6 = [0  0.16144  -0.156     0    ];
DH     = [link_1; link_2; link_3; link_4; link_5; link_6];

% create DH table for robot with no end effector offset for force calculation
% note values are SI units (rad, meters)
link_1 = [0  0.400    0.025      pi/2 ];
link_2 = [0  0        0.315      0    ];
link_3 = [0  0        0.035      pi/2 ];
link_4 = [0  0.365    0          -pi/2];
link_5 = [0  0        0          pi/2 ];
link_6 = [0  0.16144  0          0    ]; % need to set a6 to 0 
DH_forces = [link_1; link_2; link_3; link_4; link_5; link_6];

% create the KUKA robot model and test FK and IK for a given joint config
kuka        = mykuka(DH);
kuka_forces = mykuka(DH_forces);

%% prelab: test repulsion torque values

setupobstacle_lab4prep;
tau = rep([pi/10,pi/12,pi/6,pi/2,pi/2,-pi/6], kuka_forces, prepobs{1})

%% prelab: motion planning with obstacles (part 1)

p1 = [0.620  0.375 0.050];
p2 = [0.620 -0.375 0.050];
R  = [0 0 1; 0 -1 0;1 0 0];
H1 = [R p1'; zeros(1,3) 1];
H2 = [R p2'; zeros(1,3) 1];
q1 = inverse(H1, kuka);
q2 = inverse(H2, kuka);

qref = motionplan(q1, q2, 0, 10, kuka_forces, prepobs, 1e-2, 1.0, 5000);

% visualize results
figure;
hold on;
axis([-1 1 -1 1 0 1])
view(-0.32, 0.5)
plotobstacle(prepobs);
t = linspace(0, 10, 300);
q = ppval(qref, t)';
plot(kuka, q);
hold off;

%% prelab: motion planning with obstacles (part 2)

setupobstacle;
qref = motionplan(q1, q2, 0, 10, kuka_forces, obs, 1e-2, 1.0, 5000);

% visualize results
figure;
hold on;
axis([-1 1 -1 1 0 1])
view(-0.32, 0.5)
plotobstacle(obs);
t = linspace(0, 10, 300);
q = ppval(qref, t)';
plot(kuka, q);
hold off;

%% part 4.1

z_grid = 0.045;
p0 = [0.37 -0.44 0.15];
p1 = [0.37 -0.44 z_grid];
p2 = [0.75 -0.22 0.225];
p3 = [0.62 0.35 0.225];
