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

qref = motionplan(q1, q2, 0, 10, kuka_forces, prepobs, 1e-2, 1.0, 5000, ...
    0.013, 0.01);

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
obs{3}.c = [0.62;-0.640];
qref = motionplan(q1, q2, 0, 10, kuka_forces, obs, 1e-2, 3.5, 15000, ...
    0.013, 0.01);

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

setupobstacle;

z_grid = 0.045;
p0 = [0.37 -0.44 0.15];
p1 = [0.37 -0.44 z_grid];
p2 = [0.75 -0.22 0.225];
p3 = [0.62 0.35 0.225];

R  = [0 0 1; 0 -1 0;1 0 0];
H0 = [R p0'; zeros(1,3) 1];
H1 = [R p1'; zeros(1,3) 1];
H2 = [R p2'; zeros(1,3) 1];
H3 = [R p3'; zeros(1,3) 1];

q0 = inverse(H0, kuka);
q1 = inverse(H1, kuka);
q2 = inverse(H2, kuka);
q3 = inverse(H3, kuka);

qref = motionplan(q0, q1, 0, 10, kuka_forces, obs, 3e-2, 0.2, 1000, ...
    0.03, 0.01);
t = linspace(0, 10, 300);
q1_follow = ppval(qref, t)';

qref = motionplan(q1_follow(end, :), q2, 10, 20, kuka_forces, obs, 3e-2, ...
    0.5, 1000, 0.01, 0.01);
t = linspace(10, 20, 300);
q2_follow = ppval(qref, t)';

qref = motionplan(q2_follow(end, :), q3, 20, 30, kuka_forces, obs, 3e-2, ...
    1, 5000, 0.01, 0.01);
t = linspace(20, 30, 300);
q3_follow = ppval(qref, t)';

% visualize results
figure;
hold on;
axis([-1 1 -1 1 0 1])
view(-0.32, 0.5)
plotobstacle(obs);
scatter3(p1(1), p1(2), p1(3));
scatter3(p2(1), p2(2), p2(3));
scatter3(p3(1), p3(2), p3(3));
plot(kuka, q1_follow);
plot(kuka, q2_follow);
plot(kuka, q3_follow);
hold off;

%% follow commands on real robot

% find physical position for the obstacle
% obs 1
cylinder_1_pose = [obs{2}.c; 0.1];
cylinder_2_pose = [obs{3}.c; 0.1];

R  = [0 0 1; 0 -1 0;1 0 0];
H_cyn_1 = [R cylinder_1_pose; zeros(1,3) 1];
H_cyn_2 = [R cylinder_2_pose; zeros(1,3) 1];

q_cyn_1 = inverse(H_cyn_1, kuka);
q_cyn_2 = inverse(H_cyn_2, kuka);

max_vel = 0.04;
min_vel = 0.02;

getHome(max_vel);
setGripper(0);
setAngles(q0, min_vel);

for t = 1:size(q1_follow, 1)
    setAngles(q1_follow(t, :), min_vel)
end

% reached, set gripper to 1
setGripper(1);

for t = 1:size(q2_follow, 1)
    setAngles(q2_follow(t, :), min_vel);
end

for t = 1:size(q3_follow, 1)
    setAngles(q3_follow(t, :), min_vel);
end

setGripper(0);
setHome(max_vel);

%% rrt in joint space (refer to video recorded)

lb  = [0 -pi/2   0    0  0    -pi/2];
ub  = [pi/2 pi/2 2*pi pi 2*pi  pi/2];
qtol = 5e-2;
[q_path, q_err, tree] = ...
    rrt_qspace(q1, q2, kuka, prepobs, 0.03, 1000, 0.5, lb, ub, qtol);

% visualize
figure;
hold on;
axis([-1 1 -1 1 0 1])
view(-0.32, 0.5)
plotobstacle(prepobs);
if q_err > qtol
    % visualize exploration tree
    for i = 1:fix(size(tree,2)/100):size(tree,2)
        plot(kuka, tree(i).pos);
    end
else
    % visualize result path
    plot(kuka, q_path);
end
hold off;
close;

%% rrt planning in joint space (sim: for the prelab)

p1 = [0.620  0.375 0.050];
p2 = [0.620 -0.375 0.050];
R  = [0 0 1; 0 -1 0;1 0 0];
H1 = [R p1'; zeros(1,3) 1];
H2 = [R p2'; zeros(1,3) 1];
q1 = inverse(H1, kuka);
q2 = inverse(H2, kuka);

lb  = [0 -pi/2   0    0  0    -pi/2];
ub  = [pi/2 pi/2 2*pi pi 2*pi  pi/2];
qtol = 5e-2;
[q_path, q_err, tree] = ...
    rrt_qspace(q1, q2, kuka, prepobs, 0.03, 1000, 0.5, lb, ub, qtol);

% visualize
figure;
hold on;
axis([-1 1 -1 1 0 1])
view(-0.32, 0.5)
plotobstacle(prepobs);
if q_err > qtol
    % visualize exploration tree
    for i = 1:fix(size(tree,2)/100):size(tree,2)
        plot(kuka, tree(i).pos);
    end
else
    % visualize result path
    plot(kuka, q_path);
end
hold off;

%% rrt planning in workspace (slow convergence)

% xlb  = [0; 0; 0];
% xub  = [1; 1; 1];
% xtol = 5e-2;
% [xsearch_q_path, x_err, xsearch_tree] = ...
%     rrt_xspace(q1, q2, kuka, prepobs, 0.05, 5000, 0.5, xlb, xub, xtol);
% 
% % visualize result path
% figure;
% hold on;
% axis([-1 1 -1 1 0 1])
% view(-0.32, 0.5)
% plotobstacle(prepobs);
% plot(kuka, xsearch_q_path);
% hold off;
% 
% % visualize exploration tree
% if x_err > xtol
%     figure;
%     hold on;
%     axis([-1 1 -1 1 0 1])
%     view(-0.32, 0.5)
%     plotobstacle(prepobs);
%     for i = 1:fix(size(xsearch_tree,2)/10):size(xsearch_tree,2)
%         plot(kuka, xsearch_tree(i).pos);
%     end
%     hold off;
% end
