%% ece470: robot modelling and control
%  lab2: fwd and inv kinematics for the KUKA manipulator robot
%  authors: Duo Li, Pranshu Malik, and Varun Sampat
%  date: 25 February 2022

clc;
close all;
clearvars -except udpObj;

%% lab init

% DH Matrix for the KUKA, forward and inverse kinematics
% in order of theta_i, d_i, a_i, alpha_i
thetas = zeros(6); % initialize all thetas to zero

% note values are SI units (rad, meters)
link_1 = [thetas(1)  0.400    0.025      pi/2 ];
link_2 = [thetas(2)  0        0.315      0    ];
link_3 = [thetas(3)  0        0.035      pi/2 ];
link_4 = [thetas(4)  0.365    0          -pi/2];
link_5 = [thetas(5)  0        0          pi/2 ];
link_6 = [thetas(6)  0.16144  -0.29633   0    ];
DH     = [link_1; link_2; link_3; link_4; link_5; link_6];

% create the KUKA robot model and test FK and IK for a given joint config
kuka    = mykuka(DH);
fk_test = forward_kuka([pi/5 pi/3 -pi/4 pi/4 pi/3 pi/4]', kuka)
ik_test = inverse_kuka(fk_test, kuka)


%% part 4.3: calibrate DH table or robot model

% minimize calibration error (unconstrained optimization)
optdelta = fminunc(@deltajoint, [0 0])
% we get optdelta: -0.055974432393133, -0.008527298562121

% redefine robot using updated DH
myrobot = mykuka_search(optdelta)

%% calibration check

% setHome(0.04)
R  = [0 0 1; 0 -1 0; 1 0 0];
X1 = [0.68529  -0.00956  0.08351]';
Q1 = [0 0.6402 0.0257 0 0.9123 0]';
H  = [R X1; 0 0 0 1];

% can test above to match by running:
% norm(Q1'-inverse_kuka(forward_kuka(Q1,myrobot),myrobot))

q = inverse_kuka(H, myrobot);
% setAngles(q, 0.04)

%% part 4.4: transformation to the workspace frame 

p_workspace = [0.6 0.1 0.01]';
p_baseframe = FrameTransformation(p_workspace);

H = [R p_baseframe; 0 0 0 1];
q = inverse_kuka(H, myrobot);
% setAngles(q, 0.04);

%% part 4.5: draw patterns

% straight segment
q = mysegment(R, myrobot);
% for i = 1:length(q)
%     setAngles(q(i,:), 0.04);
% end
plot(myrobot, q)

% circle
q = mycircle(R, myrobot);
% for i = 1:length(q)
%     setAngles(q(i,:), 0.04);
% end
plot(myrobot, q)

% jug picture
q = myjug(R, myrobot);
% for i = 1:length(q)
%     setAngles(q(i,:), 0.04);
% end
plot(myrobot, q)
