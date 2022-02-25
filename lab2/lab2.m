%% ece470: robot modelling and control
%  lab2: fwd and inv kinematics for the KUKA manipulator robot
%  authors: Duo Li, Pranshu Malik, and Varun Sampat
%  date: 25 February 2022

clc;
close all;
clearvars -except udpObj;

% DH Matrix for the KUKA, forward and inverse kinematics
% in order of theta_i, d_i, a_i, alpha_i
thetas = zeros(6); % initialize all thetas to zero

% note values are SI units (rad, meters)
link_1 = [thetas(1)  0.400    0.025      pi/2 ];
link_2 = [thetas(2)  0        0.315      0    ];
link_3 = [thetas(3)  0        0.035      pi/2 ];
link_4 = [thetas(4)  0.365    0          -pi/2];
link_5 = [thetas(5)  0        0          pi/2 ];
link_6 = [thetas(6)  0.16144  -0.29633   0   ];
DH     = [link_1; link_2; link_3; link_4; link_5; link_6];

% create the KUKA robot model and test FK and IK for a given joint config
kuka    = mykuka(DH);
fk_test = forward_kuka([pi/5 pi/3 -pi/4 pi/4 pi/3 pi/4]', kuka)
ik_test = inverse_kuka(fk_test, kuka)


%% part 4.3: calibrate DH table or robot model

% minimize calibration error (unconstrained optimization)
optdelta = fminunc(@deltajoint, [0 0])

% redefine robot using updated DH
myrobot = mykuka_search(optdelta)