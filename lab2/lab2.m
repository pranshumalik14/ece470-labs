%% ece470: robot modelling and control
%  lab2: fwd and inv kinematics for the KUKA manipulator robot
%  authors: Duo Li, Pranshu Malik, and Varun Sampat
%  date: 25 February 2022

clc;
close all;
clearvars -except udpObj;

%% Prelab : set up DH Matrix for the KUKA, forward and inverse kinematics
% in order of theta_i, d_i, a_i, alpha_i

% syms theta_1 theta_2 theta_3 theta_4 theta_5 theta_6
thetas = zeros(6); % initialize all thetas to zero

% Note a and d values are in [m] and not [mm]

link_1 = [thetas(1)  0.400    0.025        pi/2 ];
link_2 = [thetas(2)  0        0.315        0    ];
link_3 = [thetas(3)  0        0.035        pi/2 ];
link_4 = [thetas(4)  0.365    0            -pi/2];
link_5 = [thetas(5)  0        0            pi/2 ];
link_6 = [thetas(6)  0.16144  -0.29633     0    ];

DH = [link_1; link_2; link_3; link_4; link_5; link_6];

% create the KUKA robot model
kuka   = mykuka(DH);
fk_ans = forward_kuka([pi/5 pi/3 -pi/4 pi/4 pi/3 pi/4]', kuka)

ik_ans = inverse_kuka(fk_ans, kuka)
