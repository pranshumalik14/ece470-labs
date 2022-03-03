% returns the kuka robot with a parametric DH table
function kuka = mykuka_search(delta)

% DH Matrix for the KUKA, forward and inverse kinematics
% in order of theta_i, d_i, a_i, alpha_i
thetas = zeros(6); % initialize all thetas to zero

% note values are SI units (rad, meters)
link_1 = [thetas(1)  0.400             0.025              pi/2 ];
link_2 = [thetas(2)  0                 0.315              0    ];
link_3 = [thetas(3)  0                 0.035              pi/2 ];
link_4 = [thetas(4)  0.365             0                  -pi/2];
link_5 = [thetas(5)  0                 0                  pi/2 ];
link_6 = [thetas(6)  0.16144+delta(2) -0.29633-delta(1)   0    ];
DH     = [link_1; link_2; link_3; link_4; link_5; link_6];

kuka = mykuka(DH);

end