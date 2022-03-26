% returns the joint values to make the end-effector attain the pose given in the 
% input homogeneous transformation matrix
function q = inverse(H, mykuka)

% get robot params
d = mykuka.d;
a = mykuka.a;

% determine desrired wrist center postion (xc, yc, zc)
Rd = H(1:3, 1:3);
od = H(1:3,4);
oc = od - Rd*[a(6); 0; d(6)];

xc = oc(1);
yc = oc(2);
zc = oc(3);

% compute r (and account for offset)
r  = real(sqrt(xc^2 + yc^2)) - a(1);
s  = zc - d(1); 

% theta_1
theta_1    = atan2(yc, xc);

% theta_3
h       = real(     sqrt(d(4)^2 + a(3)^2)       );
g       = real(     sqrt( r^2 + (zc - d(1))^2)  );
D       = (h^2 - a(2)^2 - g^2   )/( -2* a(2) * g);
alpha   = atan2( real(sqrt(1-D^2))  , D         );

% theta_2
beta    = atan2((zc - d(1)), r);
theta_2 = beta + alpha;

% theta_3
D_dash = (a(2)^2 - g^2 - h^2)/(-2*h*g);
gamma = atan2(real(sqrt(1 - D_dash^2)), D_dash);

psi = atan2(a(3), d(4));
theta_3 = pi/2 - alpha - gamma - psi;

% concat q
q = [theta_1 theta_2 theta_3]; % joint values to reach desired wrist center

% spherical wrist orientation: theta_4, theta_5, theta_6 (ZYZ Euler angles)
% we first compute the initial wrist orientation due to previous joint values
% R_3_6 = (R_0_3)^T * Rd
H_0_3 = forward(q', mykuka);       % joint 3 pose relative to base
R_3_6 = H_0_3(1:3, 1:3)' * H(1:3, 1:3); % rotation from frame 3 to frame 6

% wrist angles
theta_4 = atan2(R_3_6(2,3), R_3_6(1,3));
theta_5 = atan2(real(sqrt(1- R_3_6(3,3)^2)), R_3_6(3,3));
theta_6 = atan2(R_3_6(3,2), -R_3_6(3,1));

% check for singularity
if abs(R_3_6(1,3)^2 + R_3_6(2,3)^2) < 1e-6
    theta_5 = 0;    
end

q = [q theta_4 theta_5 theta_6]; % append the wrist angles

end