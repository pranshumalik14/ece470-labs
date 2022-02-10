% returns the joint values to make the end-effector attain the pose given in the 
% input homogeneous transformation matrix
function q = inverse(H, myrobot)

% determine desrired wrist center postion (xc, yc, zc)
d6  = myrobot.d(6);
R_d = H(1:3, 1:3);
o_d = H(1:3,4);
oc  = o_d - R_d*[0;0;d6]

xc = oc(1);
yc = oc(2);
zc = oc(3);

% compute r (and account for offset)
d2    = myrobot.d(2);
r = sqrt(xc^2 + yc^2 - d2^2)

% theta_1
beta  = atan2(yc, xc);
gamma = atan2(-d2, r);
theta_1 = beta - gamma;

% theta_3
d1 = myrobot.d(1);
d4 = myrobot.d(4);
a2 = myrobot.a(2);

L_sqrd = r^2 - (zc-d1)^2

% let D = sin(theta_3); here we choose the elbow-up configuration
D = (L_sqrd - a2^2 - d4^2)/(2*a2*d4)
theta_3 = atan2(D, -sqrt(1-D^2));

% theta_2
d1 = myrobot.d(1);
theta_2 = atan2(zc-d1, r) - atan2(-d4*cos(theta_3), a2+d4*sin(theta_3))

q = [theta_1; theta_2; theta_3;] % joint values to reach desired wrist center

% spherical wrist orientation: theta_4, theta_5, theta_6 (ZYZ Euler angles)
% we first compute the initial wrist orientation due to previous joint values
% R_3_6 = (R_0_3)^T * Rd
R_3_6   = (forward(q, myrobot).R' * H(1:3, 1:3)); % rotation from frame 3 to frame 6

% wrist angles
theta_4 = atan2(R_3_6(2,3), R_3_6(1,2));
theta_5 = atan2(sqrt(1 - R_3_6(3,3)), R_3_6(3,3));
theta_6 = atan2(R_3_6(3,2), -R_3_6(3,1));

% check for singularity
if abs(R_3_6(1,3)^2 + R_3_6(2,3)^2) < 1e-6
    theta_5 = 0;    
end

q = [q; theta_4; theta_5; theta_6]; % append the wrist angles

end