function q = ta_inverse_kuka(H, mykuka)

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

h   = real(sqrt(d(4)^2 + a(3)^2));
D   = ((real(sqrt(xc^2 + yc^2))-a(1))^2 + (zc-d(1))^2 - a(2)^2 - h^2)/(2*a(2)*h);
phi = atan2(real(sqrt(1-D^2)), D);

q1 = atan2(yc, xc);
q2 = atan2(zc-d(1), real(sqrt(xc^2 + yc^2))-a(1)) + atan2(h*sin(phi), h*cos(phi)+a(2));
q3 = pi/2 - phi - atan2(a(3), d(4));

% concat q
q = [q1 q2 q3]; % joint values to reach desired wrist center

% spherical wrist orientation: theta_4, theta_5, theta_6 (ZYZ Euler angles)
% we first compute the initial wrist orientation due to previous joint values
% R_3_6 = (R_0_3)^T * Rd
H_0_3 = forward_kuka(q', mykuka);       % joint 3 pose relative to base
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