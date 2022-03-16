function qref = motionplan(q0, q2, t1, t2, myrobot, obs, tol)
alpha = 0.01;
q = zeros(1, 6);
q(1, :) = q0;
while norm( q(end, 1:5) - q2(1:5) ) > tol
    tau = att(q(end, 1:6), q2, myrobot);
    temp = q(end, 1:6) + alpha *tau;
    q = [q;temp];
end

% time steps
t = linspace(t1, t2, size(q,1));

% replace q6
q6 = linspace(q0(6),q2(6),size(q,1));
q(:,6) = q6';

% apply spline
qref = spline(t, q');

end