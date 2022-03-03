function err = deltajoint(delta)

robot = mykuka_search(delta);

% calibration measurements
X1 = [0.68529  -0.00956  0.08351]';
X2 = [0.49828  -0.17624  0.08348]';
X3 = [0.49828   0.10787  0.08353]';
Q1 = [0         0.6402    0.0257    0         0.9123    0     ]';
Q2 = [-0.4517   0.9168   -0.5449   -0.4787    1.2448    0.1682]';
Q3 = [0.3293    0.9463   -0.6086    0.3466    1.2593   -0.1127]';

H1 = forward_kuka(Q1, robot);
H2 = forward_kuka(Q2, robot);
H3 = forward_kuka(Q3, robot);

% current calibration error
err = norm(H1(1:3,4)-X1) + norm(H2(1:3,4)-X2) + norm(H3(1:3,4)-X3);

end