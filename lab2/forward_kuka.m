% returns the final homogeneous transformation matrix over the sequence of transformations
% parametrized by the jointvals row-vector
function H = forward_kuka(joint, myrobot)
%     H = myrobot.A(1:size(jointvals, 1), jointvals);
%     H = H.T;
%extract columns d, a, alpha from DH table
    joint = joint'
    size(joint)
    d = myrobot.d;
    a = myrobot.a;
    alpha = myrobot.alpha;

    %generate transformation matrix H of each frame seperately and multiply
    %them together
    H = eye(4,4);
    for i = 1:size(joint, 1)
        THETA = joint(i);
        ALPHA = alpha(i);
        %this equation is given in the lecture
        h = [cos(THETA) -sin(THETA)*cos(ALPHA) sin(THETA)*sin(ALPHA) a(i)*cos(THETA);
            sin(THETA) cos(THETA)*cos(ALPHA) -cos(THETA)*sin(ALPHA) a(i)*sin(THETA);
            0 sin(ALPHA) cos(ALPHA) d(i);
            0 0 0 1];
        H = H * h;
    end
end