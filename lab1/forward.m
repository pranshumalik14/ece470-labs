% returns the final homogeneous transformation matrix over the sequence of transformations
% parametrized by the jointvals row-vector
function H = forward(jointvals, myrobot)
    H = myrobot.A(1:size(jointvals, 2), jointvals);
end