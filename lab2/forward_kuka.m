% returns the final homogeneous transformation matrix over the sequence of transformations
% parametrized by the theta row-vector
function H = forward_kuka(theta, myrobot)

H = myrobot.A(1:size(theta, 1), theta);
H = H.T;

%% alternative:
% % get robot parameters
% d     = myrobot.d;
% a     = myrobot.a;
% alpha = myrobot.alpha;
% 
% % theta should have at least one entry
% assert(size(theta,1)  > 1);
% H = trotz(theta(1)) * transl(0,0,d(1)) * transl(a(1), 0, 0) * trotx(alpha(1));
% 
% for i = 2:size(theta, 1)
%     H = H * (trotz(theta(i))*transl(0,0,d(i))*transl(a(i), 0, 0)*trotx(alpha(i)));
% end

end