% draw straight line on paper
function q = mysegment(R, robot)

% define workspace using trajectories of x, y ,z coordinates
X_workspace = [0.62*ones(1,100); linspace(-0.1,0.1,100); -0.001*ones(1,100)];
X_baseframe = zeros(3,100);

% transform workspace coordinates into baseframe coordinates
for i = 1:100
    X_baseframe(:,i) = FrameTransformation(X_workspace(:,i));
end

%compute inverse kinematic for the joints
q = zeros(100,6);

for i = 1:100
    q(i,:) = inverse_kuka([R X_baseframe(:,i); 0 0 0 1], robot);
end

end