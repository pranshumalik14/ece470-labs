function q = mycircle(R, robot)

r = 0.05;
x = 0.620;
y = 0.0;

theta = 0:pi/50:2*pi;
xunit = r * cos(theta) + x;
yunit = r * sin(theta) + y;
zunit = -0.001 * ones(1, length(theta));

X_workspace = [xunit; yunit; zunit];
X_baseframe = zeros(3, length(theta));
    
for i = 1:length(theta)
    X_baseframe(:,i) = FrameTransformation(X_workspace(:,i));
end

q = zeros(length(theta), 6);

for i = 1:length(theta)
    q(i,:) = inverse_kuka([R X_baseframe(:,i); 0 0 0 1], robot);
end

end