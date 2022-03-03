function q = myjug(R, robot)

data  = xlsread('jug.xlsx');
xdata = 0.001 * (550 + 10*data(:,1));
ydata = 0.001 * 10 * data(:,2);
zdata = 0.001 * -ones(length(data),1);

X_workspace = [xdata ydata zdata]';
X_baseframe = zeros(3, length(data));

for i = 1:length(data)
    X_baseframe(:,i) = FrameTransformation(X_workspace(:,i));
end

q = zeros(length(data), 6);

for i = 1:length(data)
    q(i,:) = inverse_kuka([R X_baseframe(:,i); 0 0 0 1], robot);
end

end