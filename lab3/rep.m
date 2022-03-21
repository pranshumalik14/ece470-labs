function tau = rep(q, myrobot, obs)
% obs.R = obs.R/100;
% obs.rho0 = obs.rho0/100;
% obs.c = obs.c/100;

Frep = zeros(3, size(q,2));

% compute the distance between the current and desired joint positions
Hqi    = zeros(4, 4, size(q,2)+1); 

Hqi(:,:,1) = trotz(0); % start from robot base frame: default world frame
for i = 1:size(q,2)
    Hqi(:,:,i+1) = forward(q(1:i)', myrobot);
end

for i = 1:size(q,2)

    Oi = Hqi(1:3,4,i+1);
    dist_from_obst = [0;0;0];
    norm_dist_from_obst = 0;
    
    if obs.type == 'cyl'
        
        norm_dist = norm(obs.c(1:2) - Oi(1:2));
    
        if norm_dist > obs.R
            dist_x = (Oi(1) - obs.c(1))*(1 - obs.R/norm_dist);
            dist_y = (Oi(2) - obs.c(2))*(1 - obs.R/norm_dist);
            dist_from_obst = [dist_x; dist_y; 0];
        end
    elseif obs.type == 'sph'
        norm_dist = norm(Oi - obs.c);
        if norm_dist > obs.R
             dist_from_obst = (Oi - obs.c) * (1 - obs.R/norm_dist);
%               dist_from_obst = Oi - (obs.R * (Oi - obs.c)/norm_dist + obs.c);
        end
    end

    norm_dist_from_obst = norm(dist_from_obst);
    grad_rho = dist_from_obst/norm_dist_from_obst;

    eta = 1;
    force = [0;0;0];

    if norm_dist_from_obst < obs.rho0
%         force = eta*(1/(norm_dist_from_obst^3) - 1/(obs.rho0))*dist_from_obst/(norm_dist_from_obst^3);
          force = eta * ( 1/(norm_dist_from_obst) - 1/(obs.rho0) ) * (1/(norm_dist_from_obst^2)) * grad_rho;
    end

    Frep(:, i) = force;
end
% Frep = [0 0 0 -106.4269 -106.4269 -106.4269; 0 0 0 -3.694 -3.694 -3.694; 0 0 0 0 0 0]
% compute torque
tau = zeros(6,1);

for i = 1:size(q,2)
    % compute the geometric translational velocity jacobian at current joint config
    % Jvn = myrobot.jacob0(q, 'trans'); % rtb toolbox inbuilt (gives final jacobian)
    Jvi = jacobvi(q(1:i), Hqi, myrobot);
    tau = tau + ([Jvi zeros(3, size(q,2)-i)]' * Frep(:,i));
end

if norm(tau) ~= 0
    tau = tau/norm(tau);
end

tau = tau';

end