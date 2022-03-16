function tau = rep(q, myrobot, obs)

% Given the current q (joint vals)
% Determine the endeffector position
Oi = forward(q, myrobot);
Oi = Oi(1:3, 4);

% compute the distance between the current and desired joint positions
oq    = zeros(3, size(q,2));
Frep = zeros(size(q,2) , 3);

for i = 1:size(q,2)
    oqi        = forward(q(1:i)', myrobot);
    oq(:,i)    = oqi(1:3, 4);
    dist_from_obst = [0;0;0];
    norm_dist_from_obst = 0;
    
    if obs.type == 'cyl'
        
        norm_dist = norm(obs.c - Oi(1:2));
    
        if norm_dist > obs.R
            dist_x = (Oi(1) - obs.c(1))*(1 - obs.R/norm_dist);
            dist_y = (Oi(2) - obs.c(2))*(1 - obs.R/norm_dist);
            dist_from_obst = [dist_x; dist_y; 0];
        end
    elseif obs.type == 'sph'
        norm_dist = norm(Oi - obs.c);
        if norm_dist > obs.R
            dist_from_obst = (Oi - obs.c) * ( (-1*obs.R)/(norm_dist) + 1 );
        end
    end
    norm_dist_from_obst = norm(dist_from_obst);

    eta = 0.1;
    force = 0.0;
    if norm_dist_from_obst < obs.rho0
        force = (1/(norm_dist_from_obst^3) - 1/(obs.rho0))/(norm_dist_from_obst^3);
    end

    Frep(i, :) = force';
end

% compute torque
tau = zeros(6,1);
end