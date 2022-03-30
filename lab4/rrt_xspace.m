% solves the motion planning problem for a manipulator robot using
% rapidly-exploring random tree (RRT) algorithm with obstacle avoidance
% note: this algorithm searches in the workspace coordinates (x-space)
function [q_path, x_err, tree] = rrt_xspace(q_start, q_goal, mykuka, obs, ...
    delta_x, n_iter, p, lb, ub, tol)

% check if problem is invalid: goal inside obstacle
if collision(q_goal, obs)
    q_path = [];
    x_err  = []; 
    tree   = [];
    return;
end

% get problem dimensions and goal
n = 3;
x_goal = forward(q_goal', mykuka);
x_goal = x_goal(1:3,4);

% utility variables
R_goal = [0 0 1; 0 -1 0;1 0 0];
H_new  = [R_goal zeros(3,1); zeros(1,3) 1];

% rrt node = struct('pos', [q1, ..., qn], 'parent', node);

% pre-sample within the xspace limits (lb, ub)
x_rands = rand(n, n_iter).*(ub-lb) + lb;
randidx = 1;

% initialize tree
tree = [struct('pos', q_start, 'parent', [])];

% grow/extend tree towards q_goal or q_rand at each iteration while we have
% not reached the goal
for i = 1:n_iter
    % get random point
    if rand() < p
        x_rand = x_goal;
    else
        x_rand = x_rands(:, randidx);
        randidx = randidx + 1;
    end

    % extend tree towards x_rand to get x_new
    nn_node = nearest(x_rand, tree, mykuka);
    q_near  = nn_node.pos;
    x_near  = forward(q_near', mykuka);
    x_near  = x_near(1:3,4);
    x_new   = x_near + delta_x*(x_rand - x_near)/norm(x_rand - x_near);
    
    % get q_new from x_new
    H_new(1:3,4) = x_new;
    q_new = inverse(H_new, mykuka);
    
    if all(x_new < lb) || all(x_new > ub)
        continue; % skip this iteration as q_new is outside joint limits
    elseif collision(q_new, obs)
        continue % skip this iteration as q_new is in collision
    else
        % add q_new to tree
        tree = [tree struct('pos', q_new, 'parent', nn_node)];

        % check if we have reached the goal: if so, break out of search
        if norm(wrapTo2Pi(q_new(1:5)) - wrapTo2Pi(q_goal(1:5))) < tol
            break;
        end
    end
end

% backtrack to find the path (create matrix of jointvals)
endpose = forward(tree(end).pos', mykuka);
if norm(endpose(1:3,4) - x_goal) < tol
    n_curr = tree(end);
else
    n_curr = nearest(x_goal, tree, mykuka);
end

q_path = [n_curr.pos];
while ~isempty(n_curr.parent)
    n_curr = n_curr.parent;
    q_path = [n_curr.pos; q_path];
end

% smoothen path and return with error
q_path  = [q_path(1, :); smoothdata(q_path(2:end-1, :)); q_path(end, :)];
endpose = forward(q_path(end,:)', mykuka);
x_err   = norm(endpose(1:3,4) - x_goal);

% helper functions for rrt
% checks if links are colliding with obstacles for the given jointvals, q
function colliding = collision(q, obs)
    % default padding on top of obstacle size
    padding = 0.08;
    
    % compute the current joint poses and distance from obstacle surface
    % start from robot base frame: default world frame
    oqi = zeros(3, size(q,2)+1);
    for joint = 2:size(q,2)+1
        Hqi = forward(q(1:joint-1)', mykuka);
        oqi(:, joint) = Hqi(1:3, 4);
    end

    % loop over each obstacle
    for obs_idx = 1:size(obs,2)    
        % loop over each joint
        for joint = 2:size(q,2)+1
            % get current link
            link = struct('start', oqi(:,joint-1), 'end', oqi(:,joint));
            
            % check for collisions
            if insideobs(link, obs{obs_idx}, padding)
                colliding = true;
                return;
            end
        end
    end
    
    % no collisions found
    colliding = false;
end

% checks if given link goes inside the given obstacle
function colliding = insideobs(link, obs, padding)
    for pos = link.start + linspace(0,1,10).*(link.end-link.start)
        if obs.type == "sph"
            if norm(pos-obs.c) < (obs.R + padding)
                colliding = true;
                return;
            end
        elseif obs.type == "cyl"
            if norm(pos(1:2)-obs.c) < (obs.R + padding) && ...
               pos(3) < (obs.h + padding) 
                colliding = true;
                return;
            end
        elseif obs.type == "pla" && all(pos ~= zeros(3,1)) % skip base check
            if pos(3) < obs.h % no padding for plane
                colliding = true;
                return;
            end
        end
    end
    colliding = false;
end

% returns the nearest neighbor node in tree to x_rand
% naive implementation: traverse through the entire tree
function nn_near = nearest(x_rand, tree, mykuka)
    nn_near = tree(1);
    min_dist  = Inf;

    for node_idx = 1:size(tree, 2)
        curr_pose = forward(tree(node_idx).pos', mykuka);
        curr_dist = norm(curr_pose(1:3,4) - x_rand);
        if curr_dist < min_dist
            min_dist = curr_dist;
            nn_near = tree(node_idx);
        end
    end
end

end