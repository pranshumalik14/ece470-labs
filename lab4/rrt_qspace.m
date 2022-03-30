% solves the motion planning problem for a manipulator robot using
% rapidly-exploring random tree (RRT) algorithm with obstacle avoidance
% note: this algorithm searches in the joint space
function [q_path, q_err, tree] = rrt_qspace(q_start, q_goal, mykuka, obs, ...
    delta_q, n_iter, p, lb, ub, tol)

% check if problem is invalid: goal inside obstacle
if collision(q_goal, obs)
    q_path = [];
    q_err  = []; 
    tree   = [];
    return;
end

% get problem dimensions    
n = size(mykuka.links, 2);

% rrt node = struct('pos', [q1, ..., qn], 'parent', node);

% pre-sample within the joint limits (lb, ub)
q_rands = rand(n_iter, n).*(ub-lb) + lb;
randidx = 1;

% initialize tree
tree = [struct('pos', q_start, 'parent', [])];

% grow/extend tree towards q_goal or q_rand at each iteration while we have
% not reached the goal
for i = 1:n_iter
    % get random point
    if rand() < p
        q_rand = q_goal;
    else
        q_rand = q_rands(randidx, :);
        randidx = randidx + 1;
    end

    % extend tree towards q_rand
    nn_node = nearest(q_rand, tree);
    q_near  = nn_node.pos;
    q_new   = q_near + delta_q*(q_rand - q_near)/norm(q_rand - q_near);
    
    if all(q_new < lb) || all(q_new > ub)
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
if norm(wrapTo2Pi(tree(end).pos(1:5)) - wrapTo2Pi(q_goal(1:5))) < tol
    n_curr = tree(end);
else
    n_curr = nearest(q_goal, tree);
end

q_path = [n_curr.pos];
while ~isempty(n_curr.parent)
    n_curr = n_curr.parent;
    q_path = [n_curr.pos; q_path];
end

% smoothen path and return with error
q_path = [q_path(1, :); smoothdata(q_path(2:end-1, :)); q_path(end, :)];
q_err  = norm(wrapTo2Pi(q_path(end, 1:5)) - wrapTo2Pi(q_goal(1:5)));

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

% returns the nearest neighbor node in tree to q_rand
% naive implementation: traverse through the entire tree
function nn_near = nearest(q_rand, tree)
    nn_near = tree(1);
    min_dist  = Inf;

    for node_idx = 1:size(tree, 2)
        if norm(tree(node_idx).pos - q_rand) < min_dist
            min_dist = norm(tree(node_idx).pos - q_rand);
            nn_near = tree(node_idx);
        end
    end
end

end