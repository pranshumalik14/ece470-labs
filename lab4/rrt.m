% solves the motion planning problem for a manipulator robot using
% rapidly-exploring random tree (RRT) algorithm with obstacle avoidance
function q_path = rrt(q_start, q_goal, mykuka, obs, delta_q, n_iter, p, lb, ub, tol)

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
    else if collision(q_new, obs)
        continue; % skip this iteration as q_new is in collision
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
n_curr = tree(end);
q_path = [n_curr.pos];
while  n_curr.parent ~= []
    n_curr = n_curr.parent;
    q_path = [q_path; n_curr.pos];
end

% smoothen path and return
q_path = smoothdata(q_path);

%% helper functions for rrt

% checks if links are colliding with obstacles for the given jointvals, q
function colliding = check_collision(q, obs)
    for obs_idx = 1:size(obs, 2)
        % todo: check
    end
    colliding = false; % todo: add logic
end

% returns the nearest neighbor node in tree to q_rand
% naive implementation: traverse through the entire tree
function nn_near = nearest(q_rand, tree)
    q_closest = q_rand;
    min_dist  = inf;

    for node_idx = 1:size(tree, 2)
        if norm(tree(node_idx).pos - q_rand) < min_dist
            min_dist = norm(tree(node_idx).pos - q_rand)
            q_closest = tree(node_idx).pos
        end
    end

    q_near = q_closest;
end

end