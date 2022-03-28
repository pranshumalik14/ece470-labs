% solves the motion planning problem for a manipulator robot using
% rapidly-exploring random tree (RRT) algorithm with obstacle avoidance
function q_path = rrt(obs, q_start, q_goal, mykuka, delta_q, n_iter, p, lb, ub)

% get problem dimensions    
n = size(mykuka.links, 2);

% rrt node = struct('pos', [q1, ..., qn], 'parent', node);

% pre-sample within the joint limits (lb, ub)
q_rands = rand(n_iter, n).*(ub-lb) + lb;

% initialize tree
tree = [struct('pos', q_start, 'parent', [])]; % depends on how kdtree needs it

% grow/extend tree towards q_goal or q_rand at each iteration while we have
% not reached the goal
for i = 1:n_iter
    q_rand = q_start + delta_q*rand(1, mykuka.n);
    q_rand = p
    
    q_near = nearest(q_rand, q_start, mykuka);
    q_new  = steer(q_rand, q_near, mykuka);
    if collision(q_new, obs)
        continue;
    else 
        tree = [tree struct('pos', q_new, 'parent', q_near)];
        if % reached goal
            break;
        end
    end
end

% backtrack to find the path

function colliding = check_collision(q, obs)
    for obs_idx = 1 : len(obs)
end

% Should return the nearest q in tree to q_rand
% Returns q_near = q_rand if tree is empty
function q_near = nearest(q_rand, tree)
% naive implementation -> traverse through the entire tree
q_closest = q_rand;
min_dist = inf;
for node_idx = 1:size(tree, 2)
    if norm(tree(node_idx).pos - q_rand) < min_dist
        min_dist = norm(tree(node_idx).pos - q_rand)
        q_closest = tree(node_idx).pos
    end
end

q_near = q_closest;
 
end

end