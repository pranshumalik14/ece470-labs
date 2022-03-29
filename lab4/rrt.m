% solves the motion planning problem for a manipulator robot using
% rapidly-exploring random tree (RRT) algorithm with obstacle avoidance
function [q_path, q_err, tree] = rrt(q_start, q_goal, mykuka, obs, delta_q, n_iter, p, lb, ub, tol)

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
    elseif collision(q_new, obs, 0.2)
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
function colliding = collision(q, obs, dr)
    % compute the current joint poses and distance from obstacle surface
    Hqi    = zeros(4, 4, size(q,2)+1);
    
    Hqi(:,:,1) = trotz(0); % start from robot base frame: default world frame
    for joint_idx = 1:size(q,2)
        Hqi(:,:,joint_idx+1) = forward(q(1:joint_idx)', mykuka);
    end

    % Forward kinematics is done => need to check for collisions now
    for obs_idx = 1:size(obs, 2)
        % todo: check
        obs_i = obs{obs_idx};
        sphere_center = obs_i.c;
        sphere_influence = obs_i.R + dr;

        if obs_i.type == "sph"
            % compute link i
            for joint_idx = 1:size(q,2)
                joint_i = Hqi(:, :, joint_idx);
                joint_i_1 = Hqi(:, :, joint_idx+1);

                org_joint_i = joint_i(1:3, 4);
                org_joint_i_1 = joint_i_1(1:3, 4);

                link_i_vec = org_joint_i_1 - org_joint_i;
                joint2sphe = sphere_center - org_joint_i;

                % dot product
                dotProduct = dot(joint2sphe, link_i_vec);
                projection_obs = dotProduct * link_i_vec;

                dist_vector = norm(projection_obs - joint2sphe);

                if dist_vector > sphere_influence
                    continue;
                else
                    colliding = true;
                    return;
                end
            end

        elseif obs_i.type == "cyl"
            continue;
        elseif obs_i.type == "pla"
            obs_i_influence = obs_i.rho0;
            obs_i_z_val = obs_i.h;

            for joint_idx = 1:size(q,2)
                joint_i = Hqi(:, :, joint_idx);
                joint_i_1 = Hqi(:, :, joint_idx+1);

                org_joint_i = joint_i(1:3, 4);
                org_joint_i_1 = joint_i_1(1:3, 4);

                % CHeck if both points above plane or below


            end
        end

    end
    colliding = false; % todo: add logic
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