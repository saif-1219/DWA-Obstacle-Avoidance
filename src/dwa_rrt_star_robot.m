function dwa_rrt_star_stable()


    %% 1. Configuration
    config = struct();
    
    % --- DWA Settings (Stabilized) ---
    config.max_speed = 1.0;      
    config.min_speed = -0.5;

    config.max_yaw_rate = 50.0 * pi / 180.0;  
    config.max_accel = 0.3; 
    config.max_yaw_accel = 50.0 * pi / 180.0; 
    
    config.v_reso = 0.01;
    config.yaw_rate_reso = 0.1 * pi / 180.0; 
    config.dt = 0.1;
    
    config.predict_time = 2.0;   
    
    config.to_goal_cost_gain = 0.15; 
    config.speed_cost_gain = 1.0; 
    config.obstacle_cost_gain = 1.0;
    
    config.robot_radius = 0.30;   
    config.goal_tolerance = 0.5;

    config.rrt_max_iter = 2000;       
    config.rrt_step_size = 1.0;       
    config.rrt_goal_sample_rate = 0.1;
    config.rrt_search_radius = 3.0;   
    
    % This ensures diagonal gaps between blocks are treated as CLOSED walls.
    config.safe_dist = 0.8;           
    
    map_size = 20; 
    resolution = 1.0;

    %% 2. Generate Static Maze (Walls)
    obs = create_maze_walls(map_size, resolution);

    % Visualize Maze
    figure(1); clf; hold on; grid on; axis equal;
    axis([-1 map_size+1 -1 map_size+1]);
    
    % Draw "Safety Zones" to show user where the planner thinks walls are
    vis_circles(obs, config.safe_dist); 
    plot(obs(:,1), obs(:,2), 'sk', 'MarkerSize', 12, 'MarkerFaceColor', 'k');
    
    title('Step 1: Select START (Left Click) and GOAL (Right Click)');

    %% 3. Interactive Start/Goal Selection
    [start_pos, goal_pos] = select_start_goal(map_size);
    
    % Initial State [x, y, yaw, v, omega]
    x = [start_pos(1); start_pos(2); 0; 0; 0];
    
    %% 4. Global Path Planning (RRT*)
    title('Planning Global Path (Solid Walls)...');
    drawnow;
    
    % Run RRT* Planner
    raw_path = rrt_star_planner(x(1:2), goal_pos, obs, map_size, config);
    
    if isempty(raw_path)
        title('Error: RRT* failed (Path blocked by walls)');
        error('RRT* failed to find a path! Try points further from walls.');
    end

    % Interpolate path 
    path = interpolate_path(raw_path, 0.2); 
    
    % Plot the calculated path
    plot(path(:,1), path(:,2), '-g', 'LineWidth', 2);
    
    %% 5. Add Dynamic Obstacles
    title({'Path Generated.', 'Left Click to ADD OBSTACLES', 'Right Click to START'});
    
    dyn_obs = [];
    while true
        [xi, yi, button] = ginput(1);
        if button == 3, break; 
        elseif button == 1
            new_ob = [xi, yi];
            dyn_obs = [dyn_obs; new_ob];
            plot(xi, yi, 'sr', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
            vis_circles([xi, yi], config.safe_dist); % Show new safety zone
        end
    end
    all_obs = [obs; dyn_obs];
    
    %% 6. Main Simulation Loop
    title('Running: Stable DWA');
    traj_x = x(1); traj_y = x(2);
    
    while true
        % 1. Find target on global path 
        target_idx = select_target_index(x, path);
        current_target = path(target_idx, :);

        % 2. DWA Control
        [u, best_traj] = dwa_control(x, config, current_target, all_obs);

        % 3. Update State
        x = motion_model(x, u, config.dt);
        traj_x = [traj_x; x(1)]; 
        traj_y = [traj_y; x(2)];

        % 4. Visualization
        clf; hold on; grid on; axis equal;
        axis([-1 map_size+1 -1 map_size+1]);
        
        plot(obs(:,1), obs(:,2), 'sk', 'MarkerSize', 12, 'MarkerFaceColor', 'k'); % Walls
        if ~isempty(dyn_obs)
            plot(dyn_obs(:,1), dyn_obs(:,2), 'sr', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
        end
        plot(path(:,1), path(:,2), '-g', 'LineWidth', 1.5); % Global Path
        plot(goal_pos(1), goal_pos(2), '*r', 'MarkerSize', 12, 'LineWidth', 2); % Goal
        
        % Show Lookahead Target (The "Carrot")
        plot(current_target(1), current_target(2), 'xg', 'MarkerSize', 10, 'LineWidth', 2);
        
        if ~isempty(best_traj)
            plot(best_traj(:,1), best_traj(:,2), '-c', 'LineWidth', 2); % Predicted
        end
        plot(traj_x, traj_y, '-b', 'LineWidth', 1.5); % History
        draw_robot(x, config.robot_radius); 
        
        % Check Goal
        dist_to_goal = sqrt((x(1) - goal_pos(1))^2 + (x(2) - goal_pos(2))^2);
        if dist_to_goal <= config.goal_tolerance
            title('Goal Reached!'); disp('Goal Reached!'); break;
        end
        drawnow;
    end
end

%% --- RRT* Planner Functions ---

function path = rrt_star_planner(start_pos, goal_pos, obs, map_size, config)
    nodes(1).x = start_pos(1);
    nodes(1).y = start_pos(2);
    nodes(1).cost = 0;
    nodes(1).parent = 0;
    
    best_goal_idx = -1;
    min_goal_cost = inf;
    
    for i = 1:config.rrt_max_iter
        if rand < config.rrt_goal_sample_rate
            rnd = goal_pos;
        else
            rnd = [rand*map_size; rand*map_size];
        end
        
        [near_idx, dist] = get_nearest_node(nodes, rnd);
        if dist == 0, continue; end
        
        theta = atan2(rnd(2) - nodes(near_idx).y, rnd(1) - nodes(near_idx).x);
        new_node.x = nodes(near_idx).x + config.rrt_step_size * cos(theta);
        new_node.y = nodes(near_idx).y + config.rrt_step_size * sin(theta);
        
        % Bounds Check
        if new_node.x < 0 || new_node.x > map_size || new_node.y < 0 || new_node.y > map_size
            continue;
        end
        
        % Collision Check (Strict)
        if ~check_collision(nodes(near_idx), new_node, obs, config.safe_dist)
            continue;
        end
        
        near_indices = find_near_nodes(nodes, new_node, config.rrt_search_radius);
        new_node.cost = nodes(near_idx).cost + config.rrt_step_size;
        new_node.parent = near_idx;
        
        % Optimization (Parent)
        for k = 1:length(near_indices)
            idx = near_indices(k);
            d = sqrt((nodes(idx).x - new_node.x)^2 + (nodes(idx).y - new_node.y)^2);
            if nodes(idx).cost + d < new_node.cost
                if check_collision(nodes(idx), new_node, obs, config.safe_dist)
                    new_node.cost = nodes(idx).cost + d;
                    new_node.parent = idx;
                end
            end
        end
        
        new_idx = length(nodes) + 1;
        nodes(new_idx) = new_node;
        
        % Optimization (Rewire)
        for k = 1:length(near_indices)
            idx = near_indices(k);
            d = sqrt((nodes(idx).x - new_node.x)^2 + (nodes(idx).y - new_node.y)^2);
            if new_node.cost + d < nodes(idx).cost
                if check_collision(nodes(new_idx), nodes(idx), obs, config.safe_dist)
                    nodes(idx).parent = new_idx;
                    nodes(idx).cost = new_node.cost + d;
                end
            end
        end
        
        d_goal = sqrt((new_node.x - goal_pos(1))^2 + (new_node.y - goal_pos(2))^2);
        if d_goal <= config.rrt_step_size
             if new_node.cost < min_goal_cost
                 min_goal_cost = new_node.cost;
                 best_goal_idx = new_idx;
             end
        end
    end
    
    path = [];
    if best_goal_idx ~= -1
        curr = best_goal_idx;
        path = [goal_pos(1), goal_pos(2)];
        while curr ~= 0
            path = [nodes(curr).x, nodes(curr).y; path];
            curr = nodes(curr).parent;
        end
    end
end

function [idx, min_dist] = get_nearest_node(nodes, rnd)
    xs = [nodes.x]; ys = [nodes.y];
    dists = (xs - rnd(1)).^2 + (ys - rnd(2)).^2;
    [min_dist, idx] = min(dists);
    min_dist = sqrt(min_dist);
end

function idxs = find_near_nodes(nodes, new_node, r)
    xs = [nodes.x]; ys = [nodes.y];
    dists = (xs - new_node.x).^2 + (ys - new_node.y).^2;
    idxs = find(dists <= r^2);
end

function safe = check_collision(n1, n2, obs, safe_dist)
    safe = true;
    % High Resolution Check: Steps every 5cm
    steps = ceil(sqrt((n1.x-n2.x)^2 + (n1.y-n2.y)^2) / 0.05);
    if steps < 2, steps = 2; end
    xs = linspace(n1.x, n2.x, steps);
    ys = linspace(n1.y, n2.y, steps);
    
    % Squared distance check is faster
    safe_sq = safe_dist^2;
    for i = 1:steps
        d = (obs(:,1) - xs(i)).^2 + (obs(:,2) - ys(i)).^2;
        if any(d < safe_sq), safe = false; return; end
    end
end

function dense_path = interpolate_path(path, resolution)
    dense_path = [];
    for i = 1:size(path, 1)-1
        p1 = path(i, :); p2 = path(i+1, :);
        dist = norm(p2 - p1);
        num_points = ceil(dist / resolution);
        if num_points == 0, num_points = 1; end
        xs = linspace(p1(1), p2(1), num_points)';
        ys = linspace(p1(2), p2(2), num_points)';
        dense_path = [dense_path; xs, ys];
    end
    dense_path = [dense_path; path(end, :)]; 
end

%% --- Helper Functions ---

function vis_circles(obs, r)
    theta = 0:0.5:2*pi;
    for i = 1:size(obs, 1)
        px = obs(i,1) + r*cos(theta);
        py = obs(i,2) + r*sin(theta);
        plot(px, py, 'Color', [0.8 0.8 0.8]);
    end
end

function obs = create_maze_walls(map_size, res)
    obs = [];
    for i = 0:res:map_size
        obs = [obs; i, 0; i, map_size; 0, i; map_size, i];
    end
    for y = 0:res:map_size/2, obs = [obs; 7, y]; end
    for y = map_size/2:res:map_size, obs = [obs; 14, y]; end
    for x = 2:res:10, obs = [obs; x, 14]; end
end

function [start_pos, goal_pos] = select_start_goal(map_size)
    start_pos = []; goal_pos = [];
    disp('Select Start (Left) and Goal (Right)...');
    while isempty(start_pos) || isempty(goal_pos)
        [xi, yi, b] = ginput(1);
        if xi<0||xi>map_size||yi<0||yi>map_size, continue; end
        if b==1, start_pos=[xi;yi]; plot(xi,yi,'ob','MarkerSize',12); text(xi,yi+1,'Start');
        elseif b==3, goal_pos=[xi;yi]; plot(xi,yi,'xr','MarkerSize',12); text(xi,yi+1,'Goal'); end
    end
end

function idx = select_target_index(state, path)
    dists = sqrt(sum((path - [state(1), state(2)]).^2, 2));
    [~, min_idx] = min(dists);
    
    % FIX: Lookahead +6 indices (approx 1.2m)
    % This stops the robot from chasing its tail
    idx = min(min_idx + 6, size(path, 1)); 
end

function draw_robot(x, r)
    theta = 0:0.1:2*pi;
    fill(x(1)+r*cos(theta), x(2)+r*sin(theta), 'r');
    plot([x(1), x(1)+r*cos(x(3))], [x(2), x(2)+r*sin(x(3))], 'k', 'LineWidth', 2);
end

%% --- DWA Core ---
function [u, best_traj] = dwa_control(x, config, goal, obs)
    dw = calc_dynamic_window(x, config);
    [u, best_traj] = calc_control_and_trajectory(x, dw, config, goal, obs);
end

function dw = calc_dynamic_window(x, config)
    Vs = [config.min_speed, config.max_speed, -config.max_yaw_rate, config.max_yaw_rate];
    Vd = [x(4)-config.max_accel*config.dt, x(4)+config.max_accel*config.dt, ...
          x(5)-config.max_yaw_accel*config.dt, x(5)+config.max_yaw_accel*config.dt];
    dw = [max(Vs(1),Vd(1)), min(Vs(2),Vd(2)), max(Vs(3),Vd(3)), min(Vs(4),Vd(4))];
end

function [u, best_traj] = calc_control_and_trajectory(x, dw, config, goal, obs)
    min_cost = inf; best_u = [0;0]; best_traj = [];
    for v = dw(1):config.v_reso:dw(2)
        for y = dw(3):config.yaw_rate_reso:dw(4)
            traj = predict_trajectory(x, v, y, config);
            to_goal = config.to_goal_cost_gain * sqrt((goal(1)-traj(end,1))^2 + (goal(2)-traj(end,2))^2);
            speed = config.speed_cost_gain * (config.max_speed - traj(end,4));
            ob_cost = config.obstacle_cost_gain * calc_obstacle_cost(traj, obs, config);
            if (to_goal + speed + ob_cost) < min_cost
                min_cost = to_goal + speed + ob_cost;
                best_u = [v; y]; best_traj = traj;
            end
        end
    end
    u = best_u;
end

function traj = predict_trajectory(x_init, v, y, config)
    x = x_init; traj = x';
    for t = 0:config.dt:config.predict_time
        x(3) = x(3) + y*config.dt;
        x(1) = x(1) + v*cos(x(3))*config.dt;
        x(2) = x(2) + v*sin(x(3))*config.dt;
        x(4) = v; x(5) = y;
        traj = [traj; x'];
    end
end

function cost = calc_obstacle_cost(traj, obs, config)
    min_dist = inf;
    for i=1:2:size(traj,1)
        for j=1:size(obs,1)
            d = sqrt((traj(i,1)-obs(j,1))^2 + (traj(i,2)-obs(j,2))^2);
            if d < min_dist, min_dist = d; end
        end
    end
    if min_dist <= config.robot_radius, cost = inf; else, cost = 1.0/min_dist; end
end

function x = motion_model(x, u, dt)
    x(3) = x(3) + u(2)*dt;
    x(1) = x(1) + u(1)*cos(x(3))*dt;
    x(2) = x(2) + u(1)*sin(x(3))*dt;
    x(4) = u(1); x(5) = u(2);
end