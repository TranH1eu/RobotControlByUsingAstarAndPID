clear;
clc;

% map setting
map_size = [50, 50];
% obstacle
obs0 = [1:map_size(1), ...
        map_size(1):map_size(1):map_size(1) * map_size(2), ...
        1:map_size(1):(map_size(1) - 1) * map_size(2), ...
        map_size(1) * (map_size(2) - 1):map_size(1) * map_size(2)];

obs1 = [
        1615:1:1649,...
        1665:1:1699,...
        1715:1:1749
        ];
    
obs2 = [
        701:1:735,...
        751:1:785,...
        ];
        
obstacle = [obs0, obs1,obs2];
hold on;
grid_map = generate_grid(map_size, obstacle);

% start and goal points
start = [5, 5];
goal = [45, 45];

% apply A* algorithm
[path, cost] = a_star_search(grid_map, start, goal);

% global x;
% setGlobalPath(path);
% disp(x);

global x;
reversed_path = zeros(size(path));

for i = 1:size(path, 1)
    reversed_path(i, 1) = path(i, 2); % Đảo ngược tọa độ x và y
    reversed_path(i, 2) = path(i, 1); % Đảo ngược tọa độ x và y
end
setGlobalPath(reversed_path);
disp(x);

% save and plot
save gridmap_46x42_scene1 grid_map
plot_grid(grid_map, path);

% Functions
function grid_map = generate_grid(size, obstacle)
    grid_map = ones(size(1), size(2));
    grid_map(obstacle) = 2;
end

function plot_grid(grid_map, path)
    [rows, cols] = size(grid_map);
    
    % Mark the path in the grid map
    for k = 1:size(path, 1)
        grid_map(path(k, 1), path(k, 2)) = 3;
    end
    
    % Create colormap: 1=free space (white), 2=obstacle (black), 3=path (red)
    cmap = [1 1 1; 0 0 0; 1 0 0];
    colormap(cmap);
    
    % Plot the grid map
    image(1.5, 1.5, grid_map);
    grid on;
    set(gca, 'xtick', 1:cols, 'ytick', 1:rows);
    axis image;
    
    % Draw grid lines
    for row = 1:rows
        line([1, cols + 1], [row, row], 'Color', '#4DBEEE');
    end
    for col = 1:cols
        line([col, col], [1, rows + 1], 'Color', '#4DBEEE');
    end
end

function [path, cost] = a_star_search(grid_map, start, goal)
    [rows, cols] = size(grid_map);
    %luu tru kich thuoc gridmap
    open_list = [];
    closed_list = false(rows, cols);
    g_cost = inf(rows, cols);
    f_cost = inf(rows, cols);
    parent = zeros(rows, cols, 2);%ma tran luu tru cha cua moi nut de tai tao duong di
    
    heuristic = @(x, y) sqrt((x(1) - y(1))^2 + (x(2) - y(2))^2) * (1.0 + 1.0/100);
    g_cost(start(1), start(2)) = 0;
    f_cost(start(1), start(2)) = heuristic(start, goal);
    open_list = [open_list; start, f_cost(start(1), start(2))];
    
    while ~isempty(open_list)
        [~, idx] = min(open_list(:, 3));
        current = open_list(idx, 1:2);
        open_list(idx, :) = [];
       
        %tong the duoc tra ve
        if isequal(current, goal)
            path = goal;
            while ~isequal(current, start)
                current = parent(current(1), current(2), :);
                current = squeeze(current)';
                path = [current; path];
            end
            cost = g_cost(goal(1), goal(2));
            return;
        end
        
        closed_list(current(1), current(2)) = true;
        neighbors = [0 1; 1 0; 0 -1; -1 0; -1 1; 1 1; -1 -1; 1 -1];
        
        for i = 1:8
            neighbor = current + neighbors(i, :);
            if neighbor(1) > 0 && neighbor(1) <= rows && neighbor(2) > 0 && neighbor(2) <= cols
                if closed_list(neighbor(1), neighbor(2)) || grid_map(neighbor(1), neighbor(2)) == 2
                    continue;
                end
                tentative_g_cost = g_cost(current(1), current(2)) + norm(neighbors(i, :));
                if tentative_g_cost < g_cost(neighbor(1), neighbor(2))
                    parent(neighbor(1), neighbor(2), :) = current;
                    g_cost(neighbor(1), neighbor(2)) = tentative_g_cost;
                    f_cost(neighbor(1), neighbor(2)) = g_cost(neighbor(1), neighbor(2)) + heuristic(neighbor, goal);
                    open_list = [open_list; neighbor, f_cost(neighbor(1), neighbor(2))];
                end
            end
        end
    end
    
    path = [];
    cost = inf;
end