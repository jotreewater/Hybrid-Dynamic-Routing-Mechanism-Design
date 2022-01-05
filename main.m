% CS 455 - Flocking Project

clc,clear
close all

% User Inputs
using_dijkstras = 1; % Using dijikstras

% Variables

% Nodes
num_nodes = 7;
node_positions = zeros(num_nodes,2);
node_positions(1,:) = [50,50];
node_positions(2,:) = [200,75];
node_positions(3,:) = [200,50];
node_positions(4,:) = [200,25];
node_positions(5,:) = [225,62.5];
node_positions(6,:) = [225,37.5];
node_positions(7,:) = [250,50];

active_node = 1;
active_node_iteration = 1;
active_node_cell_iteration = 1;

% Verticies
A = zeros(num_nodes,num_nodes);
A(1,:) = [0,1,1,1,1,1,1];
A(2,:) = [50,0,1,0,1,0,0];
A(3,:) = [50,1,0,1,1,1,0];
A(4,:) = [50,0,1,0,0,1,0];
A(5,:) = [50,1,1,0,0,1,1];
A(6,:) = [50,0,1,1,1,0,1];
A(7,:) = [50,0,0,0,1,1,0];

% Battery Levels
battery_level = zeros(num_nodes,1);
out_of_battery = 0;
for i=2:num_nodes
    battery_level(i,:) = 100;
end


% Action Cost
verticies = cell(num_nodes,1);
for i = 1:num_nodes
    neighbor_number = 1;
    for j = 1:num_nodes
        if A(i,j) > 0
            verticies{i}(neighbor_number,:) = node_positions(j,:);
            neighbor_number = neighbor_number + 1;
        end
    end
end

disp('Start.');

k = 1;

% Iteration Loop
while ~out_of_battery
    
    % Set Active Node
    if active_node == 1
        
        active_node = rand();
        if active_node < 0.33
            active_node = 7;
        elseif active_node < .495
            active_node = 6;
        elseif active_node < .66
            active_node = 5;
        elseif active_node < .825
            active_node = 4;
        else
            active_node = 2;
        end
        
        if using_dijkstras
            [shortest_path] = Dijkstras(active_node, A, num_nodes);
        else
            shortest_path(1,1) = 1;
            shortest_path(1,2) = active_node;
        end
        
        if active_node_iteration > 1
            active_node_iteration = 1;
            active_node_cell_iteration = active_node_cell_iteration + 1;
            active_node_cell_history{active_node_cell_iteration - 1,1} = active_node_history(:,1);
        end
    end
    
    % For overlaying plot elements
    
    % Draw Nodes
    hold off;
    figure(1),plot(node_positions(:,1),node_positions(:,2),'ro','LineWidth',2,'MarkerEdgeColor','k', 'MarkerFaceColor','k','MarkerSize',4.2);
    hold on;
    plot(node_positions(active_node,1),node_positions(active_node,2),'ro','LineWidth',2,'MarkerEdgeColor','red', 'MarkerFaceColor','red','MarkerSize',4.2);
    hold on;
    
    [next_node, battery_drain] = Transmit(active_node, shortest_path,num_nodes,A);
    active_node_history(active_node_iteration,1) = active_node;
    
    for i=1:num_nodes
        battery_level_history{k,1}(i,1) = battery_level(i,1);
    end
    battery_level(active_node, 1) = battery_level(active_node, 1) - battery_drain;
    
    k = k + 1;
    
    % Draw Vertices
    for i = 1:num_nodes
        for j = 1:size(verticies{i})
            if i == active_node && verticies{i}(j,1) == node_positions(next_node,1) && verticies{i}(j,2) == node_positions(next_node,2)
                plot([node_positions(active_node,1),verticies{i}(j,1)],[node_positions(active_node,2),verticies{i}(j,2)], 'r','LineWidth',2);
                hold on;
            elseif i == next_node && verticies{i}(j,1) == node_positions(active_node,1) && verticies{i}(j,2) == node_positions(active_node,2)
                plot([node_positions(next_node,1),verticies{i}(j,1)],[node_positions(next_node,2),verticies{i}(j,2)], 'r','LineWidth',2);
                hold on;
            else
                plot([node_positions(i,1),verticies{i}(j,1)],[node_positions(i,2),verticies{i}(j,2)], 'b');
            end
        end
    end
    
    active_node_iteration = active_node_iteration + 1;
    active_node = next_node;
    
    % Check Battery Levels
    for i=2:num_nodes
        if battery_level(i,1) < 1
            out_of_battery = 1;
        elseif battery_level(i,1) < 33
            A(i,1) = 200;
        elseif battery_level(i,1) < 66
            A(i,1) = 100;
        end
    end
    
    if active_node == 1
        active_node_history(active_node_iteration,1) = active_node;
    end
end

for i=1:length(active_node_cell_history)
    for j=1:length(active_node_cell_history{i})
        figure(2), plot(i,active_node_cell_history{i}(j,1), 'ob');
        hold on;
    end
end

for i=1:length(battery_level_history)
    for j=1:length(battery_level_history{i})
        figure(3), plot(i,battery_level_history{i}(j,1), 'or');
        hold on;
    end
end

disp('Done.');

% Functions
function [next_node, battery_drain] = Transmit(active_node, shortest_path,num_nodes,A)

i = 1;
battery_drain = 0.5;

while shortest_path(i) ~= active_node
    i = i + 1;
end
[next_node] = shortest_path(i-1);
if next_node == 1
    for j=1:num_nodes
        if A(active_node,1) == 50
            battery_drain = 10;
        end
        if A(active_node,1) == 100
            battery_drain = 5;
        end
        if A(active_node,1) == 200
            battery_drain = 2.5;
        end
    end
end
end

function [shortest_path] = Dijkstras(active_node, A, num_nodes)
max = 65536; % Define a maximum integer value
cost = zeros(num_nodes,num_nodes);% Throughput speed

    % Copy adjacency matrix into cost matrix
    for i=1:num_nodes
        for j=1:num_nodes
            if A(i,j) == 0
                cost(i,j) = max; % Prevents invalid behavior from being chosen
            else
                cost(i,j) = A(i,j); % Copies Verticies
            end
        end
    end
    % Initialize variables
    for i=1:num_nodes
        distance(i,1) = cost(active_node,i);
        path(i,1) = active_node; 
        visited(i,1) = 0;
    end
    
    distance(active_node) = 0; % Distance from the starting node is always zero
    visited(i,1) = 1; % Marks the active node as visited
    count = 1;
    
    % For each node, except the source node
    while count < num_nodes - 1
        minimum_distance = max; 
        % Find closest neighbor
        for i=1:num_nodes
            if distance(i,1) < minimum_distance
                if visited(i,1) ~= 1
                minimum_distance = distance(i,1);
                next_node = i;
                end
                
            end
        end
        visited(next_node,1) = 1;
        
        % Find distances to other nodes from closest neighbor
        for i=1:num_nodes
            if visited(i,1) ~= 1
                if minimum_distance + cost(next_node,i) < distance(i,1)
                    distance(i,1) = minimum_distance + cost(next_node,i);
                    path(i,1) = next_node;
                end
            end
        end
        count = count + 1;
    end
    
    disp("-- Starting from node: " + active_node + " --")
    
    % Print Results to console for verification, and update shortest path
    for i=1:num_nodes
        if i~= active_node
            j = i;
            if i == 1
                disp("Distance to node " + i + " is " + distance(i,1))
                disp("Backwards Path is " + i)
                shortest_path(1,1) = j;
                k = 2;
            end
            while j ~= active_node
                j = path(j);
                if i == 1 && j ~= active_node
                    disp("Preceeded by " + j)
                    shortest_path(k,1) = j;
                    k = k + 1;
                end
            end
        end
    end
    shortest_path(k,1) = active_node;
end
    
    
    
    
    
    
    
    
    
    
    
    
    