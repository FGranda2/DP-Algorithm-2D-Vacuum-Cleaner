% aer1517_a1_2: Main script for Problem 1.2 Dynamic Programming for a 
%               Robot Vacuum Cleaner.
%
% --
% Control for Robotics
% AER1517 Spring 2022
% Assignment 1
%
% --
% University of Toronto Institute for Aerospace Studies
% Dynamic Systems Lab
%
% Course Instructor:
% Angela Schoellig
% schoellig@utias.utoronto.ca
%
% Teaching Assistants: 
% SiQi Zhou: siqi.zhou@robotics.utias.utoronto.ca
% Adam Hall: adam.hall@robotics.utias.utoronto.ca
% Lukas Brunke: lukas.brunke@robotics.utias.utoronto.ca
%
% --
% Revision history
% [22.01.17, LB]    first version
% Modified and completed by Francisco Granda
clear all
close all
clc

%% calculate optimal control using dynamic programming
% horizon
N = 20;

% initialize the grid world
grid = GridWorld();

% allocate arrays for optimal control inputs and cost-to-go 
%U = zeros(grid.num_rows, grid.num_columns);
J = zeros(grid.num_rows, grid.num_columns);

% set the cost for the obstacle
J(grid.obstacle_pos(1), grid.obstacle_pos(2)) = inf;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%initial_condition =[1;5]; % For Part A
initial_condition =[4;3]; % For Part B
for j = 1:19 % N-1
    if j == 1
        % Set terminal node
        %prev = grid.charger_pos;
        prev = initial_condition;
        % Compute possible next nodes, cost and action from prev node
        [st, co, po] = cost2go(grid,prev);
        % Eliminate origin from pos states
        [~,ib]=ismember(prev.',st.','rows');
        if ib ~= 0
            st(:,ib) = [];
            co(:,ib) = [];
            po(:,ib) = [];
        end

        % Update cost grid J and U
        for i = 1:size(st,2)
            J(st(1,i),st(2,i)) = co(1,i);
            U{st(1,i),st(2,i)} = po(1,i); 
        end
        prev = st;
    else 
        % Reset vectors
        i_states = [];
        i_prev = [];
        i_lim = [];
        i_actions = [];
        total_cost = [];
        total_action = {};
        
        for i = 1:size(prev,2)
            [st,~,po] = cost2go(grid,prev(1:2,i));
            % Eliminate going back to previous state
            act_history = U{prev(1,i),prev(2,i)};
            last_act = act_history(1,end);
            % Execute only if action is not 0 (Stay in place at charger)
            if last_act ~= 0 
                p_state = prev_state(prev(1:2,i), last_act);
                [~,ib]=ismember(p_state.',st.','rows');
                st(:,ib) = [];
                po(:,ib) = [];
            end
            % List possible i states, with costs and actions from prev
            i_states = [i_states,st];
            i_prev = [i_prev, prev(1:2,i).*ones(size(st))];  
            i_actions = [i_actions,po];
        end
        
        % Compute costs of i to j
        for i = 1:size(i_states,2)
            total_cost(1,i) = grid.stage_cost(i_states(1,i),i_states(2,i)) + J(i_prev(1,i),i_prev(2,i));
            total_action{1,i} = [U{i_prev(1,i),i_prev(2,i)},i_actions(i)];
        end
        
        % Update J cost matrix
        for i = 1:size(total_cost,2)
            if J(i_states(1,i),i_states(2,i)) == 0
                J(i_states(1,i),i_states(2,i)) = total_cost(i);
                U{i_states(1,i),i_states(2,i)} = total_action{1,i};
            else
                if total_cost(i) <= J(i_states(1,i),i_states(2,i))
                   J(i_states(1,i),i_states(2,i)) = total_cost(i);
                   U{i_states(1,i),i_states(2,i)} = {};
                   U{i_states(1,i),i_states(2,i)} = total_action{1,i};
                end
            end
        end
        prev = unique(i_states.','rows').';
    end
end
J(initial_condition(1), initial_condition(2)) = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Simulate robot vacuum cleaner
x_0 = [4; 3];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TODO: YOUR CODE HERE - Exercise 2 (b)
% FIRST: RUN CODE IN PART A WITH THE NEW INITIAL CONDITION x_0
optimal_actions = U{1,5};
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

grid.plot_moves(x_0, optimal_actions)
