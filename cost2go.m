function [posible_states, cost_to_go, pos_actions] = cost2go(grid,state)
% COMPUTE POSIBLE NEXT STATES AND COST TO GO
pos_actions = available_actions(grid, state);
for i = 1:size(pos_actions,2)
    posible_states(1:2,i) = next_state(grid, state, pos_actions(i));
    cost_to_go(1,i) = grid.stage_cost(posible_states(1,i),posible_states(2,i));
end
end
