function x_prev = prev_state(x, a)
% COMPUTE PREVIOUS STATE BASED ON CURRENT STATE IN GRID
            if a == 0
                x_prev = x;
            elseif a == 3
                x_prev = x + [-1; 0];
            elseif a == 4
                x_prev = x + [0; 1];
            elseif a == 1 
                x_prev = x + [1; 0];
            elseif a == 2
                x_prev = x + [0; -1];
            end
        
        end
