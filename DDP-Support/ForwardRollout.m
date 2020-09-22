function [x,u, V] = ForwardRollout(problem_data, x_bar, u_bar , K , feedback_active)
    if nargin == 4
        feedback_active = 1;
    end
    
    dynamics = problem_data.dynamics;
    running_cost = problem_data.running_cost;
    terminal_cost = problem_data.terminal_cost;
    state_to_delta = problem_data.state_to_delta;
    
    x = 0*x_bar;
    u = 0*u_bar;
    
    % Initial conditions taken froom x_bar
    u(:,1) = u_bar(:,1);
    x(:,1) = x_bar(:,1);
    V = 0;

    for i = 2:size(x_bar,2)
        % Add up running cost
        V = V + running_cost(x(:,i-1), u(:,i-1));
        
        % Simulate forward
        x(:,i) = dynamics(x(:,i-1),u(:,i-1));
        
        if feedback_active
            delta  = state_to_delta(x(:,i), x_bar(:,i));
            u(:,i) = u_bar(:,i) - K(:,:,i)*delta;
        else
            u(:,i) = u_bar(:,i);
        end
    end
    V = V  + terminal_cost(x(:,i));
    
end

        
        