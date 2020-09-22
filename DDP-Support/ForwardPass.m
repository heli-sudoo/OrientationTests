function [x, u, V_new] = ForwardPass(problem_data, x_bar, u_bar, V_bar, delta_u, K, p0, options) 

    epsilon = 1; % initialize variable to cut back on step
    gamma = options.gamma; % between 0 and 1, 1=greedy 0=low standards

    disp(strcat("V_old: ",num2str(V_bar)));
    disp(strcat("Required Decrement: ",num2str(gamma*epsilon*(2-epsilon)*p0(1))));

    while 1==1
        [x,u, V_new] = ForwardRollout(problem_data, x_bar, u_bar+epsilon*delta_u , K);
        
        disp(strcat("              ",num2str(V_new)));

        if isnan(V_new) || ( V_new > (V_bar + gamma*epsilon*(2-epsilon)*p0(1)) )
            epsilon = epsilon/2;
        else
            break
        end
    end % end forward pass while
end