function [x_bar, u_bar] = DDP(problem_data, x_bar, u_bar,options)

    disp("Adding Calcs");
    problem_data = addGradHessCalcs(problem_data);
    disp('Done');

    %% Initial Sim
    [x_bar,u_bar, V_bar] = ForwardRollout(problem_data, x_bar, u_bar, 0, 0);

    %% Main Loop
    for iteration = 1:options.max_iter 
        disp(strcat("Iteration: ",num2str(iteration)));
        disp("----------------------------------------");

        % BackwardPass
        [P, p, p0, delta_u, K] = BackPass(problem_data, x_bar, u_bar);

        % Forward Pass
        if isfield(options,'debug') && options.debug
            DebugRollouts(problem_data, x_bar, u_bar, delta_u, K, p0);
        end
        [x_bar, u_bar, V_bar] = ForwardPass(problem_data, x_bar, u_bar, V_bar, delta_u, K, p0, options);
        
        % Check to see if we can stop
        if p0(1) > -options.tol_fun
            break
        end
    end % end overall for loop
end
