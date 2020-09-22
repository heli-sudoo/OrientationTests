function [P, p, p0, delta_u, K] = BackPass(info, x_bar, u_bar)

    function [H, flg] = shiftHessian(H)
        minEig = min(real(eig(H)));
        flg = minEig< (1e-6);
        if flg
            H = H +1.1*eye(size(H,1))*(1e-6 - minEig);
        end
    end

    function H = sym(H)
        H = (H+H')/2;
    end


    delta_example = info.state_to_delta(x_bar(:,end),x_bar(:,end));

    N_horizon = size(x_bar,2);
    N_state = length(delta_example);
    N_control = size(u_bar,1);

    % Preallocate Ouputs for Speed
    p0 = zeros( N_horizon, 1                   );
    p  = zeros( N_state  , N_horizon           );
    P  = zeros( N_state  , N_state  , N_horizon);
    K  = zeros( N_control, N_state  , N_horizon);
    delta_u = zeros( N_control  , N_horizon);
    
    p(:,N_horizon) = info.terminal_d(  x_bar(:,N_horizon) );
    P(:,:, N_horizon) = shiftHessian( info.terminal_dd( x_bar(:,N_horizon) ) );

    % no expected cost reduction because we're done
    p0(N_horizon) = 0;

    for i = (N_horizon-1):-1:1

        % Transformation between cayley and change in quat
        E      = info.E_func( x_bar(:,i) );
        E_next = info.E_func( x_bar(:,i+1) );

        % Jacobains of the dynamics w.r.t, changes in x_cayley
        xi = x_bar(:,i);
        ui = u_bar(:,i);

        A = E_next'* info.dynamics_A(xi,ui) * E;
        B = E_next'* info.dynamics_B(xi,ui);

        % Temp variables so the equations are less ugly
        Lxx = shiftHessian( info.running_dd(xi,ui) );
        
        Qxx = Lxx                                    + A'*P(:,:,i+1)*A;
        Quu = info.running_uu(xi,ui)                 + B'*P(:,:,i+1)*B;
        Qux = info.running_ud(xi,ui)                 + B'*P(:,:,i+1)*A;
        Qx  = info.running_d( xi,ui)                 + A'*p(:,i+1);
        Qu  = info.running_u( xi,ui)                 + B'*p(:,i+1);
        Q0  = p0(i+1);

        % Ricatti Eqn.
        P(:,:,i) = sym( Qxx   -Qux'*(Quu\Qux) );
        p(:,i)   =      Qx    -Qux'*(Quu\Qu );
        p0(i)    =      Q0 -1/2*Qu'*(Quu\Qu );
        
        delta_u(:,i) = -Quu\Qu;
        K(:,:,i) = Quu\Qux;
        
        assert( min(eig(Quu)) > 0);
        assert(min(eig(P(:,:,i))) > 0);
    end
end