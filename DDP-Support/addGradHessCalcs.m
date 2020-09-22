function data = addGradHessCalcs(data)
    
    x = sym('x',[data.N_state 1],'real');
    u = sym('u',[data.N_control 1],'real');
    delta = sym('d',[data.N_delta 1],'real');
    zero = 0*delta;
    
    E = jacobian( data.delta_to_state(x, delta), delta );
    E = subs(E,delta,zero);
    
    data.E_func = matlabFunction(E,'vars',{x});

    running = data.running_cost( data.delta_to_state(x,delta), u );
    terminal = data.terminal_cost( data.delta_to_state(x,delta) );
        
    data.running_d = matlabFunction( subs( gradient(running, delta ) , delta, zero), 'vars', {x,u});
    data.running_u = matlabFunction( subs( gradient(running, u ), delta, zero), 'vars', {x,u});

    data.running_dd = matlabFunction( subs(  hessian( running,  delta), delta, zero), 'vars', {x,u});
    data.running_uu = matlabFunction( subs(  hessian( running,  u    ), delta, zero), 'vars', {x,u});
    data.running_ud = matlabFunction( subs( jacobian( gradient( running, u), delta), delta, zero), 'vars', {x,u});
    
    data.terminal_d  = matlabFunction( subs( gradient(terminal, delta ) , delta, zero), 'vars', {x});
    data.terminal_dd = matlabFunction( subs( hessian( terminal, delta ) , delta, zero), 'vars', {x});
    
    data.dynamics_A = matlabFunction( jacobian( data.dynamics(x,u), x),'vars',{x,u});
    data.dynamics_B = matlabFunction( jacobian( data.dynamics(x,u), u),'vars',{x,u});
    
end