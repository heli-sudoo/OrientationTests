addpath( genpath([pwd '/../']) );


clc
clear
% Rotational inertia
I = diag([2,4,3]);

PseudoInertia = 1/2*trace(I)*eye(3)-I;
assert( all(eig(PseudoInertia) > 0) ) % necessary for inertia to be valid. 
PseudoInertia = PseudoInertia/max(eig(PseudoInertia))*.15; % Rescale it so the ellipse we draw isn't massive

%% Symbolic Setup
q_sym = sym('q_sym',[4 1],'real'); % quaternion
w_sym = sym('w_sym',[3 1],'real'); % given in body coordinates
u_sym = sym('tau',[3 1]); % given in body coordinates
x_sym = [q_sym ; w_sym];

%% Problem Info
dt = 0.02; % Time step (seconds)
problem_data.N_horizon = 200;

problem_data.N_state   = 7; % Dimension of x
problem_data.N_control = 3; % Dimension of u
problem_data.N_delta   = 6; % Dimension of delta state (w/cayley)

%% Dynamics
wdot = I\(u_sym-skew(w_sym)*I*w_sym); % Euler equations
nextq = q_sym + quatRateRight(q_sym,w_sym)*dt; % This is a dirty integration routine.
nextq = nextq/sqrt(nextq'*nextq);  % Should use quat product, but this has nicer expression
                                   % that avoids divide by zero when omega=0                    
nextw = w_sym + wdot*dt;
x_quat = [q_sym ; w_sym];
f = [nextq ; nextw]; % Next state;
fquat_fun = matlabFunction(f,'vars',{x_quat, u_sym});

% Functions to convert between full state and delta state
problem_data.state_to_delta = @(x, xbar) stateToDeltaOrientation(x,xbar);
problem_data.delta_to_state = @(xbar, delta) deltaToStateOrientation(xbar, delta);
problem_data.dynamics = fquat_fun;


%% Setting up Cost functions

% desired states
q_des = [1 0 0 0]';
w_des = [0 0 0]';
x_des = [q_des; w_des];

q_d = sym('q_des',[4 1],'real');

weight_quat         = 1e3;
weight_w            = 100;
weight_u            = 1/2;

weight_quat_final   = 1e6;
weight_w_final      = 1e4;

% quadratic parts of cost function. Weighted by terms above
Q_w = eye(3);
R = eye(3);

% I am squaring so that we don't have divide by zero issues.
cost_function_for_quat = (1-(q_d'*q_sym)^2);

cost_quat_func  = matlabFunction(cost_function_for_quat, 'vars', {q_sym,q_d});

problem_data.running_cost = @(x,u) ( 1/2*weight_w*(x(5:7)-w_des)'*Q_w*(x(5:7)-w_des) + ...
                                  weight_quat*cost_quat_func(x(1:4),q_des) + 1/2.*weight_u*u'*R*u ) * dt ;
                              
problem_data.terminal_cost = @(x) weight_w_final*1/2*(x(5:7)-w_des)'*Q_w*(x(5:7)-w_des) + ...
                                   weight_quat_final*cost_quat_func(x(1:4),q_des);
                                                  
%% Preallocate for speed
x_bar = zeros(problem_data.N_state,problem_data.N_horizon);
u_bar = zeros(problem_data.N_control,problem_data.N_horizon);

%% Initial Simulation
q_bar0 = [0 .2 .6 0]';    % In this case
q_bar0 = q_bar0/norm(q_bar0);
w_bar0 = [2 0 3]'*2;
x_bar(:,1) = [q_bar0 ; w_bar0 ];

[x_bar,u_bar] = ForwardRollout(problem_data,x_bar, u_bar, 0, 0);
DrawSim( q_des, x_bar, u_bar, dt,PseudoInertia )
input('Ready?');

%% Optimize
solver_options.tol_fun = 1e0;
solver_options.max_iter = 1500;
solver_options.debug = 0;
solver_options.gamma = .1;

[x_opt, u_opt] = DDP(problem_data, x_bar, u_bar ,solver_options);

%% Simulation
DrawSim( q_des, x_opt,u_opt, dt,PseudoInertia )