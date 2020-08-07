% In this case, the "nominal" trajectory is going to be staying
% put at the identity quaternion
addpath(genpath(pwd));

% Rotational inertia
I = diag([2,4,3]);

PseudoInertia = 1/2*trace(I)*eye(3)-I;
assert( all(eig(PseudoInertia) > 0) ) % necessary for inertia to be value
PseudoInertia = PseudoInertia/max(eig(PseudoInertia))*.15;


q = sym('q',[4 1],'real');
w = sym('w',[3 1],'real'); % body coordinates
u = sym('tau',[3 1]);

wdot = I\(u-skew(w)*I*w); % Euler equations

dt = 0.002; % Time step
%nw = sqrt(w'*w);
%what = w/nw;
%nextq = quatL(q)*[cos(nw*dt/2) ; what*sin(nw*dt/2)]; % quat_dot*dt

%% Dynamics
nextq = q + quatRateRight(q,w)*dt; % This is a dirty integration routine.
nextq = nextq/sqrt(nextq'*nextq);  % Should use quat product, but this has nicer expression
nextw = w + wdot*dt;

x_quat = [q ; w];
f = [nextq ; nextw]; % Next state;

% Jacobians of the dynamics 
Aq = jacobian(f,x_quat);
Bq = jacobian(f,u);

% Make them into callable functions rather than symbolics
fquat_fun = matlabFunction(f,'vars',{x_quat, u});
Aquat_fun = matlabFunction(Aq,'vars',{x_quat, u});
Bquat_fun = matlabFunction(Bq,'vars',{x_quat, u});

% Desired and nominal are the same here
q_des = [1 0 0 0]';
q_des = rand(4,1);
q_des = q_des/norm(q_des);
q_bar = q_des;    % In this case
w_bar = [0 0 0]';
u_bar = [0 0 0]';

x_bar = [q_bar ; w_bar];
x_bar_next = fquat_fun(x_bar, u_bar);
q_bar_next = x_bar_next(1:4);

% G as in the paper
G_bar = quatG(q_bar);
G_bar_next = quatG(q_bar_next);

% Transformation between cayley and change in quat
E = [G_bar zeros(4,3); zeros(3,3) eye(3)];
% Transformation between cayley and change in quat at next time step
E_next = [G_bar_next zeros(4,3); zeros(3,3) eye(3)];

% Jacobains of the dynamics w.r.t, changes in x_cayley
A_phi = E_next'* Aquat_fun(x_bar,u_bar) * E;
B_phi = E_next'* Bquat_fun(x_bar,u_bar);

% I am squaring so that we don't have divide by zero issues.
cost_function_for_quat = 10*(1-(q_des'*q)^2);

% Gradient of cost
grad_quat = subs(jacobian(cost_function_for_quat,q),q, q_bar);
grad_phi = grad_quat*G_bar;

% Hessian of cost
Hess_quat = subs(hessian(cost_function_for_quat,q), q, q_bar);
Hess_phi  = G_bar_next'*Hess_quat*G_bar - eye(3)*(grad_quat*q_bar);

% Hessian of cost w.r.t. x_cayley
Q_w    = eye(3)/10;
Q = double([Hess_phi zeros(3); zeros(3) Q_w]);
R = eye(3)/3;

%
[P, K] = idare(A_phi,B_phi,Q,R);

%% Simulation
t = 0;
tf = 10;

e_quat = [1.1 .4 .1 .1]';
e_quat = e_quat/norm(e_quat);
q = quatProduct(q_des,e_quat);
w = [0 2 3]';
x = [q ; w];
%%
figure(1);
clf;
g = axes();
CoordAxes(g,.05/5, .1/2/5,.3/5, { [1 1 1]*.6, [1 1 1]*.6, [1 1 1]*.6},1,1 )

h_des = hgtransform(g);
h_des.Matrix = [quatToRot(q_des) zeros(3,1) ; 0 0 0 1];

CoordAxes(h_des,.05/2, .1/2,.3/2, { [1 0 0]*.5, [0 1 0]*.5, [0 0 1]*.5},1,1 )



h_body = hgtransform(g);

CoordAxes(h_body,.03, .055,.3/2, { [1 0 0], [0 1 0], [0 0 1]},1,.75 )

draw_ellipse( h_body , [0 0 0]', PseudoInertia, [.3 .3 .3 ; 1 1 1], 1 );

d = 2;
strengthOut = .3;
strengthUp  = .5;
light('Position',[-d 0 d],'Style','infinite','Color',[1 1 1]*strengthOut);
light('Position',[d 0 d],'Style','infinite','Color',[1 1 1]*strengthOut);
light('Position',[0 0 d],'Style','infinite','Color',[1 1 1]*strengthUp);
light('Position',[0 0 -d],'Style','infinite','Color',[1 1 1]*strengthUp);




xlim([-1.2 1.1]);
ylim([-1.1 1.1]);
zlim([-1.1 1.1]);
view([47 30])

while t < tf
    q = x(1:4);
    w = x(5:end);
    R = quatToRot(q);
    
    h_body.Matrix = [R zeros(3,1); 0 0 0 1];
    
    
    delta_quat = quatProduct( quatConj(q_bar) , q );
    delta_cayl  = quatToCayley(delta_quat);
    
    delta_w = w-w_bar;
    
    delta_x_cayl = [ delta_cayl ; delta_w ];
    u = u_bar - K * delta_x_cayl;
    
    x = fquat_fun(x, u);
    t = t+dt;
    pause(dt);
end




