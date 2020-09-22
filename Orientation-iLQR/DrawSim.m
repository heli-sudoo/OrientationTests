function DrawSim( q_des, x,u, dt,PseudoInertia )
    q = x(1:4,:);
    % Pretty pictures
    figure(1);
    clf;
    g = axes();

    
    % Earth coordinate frame (grayish and skinny)
    CoordAxes(g,.05/5, .1/2/5,.3/5, { [1 1 1]*.6, [1 1 1]*.6, [1 1 1]*.6},1,1 );

    h_des = hgtransform(g);
    h_des.Matrix = [quatToRot(q_des) zeros(3,1) ; 0 0 0 1];

    % Desired coordinate frame (darker than regular one)
    CoordAxes(h_des,.05/2, .1/2,.3/2, { [1 0 0]*.5, [0 1 0]*.5, [0 0 1]*.5},1,1 );

    h_body = hgtransform(g);

    % Body fixed coordinate frame
    CoordAxes(h_body,.03, .055,.3/2, { [1 0 0], [0 1 0], [0 0 1]},1,.75 );
    
    % Torque Arrow
    arrow = ArrowGraphic(h_body,0.03,0.055, .3/2, [255 105 180]/255 ,1);
                         
    draw_ellipse( h_body , [0 0 0]', PseudoInertia, [.3 .3 .3 ; 1 1 1], 1 );

    % lighting 
    d = 2; % distance camera is away from origin
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

    % Carry out the simulation

    for ix = 1:size(x,2)-1

        % Update the graphics by setting the homogeneous tranform for the body
        % frame
        h_body.Matrix = [quatToRot(q(:,ix)) zeros(3,1); 0 0 0 1];
        h_des.Matrix = [quatToRot(q_des) zeros(3,1) ; 0 0 0 1];
        
        nu = norm(u(:,ix));
        if nu < 1e-6
            nu = 1e-6;
        end
              
        % update torque arrow. Use a logrithmic scale for arrow length
        arrow.setVec( u(:,ix)/nu*log(nu+1)/4 );
        
        % deviation from nominal state
        pause(dt);
    end
end




