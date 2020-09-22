function DebugRollouts(problem_data, x_bar, u_bar, delta_u, K )

    ep_list = linspace(0,1,150);
    V_list = 0*ep_list;
    for i =1:size(ep_list,2)
        epsilon = ep_list(i);
        [~,~, V_new] = ForwardRollout(x_bar, u_bar+epsilon*delta_u , K, problem_data);
        V_list(i) = V_new;
    end
    figure(500); clf;
    plot(ep_list, V_list,'LineWidth',4); hold on;
    plot(ep_list, V_old + ep_list.*(2-ep_list)*p0(1),'r-.','LineWidth',2);
    
    legend('Actual','Expected');
    
    yy = ylim();
    ylim([yy(1) (V_old-yy(1))*1.1+yy(1)]);  
    input('?')
    %return
    
    pause(0.1);
end
