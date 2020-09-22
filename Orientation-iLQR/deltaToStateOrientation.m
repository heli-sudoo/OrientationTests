function x = deltaToStateOrientation(x_bar, delta)
    
    w_bar = x_bar(5:7);
    q_bar = x_bar(1:4);
   
    delta_cayl  = delta(1:3);
    delta_w     = delta(4:6);
    
    q = quatProduct( q_bar, cayleyToQuat(delta_cayl) );
    w = w_bar + delta_w;
    
    x = [q;w];
end
