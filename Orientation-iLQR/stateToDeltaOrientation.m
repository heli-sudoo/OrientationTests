function delta = stateToDeltaOrientation(x,x_bar)
    q = x(1:4);
    w = x(5:7);

    w_bar = x_bar(5:7);
    q_bar = x_bar(1:4);
    delta_quat = quatProduct( quatConj(q_bar) , q );
    delta_cayl  = quatToCayley(delta_quat);
    delta_w = w-w_bar;
    delta = [ delta_cayl ; delta_w ];
end
