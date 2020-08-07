function G = quatG(q)
    H = [0 0 0 ; eye(3)];
    G = quatL(q)*H;
end
