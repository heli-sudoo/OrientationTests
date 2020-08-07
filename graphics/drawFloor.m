function drawFloor(parent)
    % Create the ground. It reaches from -5 to +5
    v = []; f = []; c = [];
    % Number of tiles on each side of the origin
    nTilesX = 8;  
    nTilesZ = 8;  
    % Size of the tiles
    sTiles = 1;  
    % Create vertices:
    for i = -sTiles*nTilesX:sTiles:sTiles*nTilesX
        for j = -sTiles*nTilesZ:sTiles:sTiles*nTilesZ
            v = [v;[i,j,0]];                         
        end
    end
    % Connect them and color them as checkerboard:
    for i = 1:2*nTilesZ
        for j = 1:2*nTilesX
            f = [f;[i,i+1,i+2+2*nTilesZ,i+1+2*nTilesZ]+(j-1)*(2*nTilesZ+1)];	% Connect vertices
            if mod(j+i,2)==0;	% Color faces alternating
                c = [c;[1,1,1]];
            else
                c = [c;[.65,.65,.65]];
            end
        end
    end

    p = patch('Parent',parent,'faces', f, 'vertices', v, 'FaceVertexCData', c,  'FaceColor','Flat',...
       'EdgeColor', 'none', ...
       'FaceLighting', 'gouraud', ...
       'BackFaceLighting', 'unlit');