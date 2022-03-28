function path = make_path(shapeName)
    switch shapeName
        case 'square'
            edge1 = [linspace(0.30, 0.50, 10);
                linspace(-0.10, -0.10, 10);
                0.6 * ones(1,10)];
            
            edge2 = [linspace(0.50, 0.50, 10);
                linspace(-0.10, 0.10, 10);
                0.6 * ones(1,10)];
            
            edge3 = [linspace(0.50, 0.30, 10);
                linspace(0.10, 0.10, 10);
                0.6 * ones(1,10)];
            
            edge4 = [linspace(0.30, 0.30, 10);
                linspace(0.10, -0.10, 10);
                0.6 * ones(1,10)];
            
            path = [edge1, edge2, edge3, edge4];
            
        case 'spiral'
            theta = linspace(0, 8*pi, 40);
            r = linspace(0, 0.2, 40);
            x = r .* cos(theta) + 0.2 ;
            y = r .* sin(theta);
            z = 0.6 * ones(1,40);
            
            path = [x; y; z];
            
        case 'circle'
            theta = linspace(0, 2*pi, 40);
            r = 0.2;
            x = 0.5 * ones(1,40);
            y = r .* sin(theta);
            z = r .* cos(theta) + 0.6 ;
            
            path = [x; y; z];
            
        case 'infinity'
            theta = linspace(-pi/2, 3*pi/2, 40);
            r = 0.3;
            x = [linspace(0.5, 0.3, 20) linspace(0.3, 0.1, 20)];
            y = r .* sin(theta);
            z = r .* cos(theta) .* sin(theta) + 0.6 ;
            
            path = [x; y; z];
            
        otherwise
            error('No such shape found!');
    end
end