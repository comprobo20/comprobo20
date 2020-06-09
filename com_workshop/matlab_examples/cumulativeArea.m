function [level, xcob, ycob] = cumulativeArea(points,desiredArea)
    function [area, xcentroid, ycentroid] = areaHelper(level, computeCentroid)
        if nargin < 2
            computeCentroid = false;
        end
        area = 0;
        xcentroid = 0;
        ycentroid = 0;

        for i = 1 : size(edges,1)
            deltaY = level - edges(i,5);
            % make sure it's active
            deltaY = max(0,deltaY);
            % don't exceed the tops
            deltaY = min(deltaY,edges(i,6)-edges(i,5));
            area = area + 0.5*deltaY^2*f(i,1)+deltaY*f(i,2);
            if computeCentroid
                xcentroid = xcentroid + 1/6*deltaY^3*g(i,1)+1/2*deltaY^2*g(i,2)+deltaY*g(i,3);
                ycentroid = ycentroid + 1/6*deltaY^3*h(i,1)+1/2*deltaY^2*h(i,2)+deltaY*h(i,3);
            end
        end
        if computeCentroid
            xcentroid = xcentroid/area;
            ycentroid = ycentroid/area;
        end
    end
    edges = [points(1:end-1,:) points(2:end,:)];
    edges(:,end+1) = min([edges(:,2) edges(:,4)]')';
    edges(:,end+1) = max([edges(:,2) edges(:,4)]')';
    % area = 0.5*a*delta_y^2+b*delta_y (a,b are 1,2 columns of f)
    f = zeros(size(edges,1),2);
    % xcom = 1/6*a*delta_y^3+b*delta_y^2/2+c*delta_y (a,b,c are 1,2,3
    % columns of g)
    g = zeros(size(edges,1),3);
    % ycom = 1/6*a*delta_y^3+b*delta_y^2/2+c*delta_y (a,b,c are 1,2,3
    % columns of h)
    h = zeros(size(edges,1),3);
    for i = 1 : size(edges,1)
        if edges(i,4) > edges(i,2)
            f(i,1) = (edges(i,3)-edges(i,1))/(edges(i,4)-edges(i,2));
            f(i,2) = edges(i,1);
            g(i,1) = f(i,1)^2;
            g(i,2) = f(i,1)*f(i,2);
            g(i,3) = f(i,2)^2/2;
            slope = f(i,1);
            start = edges(i,1);
            zstart = edges(i,2);
            h(i,1) = 2*slope;
            h(i,2) = start+slope*zstart;
            h(i,3) = start*zstart;
        elseif edges(i,4) < edges(i,2)
            f(i,1) = -(edges(i,3)-edges(i,1))/(edges(i,4)-edges(i,2));
            f(i,2) = -edges(i,3);
            g(i,1) = -f(i,1)^2;
            g(i,2) = -f(i,1)*f(i,2);
            g(i,3) = -f(i,2)^2/2;
            slope = -f(i,1);
            start = edges(i,3);
            zstart = edges(i,4);
            h(i,1) = -2*slope;
            h(i,2) = -start-slope*zstart;
            h(i,3) = -start*zstart;
        end % ignore horizontal edges
    end
    lb = min(points(:,2));
    ub = max(points(:,2));
    if isinf(desiredArea)
        level = ub;
        [levelArea,xcob,ycob] = areaHelper(level,true);
        return;
    end
    if areaHelper(ub) < desiredArea
        % impossible
        return
    end
    levelArea = Inf;    % ensure we always run the loop once by initializing to large value
    % TODO might be able to optimize further with sorting and bookkeeping
    while abs(levelArea-desiredArea)>10^-4
        level = (ub + lb)/2;
        levelArea = areaHelper(level);
        if levelArea > desiredArea
            ub = level;
        else
            lb = level;
        end
    end
    [levelArea,xcob,ycob] = areaHelper(level,true);
end