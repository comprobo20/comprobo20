function [level, xcob, ycob] = cumulativeArea(points,desiredArea)
% cumulativeArea  Given the polygon defined by vertices in points (assumed
% to be arranged counter-clockwise and form a closed path (i.e.
% the initial and final vertices should be the same), and a desiredArea,
% compute level, which defines the halfspace y <= level, such that the
% intersection of the polygon and this halfspace has area equal to
% desiredArea.  Also returned are the centroid of the area of this
% intersection.
%
% If desiredArea has the special value of Inf, the level will be
% automatically set to the maximum y-value of the vertices of the polygon.
%
% If desiredArea is finite and greater than the area of the polygon, no
% values will be returned from the function.
%
% EXAMPLES:
%
%   Compute the level where the intersection of a unit square and the
%   halfspace equals 0.5.
%
%   [level, xcob, ycob] = cumulativeArea([0, 0; 0 1; 1 1; 1 0; 0 0],0.5)
%   This yields level = 0.5, xcob = 0.5, ycob = 0.25
%
%   Compute the level where the intersection of an equilateral triangle of
%   side length 1 and the halfspace is 0.2.
%
%   [level, xcob, ycob] = cumulativeArea([0, 0;...
%                                         1 0;
%                                         0.5 sqrt(3)/2;
%                                         0 0],0.2)
%   This yields level = 0.2307, xcob = 0.5, ycob = 0.1094
    
    % construct edges from vertices
    edges = [points(1:end-1,:) points(2:end,:)];
    
    % keep track of the top and bottom of each edge
    edges(:,end+1) = min([edges(:,2) edges(:,4)],[],2);
    edges(:,end+1) = max([edges(:,2) edges(:,4)],[],2);

    % To compute the intersection of the polygon and any halfspace we have
    % to integrate along the sides of the polygon for any sides that are at
    % least partially below the halfspace and across the line that lies on
    % the boundary of the halfspace.
    %
    % Here are the relevant integrals for a given edge (x1,y1) to (x2,y2)
    % Let:
    %
    % b be the y-coordinate of the bottom the edge (i.e., min(y1,y2))
    % u be the y-coordinate of the top of a given edge (i.e., max(y1,y2))
    % v be the x-coordinate of the bottom vertex
    % h be the boundary of the halfspace (y <= h)
    % delta_y be the difference of h and the bottom of the edge capped so
    % that it is non-negative and never exceeds b-u (min(max(h-b,0),u-b))
    % invslope = (x2 - x1)/(y2 - y1) (note: run / rise)
    %
    % Note: some of this could probably be condensed, but I wanted to err
    % on the side of being exhaustive.
    %
    % Area:
    %  Horizontal edges (y1 = y2):
    %    0
    %
    %  Edges that point upward (y2 > y1)
    %    int_{0}^{delta_y} x dy
    %      = int_{0}^{delta_y} v + invslope*y dy
    %      = v*delta_y + 1/2*invslope*delta_y^2
    %
    %  Edges that point downward (y2 < y1)
    %    int_{delta_y}^{0} x dy
    %      = -int_{0}^{delta_y} v + invslope*y dy
    %      = -v*delta_y - 1/2*invslope*delta_y^2
    %
    % Computing the x centroid (need to also divide by area)
    %  Horizontal edges:
    %    0
    %
    %  Edges that point upward
    %    int_{0}^{delta_y} 1/2*x^2 dy
    %      = int_{0}^{delta_y} 1/2*(v + invslope*y)^2 dy
    %      = 1/6*delta_y^3*invslope^2 + 1/2*delta_y^2*invslope*v
    %               + 1/2*delta_y^2*v^2
    %
    %  Edges that point downward
    %    int_{delta_y}^{0} 1/2*x^2 dy
    %      = -int_{0}^{delta_y} 1/2*(v + invslope*y)^2 dy
    %      = -1/6*delta_y^3*invslope^2 - 1/2*delta_y^2*invslope*v
    %               - 1/2*delta_y^2*v^2
    %
    % Computing the y centroid (need to also divide by area)
    %  Horizontal edges:
    %    0
    %
    %  Edges that point upward
    %    int_{0}^{delta_y} x*y dy
    %      = int_{0}^{delta_y} (v + invslope*y)*(y + b) dy
    %      = 1/3*delta_y^3*invslope + 1/2*delta_y^2*(v + b*invslope))
    %               + b*v*delta_y
    %
    %  Edges that point downward
    %    int_{delta_y}^{0} x*y dy
    %      = -int_{0}^{delta_y} (v + invslope*y)*(y + b) dy
    %      = -1/3*delta_y^3*invslope - 1/2*delta_y^2*(v + b*invslope))
    %               - b*v*delta_y

    % area_i = 1/2*f(i,1)*delta_y^2+f(i,2)*delta_y
    f = zeros(size(edges,1),2);
    % xcom_i = 1/6*g(i,1)*delta_y^3+1/2*g(i,2)*delta_y^2+g(i,3)*delta_y
    g = zeros(size(edges,1),3);
    % ycom_i = 1/6*h(i,1)*delta_y^3+1/2*h(i,2)*delta_y^2+h(i,3)*delta_y
    h = zeros(size(edges,1),3);
    for i = 1 : size(edges,1)
        if edges(i,4)>edges(i,2) % rising edge
            b = edges(i,2);
            v = edges(i,1);
            s = 1;      % used to negate the integrals if needed
        elseif edges(i,4)<edges(i,2) % falling edge
            b = edges(i,4);
            v = edges(i,3);
            s = -1;     % used to negate the integrals if needed
        else % ignore horizontal edges
            continue
        end
        invslope = (edges(i,3)-edges(i,1))/(edges(i,4)-edges(i,2));
        % plug in expressions from commented sections above
        f(i,1) = s*invslope;
        f(i,2) = s*v;

        g(i,1) = s*invslope^2;
        g(i,2) = s*invslope*v;
        g(i,3) = s*0.5*v^2;

        h(i,1) = s*2*invslope;
        h(i,2) = s*(v+b*invslope);
        h(i,3) = s*b*v;
    end
    lb = min(points(:,2));
    ub = max(points(:,2));
    if isinf(desiredArea)
        level = ub;
        [~,xcob,ycob] = areaHelper(level,true);
        return;
    end
    if areaHelper(ub) < desiredArea
        % impossible
        return
    end
    
    % Use binary search to find the halfspace the yields the appropriate
    % area when intersected with the polygon
    % (https://en.wikipedia.org/wiki/Binary_search_algorithm)
    levelArea = Inf;    % ensure we always run the loop once by initializing to large value
    % TODO might be able to optimize further with sorting and bookkeeping
    while abs(levelArea - desiredArea) > 10^-4
        level = (ub + lb)/2;
        levelArea = areaHelper(level);
        if levelArea > desiredArea
            ub = level;
        else
            lb = level;
        end
    end
    % compute the centroid of the intersection area
    [~,xcob,ycob] = areaHelper(level,true);
    
    function [area, xcentroid, ycentroid] = areaHelper(level, computeCentroid)
        % A helper function that computes the area and centroid of the
        % polygon intersected with the halfspace y <= level.
        %
        % If computeCentroid is false, then the centroid will not be
        % computed.  This is helpful for computational reasons when you
        % don't need the centroid (e.g., when you are finding the
        % waterline).
        if nargin < 2
            computeCentroid = false;
        end

        area = 0;
        xcentroid = 0;
        ycentroid = 0;

        for i = 1 : size(edges,1)
            % we first check how far along the edge we are integrating.
            % edges(i,5) is the y-coordinate of the bottom of the edge.  
            deltaY = level - edges(i,5);
            % If deltaY is negative, that means the level did not reach the
            % edge and thus it is not part of the curve we are integrating.
            deltaY = max(0,deltaY);
            % We can only integrate up to the top edge  
            deltaY = min(deltaY, edges(i,6) - edges(i,5));
            % the integral for area is quadratic in deltaY
            area = area + 1/2*deltaY^2*f(i,1) + deltaY*f(i,2);
            if computeCentroid
                % the integrals for centroid are cubic in deltaY
                xcentroid = xcentroid + 1/6*deltaY^3*g(i,1) + 1/2*deltaY^2*g(i,2) + deltaY*g(i,3);
                ycentroid = ycentroid + 1/6*deltaY^3*h(i,1) + 1/2*deltaY^2*h(i,2) + deltaY*h(i,3);
            end
        end
        if computeCentroid
            xcentroid = xcentroid/area;
            ycentroid = ycentroid/area;
        end
    end
end