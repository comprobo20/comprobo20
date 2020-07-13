% points = [0 0; 1 0; 2 1; 0 2; -2 1; -1 0; 0 0]
% chopPolygons(points, [0.5, 1]);
function polygonQueue = chopPolygon(points, verticalCuts)
% put lowest point first since we assume the first point is below vertical cuts
pointsTmp = points(1:end-1,:);
[minlevel, minind] = min(pointsTmp(:,2));
points = [pointsTmp(minind:end,:); pointsTmp(1:minind,:)];
currRegion = 0;
polygonQueue = cell(length(verticalCuts)+1,1);
for i = 1:size(polygonQueue,1)
    polygonQueue{i} = zeros(0,2);
end
polygonQueue{1} = points(1,:);
for i = 2:size(points,1)
    if points(i,2) > points(i-1,2)
        inds = find(points(i,2)>verticalCuts);
        if isempty(inds)
            nextRegion = 0;
        else
            nextRegion = max(inds);
        end
        crossings = currRegion+1:nextRegion;
    elseif points(i,2) < points(i-1,2)
        inds = find(points(i,2)<verticalCuts);
        if isempty(inds)
            nextRegion = length(verticalCuts);
        else
            nextRegion = min(inds)-1;
        end
        crossings = currRegion:-1:nextRegion+1;
    else
        nextRegion = currRegion;
        crossings = [];
    end
    for j=1:length(crossings)
        slope = (points(i,2)-points(i-1,2))/(points(i,1)-points(i-1,1));
        deltaY = verticalCuts(crossings(j)) - points(i-1,2);
        deltaX = deltaY/slope;
        crossingCoordinate = [points(i-1,1)+deltaX, verticalCuts(crossings(j))];
        polygonQueue{crossings(j)}(end+1,:) = crossingCoordinate;
        polygonQueue{crossings(j)+1}(end+1,:) = crossingCoordinate;
    end
    polygonQueue{nextRegion+1}(end+1,:) = points(i,:);
    currRegion = nextRegion;
end
% close the other queues
for i=2:size(polygonQueue,1)
    if size(polygonQueue{i,1},1) > 0
        polygonQueue{i,1}(end+1,:) = polygonQueue{i,1}(1,:);
    end
end
end