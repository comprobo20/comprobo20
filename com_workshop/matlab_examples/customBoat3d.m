% TODO: is it possible there are some degenerate meshes still being used?
% [Wrn] [MeshManager.cc:1343] Ignoring edge without 2 distinct vertices

function customBoat3d()
    function closeRequest(src,callbackdata)
        % Close request function 
        % to display a question dialog box
        % get rid of subscriptions to avoid race conditions
        clear pauseSvc;
        clear updateModelSvc;
        delete(gcf)
    end
    function setDensityRatio(hObject, eventdata, handles)
        % callback function for the density ratio slider
        densityRatio = get(hObject, 'Value');
        % the boat has not been spawned in the current configuration
        spawned = false;
        plotAVSCurve(allPoints);
    end

    function setballastLevel(hObject, eventdata, handles)
        % callback function for the set ballast level slider
        ballastLevel = get(hObject, 'Value');
        % the boat has not been spawned in the current configuration
        spawned = false;
        % redraw the ballast line
        set(b,'XData',[minX maxX],'YData',[ballastLevel ballastLevel]);
        plotAVSCurve(allPoints);
    end

    function [torque,d,CoM] = getWaterLineHelper(theta, CoM, densityRatioWeightedVolume, allSlices, deltaZ)
        mass = densityRatioWeightedVolume*1000;%Kg
        volume_displaced_at_eq = densityRatioWeightedVolume;

        % define the direction of gravity
        down = [0, 0, -1];
        rotatedSlices = {};
        for i = 1 : length(allSlices)
            points = allSlices{i};
            % rotate the boat about the origin to match the Gazebo simulator
            yy = cosd(theta)*points(:,1) + sind(theta)*points(:,2);
            zz = - sind(theta)*points(:,1) + cosd(theta)*points(:,2);
            rotatedSlices{i} = [yy zz];
        end
 
        [d,ycob,zcob] = cumulativeVolume(rotatedSlices,deltaZ,volume_displaced_at_eq);
        % find the torque
        rotated_com = [cosd(theta) sind(theta);...
                       -sind(theta) cosd(theta)]*CoM';
        torque = cross([0,ycob-rotated_com(1),zcob-rotated_com(2)],-down*mass*9.8);%Nm
        torque = torque(1);
    end

    function [regions isBallast areas] = splitHullAtBallastLevel(poly)
        polySplit = addboundary(poly,[minX,maxX,maxX,minX,minX],[ballastLevel,ballastLevel,ballastLevel+10^-5,ballastLevel+10^-5,ballastLevel]);
        r = polySplit.regions;
        regions = {};
        isBallast = [];
        areas = [];
        for i = 1 : length(r)
            % this if statement prevents adding regions that are
            % either not in the boat (e.g., the boundary doesn't
            % intersect the boat) or too small.  The factor of 2 is
            % a fudge factor since the area was often close to what
            % we would predict but just a little bit larger

            if r(i).area > 2*(maxX - minX)*10^-5
                % make sure boundary is counterclockwise by
                % checking if the area is positive
                vertices = [r(i).Vertices; r(i).Vertices(1,:)];
                area = 0;
                for j = 1:size(vertices,1)-1
                    area = area + 0.5*(vertices(j,1)*vertices(j+1,2)-vertices(j+1,1)*vertices(j,2));
                end
                if area < 0
                    % polygon area was clockwise, make it counter
                    % clockwise
                    vertices = vertices(end:-1:1,:);
                    area = -area;
                end
                areas(end+1) = area;
                % check if the region is in the ballast zone or not
                % (10^-5 is a fudge factor)
                if max(vertices(:,end))<=ballastLevel+10^-5
                    isBallast(end+1) = true;
                else
                    isBallast(end+1) = false;
                end
                regions{end+1} = vertices;
            end
        end
    end

    function polyChopped = chopOffAreaAboveDeck(poly)
        deckLevel = maxY;
        polySplit = addboundary(poly,[minX,maxX,maxX,minX,minX],[deckLevel,deckLevel,deckLevel+10^-5,deckLevel+10^-5,deckLevel]);
        regions = polySplit.regions;
        for r=1:length(regions)
            if abs(deckLevel-max(regions(1).Vertices(:,2)))<10^-6
                polyChopped = regions(r);
                return
            end
        end
        % return an empty polygon (e.g., if the loft reduces the polygon to
        % nothing
        polyChopped = polyshape();
    end

    function CoM = plotAVSCurve(allPoints)
        if size(allPoints,1) <= 3
            return
        end
        torques = zeros(size(thetas));
        waterlines = zeros(size(thetas));
        poly = polyshape(allPoints);
        if poly.NumRegions ~= 1 | poly.NumHoles ~= 0
            disp("Can't spawn model as it has holes or intersections");
            disp("Use the 'r' key to reset");
            return
        end
        totalWeightedVolume = 0;
        deltaz = Z(2)-Z(1);
        % right now we are just worried about the 2D CoM (could add 3D)
        CoM = [0 0];
        allSlices = {};
        for slice=1:length(Z)
            polyLofted = poly.translate(0, loftCurve(slice));
            polyChopped = chopOffAreaAboveDeck(polyLofted);
            if polyChopped.area == 0
                continue
            end
            % make sure boundary is counterclockwise by
            % checking if the area is positive
            vertices = [polyChopped.Vertices; polyChopped.Vertices(1,:)];
            area = 0;
            for j = 1:size(vertices,1)-1
                 area = area + 0.5*(vertices(j,1)*vertices(j+1,2)-vertices(j+1,1)*vertices(j,2));
            end
            if area < 0
                % polygon area was clockwise, make it counterclockwise
                vertices = vertices(end:-1:1,:);
                area = -area;
            end
            allSlices{end+1} = vertices;
            
            [regions isBallast areas] = splitHullAtBallastLevel(polyChopped);
            for i = 1 : length(isBallast)
                [~,xcomRegion,ycomRegion] = cumulativeArea(regions{i},Inf); % Inf means use the whole polygon
                if isBallast(i)
                    regionDensity = ballastDensityRatio;
                else
                    regionDensity = densityRatio;
                end
                CoM = CoM + [xcomRegion ycomRegion]*areas(i)*regionDensity*deltaz;
                totalWeightedVolume = totalWeightedVolume + areas(i)*regionDensity*deltaz;
            end
        end
        CoM = CoM / totalWeightedVolume;

        for i = 1:length(thetas)
            [torques(i),waterlines(i)] = getWaterLineHelper(thetas(i),CoM,totalWeightedVolume,allSlices,deltaz);
        end
        set(CoMPlot,'XData',CoM(1),'YData',CoM(2));
        set(avsPlot,'YData',torques);
    end
    function avsClick(axesObj, eventDat)
        if ~spawned
            spawnBoat();
            pause(2);
        end
        x = eventDat.IntersectionPoint(1);
        deltaz = Z(2)-Z(1);
        [~,waterline,~] = getWaterLineHelper(x,CoM,totalWeightedVolume,allSlices,deltaz);

        msg = rosmessage(updateModelSvc);
        msg.ModelState.ModelName = 'customboat';
        msg.ModelState.Pose.Position.Z = -waterline;
        quat = eul2quat([0 deg2rad(x) pi/2]);
        msg.ModelState.Pose.Orientation.W = quat(1);
        msg.ModelState.Pose.Orientation.X = quat(2);
        msg.ModelState.Pose.Orientation.Y = quat(3);
        msg.ModelState.Pose.Orientation.Z = quat(4);
        call(updateModelSvc, msg);
        disp(['Selected angle ',num2str(x),' degrees']);
    end

    function mouseDownFunction(axesObj, eventDat)
        x = eventDat.IntersectionPoint(1);
        y = eventDat.IntersectionPoint(2);
        if x < 0
            disp('Only clicks on the right side.  The left will be mirrored');
            return;
        end
        % add the new point to the hull
        userPoints = [userPoints; [x y]];
        updateAllPoints();
        % the boat has not been spawned in the current configuration
        spawned = false;
        poly = polyshape(allPoints);
        if poly.NumRegions ~= 1 | poly.NumHoles ~= 0
            disp('Illegal polygon.  No intersections or duplicate vertices are allowed');
        end
        set(p,'XData',allPoints(:,1),'YData',allPoints(:,2));
        set(s,'XData',allPoints(:,1),'YData',allPoints(:,2));
        plotAVSCurve(allPoints);
    end

    function updateAllPoints()
        if ~isempty(userPoints)
            mirroredUserPoints = [-userPoints(:,1) userPoints(:,2)];
        else
            mirroredUserPoints = userPoints;
        end
        allPoints = [startPoint; userPoints; endPoint; mirroredUserPoints(end:-1:1,:); startPoint];
    end

    function spawnBoat(shouldPause)
        if nargin < 1
            shouldPause = false;
        end
        poly = polyshape(allPoints);
        if poly.NumRegions ~= 1 | poly.NumHoles ~= 0
            disp("Can't spawn model as it has holes or intersections");
            disp("Use the 'r' key to reset");
            return
        end
        % spawn the model
        doc = com.mathworks.xml.XMLUtils.createDocument('sdf');
        sdfElem = doc.getDocumentElement();
        sdfElem.setAttribute('version', '1.5');
        modelElem = doc.createElement('model');
        sdfElem.appendChild(modelElem);
        modelElem.setAttribute('name', 'customboat');
        allRegions = {};
        allMaterials = {};
        allComponentDensityRatios = [];
        allZs = [];
        for slice=1:length(Z)
            polyLofted = poly.translate(0, loftCurve(slice));
            polyChopped = chopOffAreaAboveDeck(polyLofted);
            if polyChopped.area == 0
                continue
            end
            [regions isBallast ~] = splitHullAtBallastLevel(polyChopped);
            allRegions = [allRegions regions];
            for i = 1 : length(regions)
                if isBallast(i)
                    componentDensityRatio = ballastDensityRatio;
                    material = 'Gazebo/Red';
                else
                    componentDensityRatio = densityRatio;
                    material = 'Gazebo/Gray';
                end
                allComponentDensityRatios(end+1) = componentDensityRatio;
                allMaterials{end+1} = material;
                allZs(end+1) = Z(slice);
            end
        end
        polyline(doc, modelElem, 'customBoat', allComponentDensityRatios, Z(2)-Z(1), allRegions, allMaterials, true, allZs);
        if shouldPause
            msg = rosmessage(pauseSvc);
            call(pauseSvc, msg);
        end
        % setting a yaw of pi/2 causes the boat to orirent in an upright stance
        spawnModel('customboat',xmlwrite(sdfElem), 0, 0, 1, pi/2, 0, 0, true);
        spawned = true;
    end

    function keyPressedFunction(fig_obj, eventDat)
        ck = get(fig_obj, 'CurrentKey');
        switch ck
            case 's'
                spawnBoat(true);
            case 'r'
                userPoints = [];
                % the boat has not been spawned in the current configuration
                spawned = false;
                updateAllPoints();
                set(p,'XData',allPoints(:,1),'YData',allPoints(:,2));
                set(s,'XData',allPoints(:,1),'YData',allPoints(:,2));
         end
    end
    % track whether the boat has been spawned in Gazebo
    spawned = false;
    % y-limits of hull cross section visualization
    minY = -0.5;
    maxY = 1.0;
    % x-limits of hull cross section visualization
    minX = -2;
    maxX = 2;
    % the ratio of density of non-ballast mateiral to water
    densityRatio = 0.25;
    % the hull below this level is ballast
    ballastLevel = minY;
    % density ratio of ballast is 1.24 (solid PLA)
    ballastDensityRatio = 1.24;
    % the boat length (this is the extrusion dimension)
    boatLength = 2;
    
    longitudinalShapeParameter = 2;
    Z = linspace(-boatLength/2, boatLength/2, 50);
    Z = Z - (Z(2)-Z(1))/2;  % maintain a Z center of mass of 0

    loftCurve = abs(2.*(Z + (Z(2)-Z(1))/2)./boatLength).^longitudinalShapeParameter;
    
	f = figure('CloseRequestFcn',@closeRequest);
    subplot(7,1,4:6);
    % used to pause the simulation when a new boat is spawned
    pauseSvc = rossvcclient('/gazebo/pause_physics');
    % used for putting the boat at various angles
    updateModelSvc = rossvcclient('gazebo/set_model_state');
    % populate the hull with these two points
    startPoint = [0 0];
    endPoint = [0 1];
    userPoints = [];
    allPoints = [startPoint; endPoint];
    allSlices = {};
    % plot the vertices
    s = scatter(allPoints(:,1), allPoints(:,2),'MarkerFaceColor', uint8([128 128 128]), 'MarkerEdgeColor', uint8([64 64 64]));
    hold on;
    % plot the edges
    p = plot(allPoints(:,1), allPoints(:,2),'k');
    % draw a line for the ballast level
    b = plot([minX maxX],[ballastLevel ballastLevel],'k');
    % track the center of mass
    CoM = [(startPoint(1)+endPoint(1))/2, (startPoint(2)+endPoint(2))/2];
    % this is the cross-sectional area weighted by the density of each part
    % of the cross section.  The boat's mass is L*totalWeightedArea*1000
    totalWeightedVolume = 0;

    % show the CoM
    CoMPlot = plot(CoM(1),CoM(2),'b.');
    set(CoMPlot,'MarkerSize',25);

    xlabel('x (m)');
    ylabel('y (m)');
    xlim([minX maxX]);
    ylim([minY maxY]);
    % put a translucent red patch over the left half of the figure to
    % indicate to the user not to click there
    redPatch = patch([minX 0 0 minX]', [minY minY maxY maxY]', [1 0.5 0.5]);
    alpha(redPatch, 0.5);
    % these commands can be executed by pressing the appropriate key when
    % the focus is on the figure window
    title('s for "spawn", r for "reset".  Crossing lines are not permitted.');
    daspect([1 1 1]);
    % add a labeled slider for the density ratio
    sld = uicontrol('Style', 'slider',...
        'Min',0.01,'Max',0.99,'Value',densityRatio,...
        'Position', [20 20 120 20],...
        'Callback', @setDensityRatio);
    txt = uicontrol('Style','text',...
        'Position',[20 45 120 20],...
        'String','Density Ratio');
    % add a labeled slider for the ballast level
    sld = uicontrol('Style', 'slider',...
        'Min',minY,'Max',maxY,'Value',ballastLevel,...
        'Position', [160 20 120 20],...
        'Callback', @setballastLevel);
    % Add a text uicontrol to label the slider.
    txt = uicontrol('Style','text',...
        'Position',[160 45 120 20],...
        'String','ballast Level');
    % listen for input events
    set(f,'WindowKeyPressFcn', @keyPressedFunction);
    set(gca,'ButtonDownFcn',@mouseDownFunction,'HitTest','on');
    % plot the AVS curve
    thetas = linspace(0,180,1800+1);
    subplot(7,1,1:2);
    avsPlot = plot(thetas,zeros(size(thetas)));
    % when the user clicks the AVS plot, the boat is placed at that angle
    set(gca,'ButtonDownFcn',@avsClick,'HitTest','on');
    hold on;
    % draw a red line to make the AVS easier to see
    plot(thetas,zeros(size(thetas)),'r');
    xlabel('heel angle (degrees)');
    ylabel('torque');
end
