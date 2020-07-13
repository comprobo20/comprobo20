% Relevant links to send to 3D print service
%   Switch infill at z value
%   (https://community.ultimaker.com/topic/9167-is-there-a-way-to-change-infill-percentage-at-a-certain-height/)
%   Different infill in the same part
%   (https://3dprinting.stackexchange.com/questions/6522/different-infill-in-the-same-part)
% TODO:
%   - The mesh fails the watertight check, might need to validate it against
%       this criterion: https://axom.readthedocs.io/en/develop/axom/quest/docs/sphinx/check_and_repair.html
%       The most likely culprit is the interface between the deck of the boat
%       and the hull (the triangles used on the sides to form the interface to
%       the deck are not the same as those used on top).
%   - Disable slicing of boat interior if thickness set to 0 (for
%   computational reasons)
%

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
        mass = densityRatioWeightedVolume*1000; %Kg
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

    function [adjustArea CoMX, CoMY] = adjustForBoatDeck(perimeterCrossSection)
        % This is only an approximation as it doesn't account for overlap
        % between the deck section and the sides
        adjustArea = 0;
        CoMX = 0;
        CoMY = 0;
        for i=1:size(perimeterCrossSection.Vertices,1)
            v1 = perimeterCrossSection.Vertices(i,:);
            v2 = perimeterCrossSection.Vertices(mod(i, size(perimeterCrossSection.Vertices,1))+1,:);
            if v1(2) == maxY && v2(2) == maxY
                % chop off some area
                segmentArea = -abs(v1(1) - v2(1))*0.4/1000*boatLength/exportedBoatLength;
                segmentCoMX = (v1(1) + v2(1))/2;
                segmentCoMY = maxY - perimeterThickness + 0.2/1000*boatLength/exportedBoatLength;
                adjustArea = adjustArea + segmentArea;
                CoMX = CoMX + segmentArea*segmentCoMX;
                CoMY = CoMY + segmentArea*segmentCoMY;
            end
        end
        CoMX = CoMX/adjustArea;
        CoMY = CoMY/adjustArea;
    end

    function vertices = getCCWVertices(myPoly)
        % make sure boundary is counterclockwise by
        % checking if the area is positive
        vertices = [myPoly.Vertices; myPoly.Vertices(1,:)];
        area = 0;
        for j = 1:size(vertices,1)-1
             area = area + 0.5*(vertices(j,1)*vertices(j+1,2)-vertices(j+1,1)*vertices(j,2));
        end
        if area < 0
            % polygon area was clockwise, make it counterclockwise
            vertices = vertices(end:-1:1,:);
            area = -area;
        end
    end

    function plotAVSCurve(allPoints, visualize)
        if nargin < 2
            visualize = false;
        end
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
        totalVolume = 0;
        deltaz = Z(2)-Z(1);
        % right now we are just worried about the 2D CoM (could add 3D)
        CoM = [0 0];
        allSlices = {};
        if visualize
            if isempty(crossSectionFigure)
                crossSectionFigure = figure();
            end
        end
        for slice=1:length(Z)
            loftedPoints = [allPoints(:,1) allPoints(:,2) + loftCurve(slice)];
            partitioned = chopPolygon(loftedPoints, maxY);
            fullCrossSectionPoly = polyshape(partitioned{1});
            boatInterior = polybuffer(fullCrossSectionPoly,-perimeterThickness,'JointType','miter');
            perimeterPolygon = subtract(fullCrossSectionPoly, boatInterior);
            if perimeterPolygon.area ~= 0
                [perimeterCoMX, perimeterCoMY] = perimeterPolygon.centroid;
                CoM = CoM + [perimeterCoMX perimeterCoMY]*perimeterPolygon.area*ballastDensityRatio*deltaz;
                [adjustArea, adjustCoMX, adjustCoMY] = adjustForBoatDeck(perimeterPolygon);
                CoM = CoM + [adjustCoMX adjustCoMY]*adjustArea*ballastDensityRatio*deltaz;
                totalWeightedVolume = totalWeightedVolume + perimeterPolygon.area*ballastDensityRatio*deltaz;
                totalVolume = totalVolume + perimeterPolygon.area*deltaz;
                totalWeightedVolume = totalWeightedVolume + adjustArea*ballastDensityRatio*deltaz
            end
            if visualize
                set(0,'CurrentFigure',crossSectionFigure);
                clf;
                plot(perimeterPolygon);
                xlim([minX maxX]);
                ylim([minY maxY+yBuffer]);
                hold on;
            end

            interiorRegions = boatInterior.regions;
            hullPolys = polyshape.empty;
            ballastPolys = polyshape.empty;
            if min(boatInterior.Vertices(:,2)) < ballastLevel  % some ballast is present in the slice
                for k=1:length(interiorRegions)
                    partitioned = chopPolygon(getCCWVertices(interiorRegions(k)), ballastLevel);
                    ballast = polyshape(partitioned{1});
                    if ballast.area > 0
                        ballastPolys(end+1) = ballast;
                    end
                    hull = polyshape(partitioned{2});
                    if hull.area > 0
                        hullPolys(end+1) = hull;
                    end
                end
            else
                if boatInterior.area > 0
                    hullPolys(end+1) = boatInterior;
                end
            end
            if fullCrossSectionPoly.area == 0
                continue;
            end
            regions = fullCrossSectionPoly.regions;
            for k=1:length(regions)
                allSlices{end+1} = getCCWVertices(regions(k));
            end
            if ~isempty(ballastPolys)
                ballastPoly = union(ballastPolys);
            else
                ballastPoly = polyshape();
            end
            regions = ballastPoly.regions;
            for k=1:length(regions)
                [~,xcomRegion,ycomRegion] = cumulativeArea(getCCWVertices(regions(k)),Inf); % Inf means use the whole polygon
                CoM = CoM + [xcomRegion ycomRegion]*regions(k).area*ballastDensityRatio*deltaz;
                totalVolume = totalVolume + regions(k).area*deltaz;
                totalWeightedVolume = totalWeightedVolume + regions(k).area*ballastDensityRatio*deltaz;
                if visualize
                    set(0,'CurrentFigure',crossSectionFigure);
                    plot(regions(k),'FaceColor','r');
                end
            end
            if ~isempty(hullPolys)
                hullPoly = union(hullPolys);
            else
                hullPoly = polyshape();
            end
            regions = hullPoly.regions;
            for k=1:length(regions)
                [~,xcomRegion,ycomRegion] = cumulativeArea(getCCWVertices(regions(k)),Inf); % Inf means use the whole polygon
                CoM = CoM + [xcomRegion ycomRegion]*regions(k).area*densityRatio*deltaz;
                totalVolume = totalVolume + regions(k).area*deltaz;
                totalWeightedVolume = totalWeightedVolume + regions(k).area*densityRatio*deltaz;
                if visualize
                    set(0,'CurrentFigure',crossSectionFigure);
                    plot(regions(k),'FaceColor','m');
                end
            end
            if visualize
                pause(0.001);
            end
        end
        CoM = CoM / totalWeightedVolume;
        averageInfill = totalWeightedVolume/totalVolume/ballastDensityRatio;
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
        if ~useWithoutGazebo
            msg = rosmessage(updateModelSvc);
            msg.ModelState.ModelName = 'customboat';
            msg.ModelState.Pose.Position.Z = -waterline;
            quat = eul2quat([0 deg2rad(x) pi/2]);
            msg.ModelState.Pose.Orientation.W = quat(1);
            msg.ModelState.Pose.Orientation.X = quat(2);
            msg.ModelState.Pose.Orientation.Y = quat(3);
            msg.ModelState.Pose.Orientation.Z = quat(4);
            call(updateModelSvc, msg);
        end
        disp(['Selected angle ',num2str(x),' degrees']);
    end

    function mouseDownFunction(axesObj, eventDat)
        x = eventDat.IntersectionPoint(1);
        y = eventDat.IntersectionPoint(2);
        if x < 0
            disp('Only clicks on the right side.  The left will be mirrored');
            return;
        end
        % cap at maxY
        y = min(y, maxY);
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
        meshBoatSTLPath = '';
        poly = polyshape(allPoints);
        if poly.NumRegions ~= 1 | poly.NumHoles ~= 0
            disp("Can't spawn model as it has holes or intersections");
            disp("Use the 'r' key to reset");
            return
        end
        if createMeshBoat
            shiftedZs = Z + (Z(2)-Z(1))/2;
            stlFileName = makeLoftedMesh(allPoints(:,1), allPoints(:,2), maxY, shiftedZs, loftCurve, false);
            meshBoatSTLPath = fullfile(pwd,stlFileName)
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
            loftedPoints = [allPoints(:,1) allPoints(:,2) + loftCurve(slice)];
            partitioned = chopPolygon(loftedPoints, maxY);
            fullCrossSectionPoly = polyshape(partitioned{1});
            boatInterior = polybuffer(fullCrossSectionPoly,-perimeterThickness,'JointType','miter');
            perimeterPolygon = subtract(fullCrossSectionPoly, boatInterior);

            if fullCrossSectionPoly.area == 0
                continue
            end
            [xComPerimeter yComPerimeter] = perimeterPolygon.centroid;
            [adjustArea, adjustCoMX, adjustCoMY] = adjustForBoatDeck(perimeterPolygon);
            shiftedCoMX = (xComPerimeter*perimeterPolygon.area + adjustArea*adjustCoMX)/(perimeterPolygon.area+adjustArea);
            shiftedCoMY = (yComPerimeter*perimeterPolygon.area + adjustArea*adjustCoMY)/(perimeterPolygon.area+adjustArea);
            % create a synthetic region to force the polyline function to
            % give the correct mass and center of mass (but the moment of
            % inertia tensor will not be correct)
            % TODO: we would need to be able to deal with moment of inertia
            % of a non-simple polygon (with holes)
            fakeSquare = sqrt(perimeterPolygon.area+adjustArea);
            allRegions{end+1} = [shiftedCoMX - fakeSquare/2 shiftedCoMY - fakeSquare/2;...
                                  shiftedCoMX + fakeSquare/2 shiftedCoMY - fakeSquare/2;...
                                  shiftedCoMX + fakeSquare/2 shiftedCoMY + fakeSquare/2;...
                                  shiftedCoMX - fakeSquare/2 shiftedCoMY + fakeSquare/2;...
                                  shiftedCoMX - fakeSquare/2 shiftedCoMY - fakeSquare/2];
            allComponentDensityRatios(end+1) = ballastDensityRatio;
            allMaterials{end+1} = 'Gazebo/Red';
            allZs(end+1) = Z(slice);
                
            interiorRegions = boatInterior.regions;
            hullPolys = polyshape.empty;
            ballastPolys = polyshape.empty;
            if min(boatInterior.Vertices(:,2)) < ballastLevel   % we have some ballast in this slice
                for k=1:length(interiorRegions)
                    partitioned = chopPolygon(getCCWVertices(interiorRegions(k)), ballastLevel);
                    ballast = polyshape(partitioned{1})
                    if ballast.area > 0
                        ballastPolys(end+1) = ballast;
                    end
                    hull = polyshape(partitioned{2});
                    if hull.area > 0
                        hullPolys(end+1) = hull;
                    end
                end
            else
                if boatInterior.area > 0
                    hullPolys(end+1) = boatInterior;
                end
            end
            if ~isempty(ballastPolys)
                ballastPoly = union(ballastPolys);
            else
                ballastPoly = polyshape();
            end
            regions = ballastPoly.regions;
            for k=1:length(regions)
                allRegions{end+1} = getCCWVertices(regions(k));
                allComponentDensityRatios(end+1) = ballastDensityRatio;
                allMaterials{end+1} = 'Gazebo/Red';
                allZs(end+1) = Z(slice);
            end
            if ~isempty(hullPolys)
                hullPoly = union(hullPolys);
            else
                hullPoly = polyshape();
            end
            regions = hullPoly.regions;
            for k=1:length(regions)
                allRegions{end+1} = getCCWVertices(regions(k));
                allComponentDensityRatios(end+1) = densityRatio;
                allMaterials{end+1} = 'Gazebo/Gray';
                allZs(end+1) = Z(slice);
            end
        end
        polyline(doc, modelElem, 'customBoat', allComponentDensityRatios, Z(2)-Z(1), allRegions, allMaterials, true, allZs, meshBoatSTLPath);
        if shouldPause
            msg = rosmessage(pauseSvc);
            call(pauseSvc, msg);
        end
        if ~useWithoutGazebo
            % setting a yaw of pi/2 causes the boat to orirent in an upright stance
            spawnModel('customboat',xmlwrite(sdfElem), 0, 0, 1, pi/2, 0, 0, true);
        end
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
            case 'x'
                scaleFactor = exportedBoatLength / boatLength * 1000; % most 3d printing services assume a unit of mm (hence the factor of 1000)
                % shift the Zs, so they are positioned at the center of
                % each slice (TODO: this might be chopping off a slice from
                % each end of the boat)
                shiftedZs = Z + (Z(2)-Z(1))/2;
                exportBoatForPrinting(scaleFactor, allPoints, shiftedZs, loftCurve, maxY, ballastLevel, averageInfill, f);
        end
    end

    % allow usage without being connected to Gazebo simulataor
    useWithoutGazebo = false;

    % track whether the boat has been spawned in Gazebo
    spawned = false;
    % y-limits of hull cross section visualization
    minY = -0.5;
    maxY = 1.0;
    yBuffer = 0.1; % used to make it easier to complete the deck
    % x-limits of hull cross section visualization
    minX = -2;
    maxX = 2;

    % the hull below this level is ballast
    ballastLevel = minY;
    % density ratio of ballast is 1.24 (solid PLA)
    ballastDensityRatio = 1.24;
    % the ratio of density of non-ballast material to water (for
    % compatibility with 3d print services, simulate 20% infill
    % solid PLA (based on Prusa Slicer, the density of 20% infill runs a
    % little higher than 20% of the density of solid PLA)s
    densityRatio = ballastDensityRatio*0.2113;
    % average infill (useful for ordering 3d prints)
    averageInfill = 0;
    % the boat length (this is the extrusion dimension)
    boatLength = 5;
    % exported boat length in meters (if creating a 3d print)
    exportedBoatLength = 0.2032;% m (or 8 inches)
    % use this if you want to model the shell added by a 3d print
    % 1.2 is roughly the thickness in millimeters when printing with 3
    % walls and 6 layers top and bottom
    perimeterThickness = 1.2/1000*boatLength/exportedBoatLength;
    % this is a global variable that is used for visualizing the boat cross
    % sections
    crossSectionFigure = [];
    createMeshBoat = true;
    
    longitudinalShapeParameter = 4;
    Z = linspace(-boatLength/2, boatLength/2, 200);
    Z = Z - (Z(2)-Z(1))/2;  % maintain a Z center of mass of 0
    loftCurve = abs(2.*(Z + (Z(2)-Z(1))/2)./boatLength).^longitudinalShapeParameter;
    if ~useWithoutGazebo
        getPhysicsClient = rossvcclient('/gazebo/get_physics_properties');
        m = rosmessage(getPhysicsClient);
        physicsProperties = call(getPhysicsClient, m);

        setPhysicsClient = rossvcclient('/gazebo/set_physics_properties');
        m = rosmessage(setPhysicsClient);
        if createMeshBoat
            m.TimeStep = 0.005;
            m.MaxUpdateRate = 200;
        else
            m.TimeStep = 0.01;
            m.MaxUpdateRate = 100;
        end
        m.Gravity = physicsProperties.Gravity;
        m.OdeConfig = physicsProperties.OdeConfig;
        call(setPhysicsClient, m);
    end

	f = figure('CloseRequestFcn',@closeRequest);
    subplot(7,1,4:6);
    if ~useWithoutGazebo
        % used to pause the simulation when a new boat is spawned
        pauseSvc = rossvcclient('/gazebo/pause_physics');
        % used for putting the boat at various angles
        updateModelSvc = rossvcclient('gazebo/set_model_state');
    end
    % populate the hull with these two points
    startPoint = [0 0];
    endPoint = [0 1];
    allPoints = [startPoint; endPoint];

    userPoints = [];
    % This was added to make a boat for John (useful for testing purposes)
    populateWithPolyBoatTmp = false;
    if populateWithPolyBoatTmp
        polyboatX = linspace(1/20,1,20);
        polyboatY = abs(2.*polyboatX/2).^2;
        userPoints = [polyboatX' polyboatY'];
    end
    updateAllPoints();
    stlFileName = '';
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
    ylim([minY maxY+yBuffer]);
    % put a translucent red patch over the left half of the figure to
    % indicate to the user not to click there
    redPatch = patch([minX 0 0 minX]', [minY minY maxY maxY]', [1 0.5 0.5], 'PickableParts', 'none', 'linestyle','none');
    alpha(redPatch, 0.5);

    % put a translucent gray patch over the region above the deck
    grayPatch = patch([minX maxX maxX minX]', [maxY maxY maxY+yBuffer maxY+yBuffer]', [0.9 0.9 0.9], 'PickableParts', 'none', 'linestyle', 'none');
    alpha(grayPatch, 0.5);

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
    thetas = linspace(0,180,180+1);
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
