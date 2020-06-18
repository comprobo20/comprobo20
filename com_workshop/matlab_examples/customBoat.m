function customBoat()
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

    function [torque,d,CoM] = getWaterLineHelper(theta, L, CoM, densityRatioWeightedArea, allPoints)
        mass = densityRatioWeightedArea*L*1000;%Kg

        % define the direction of gravity
        down = [0, 0, -1];

        % rotate the boat about the origin to match the Gazebo simulator
        yy = cosd(theta)*allPoints(:,1) + sind(theta)*allPoints(:,2);
        zz = - sind(theta)*allPoints(:,1) + cosd(theta)*allPoints(:,2);
        area_displaced_at_eq = mass/L/1000;
        [d,ycob,zcob] = cumulativeArea([yy zz],area_displaced_at_eq);
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
        [regions isBallast areas] = splitHullAtBallastLevel(poly);
        totalWeightedArea = 0;
        CoM = [0 0];
        for i = 1 : length(isBallast)
            [~,xcomRegion,ycomRegion] = cumulativeArea(regions{i},Inf); % Inf means use the whole polygon
            if isBallast(i)
                regionDensity = ballastDensityRatio;
            else
                regionDensity = densityRatio;
            end
            CoM = CoM + [xcomRegion ycomRegion]*areas(i)*regionDensity;
            totalWeightedArea = totalWeightedArea + areas(i)*regionDensity;
        end
        CoM = CoM / totalWeightedArea;
        for i = 1:length(thetas)
            [torques(i),waterlines(i)] = getWaterLineHelper(thetas(i),boatLength,CoM,totalWeightedArea,allPoints);
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
        [~,waterline,~] = getWaterLineHelper(x,boatLength,CoM,totalWeightedArea,allPoints);
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
        [regions isBallast ~] = splitHullAtBallastLevel(poly);
        % spawn the model
        doc = com.mathworks.xml.XMLUtils.createDocument('sdf');
        sdfElem = doc.getDocumentElement();
        sdfElem.setAttribute('version', '1.5');
        modelElem = doc.createElement('model');
        sdfElem.appendChild(modelElem);
        modelElem.setAttribute('name', 'customboat');
        for i = 1 : length(regions)
            if isBallast(i)
                componentDensityRatio = ballastDensityRatio;
                material = 'Gazebo/Red';
            else
                componentDensityRatio = densityRatio;
                material = 'Gazebo/Gray';
            end
            % TODO: could revamp this to use what we do for 3d boats, but
            % so far it doesn't seem necessary and might introduce
            % additional bugs
            polyline(doc, modelElem, ['polycomponent',num2str(i)], componentDensityRatio, boatLength, regions{i}, material, true);
            if i == 1
                % use this as the parent for all fixed joints
                baseLink = ['polycomponent',num2str(i)];
            else
                % add a joint to fuse the two regions together
                joint = doc.createElement('joint');
                modelElem.appendChild(joint);
                joint.setAttribute('name',['polycomponent',num2str(i),'joint']);
                joint.setAttribute('type','fixed');

                parentLink = doc.createElement('parent');
                parentLink.setTextContent(baseLink);
                joint.appendChild(parentLink);
                childLink = doc.createElement('child');
                childLink.setTextContent(['polycomponent',num2str(i)]);
                joint.appendChild(childLink);
            end
        end
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
    maxY = 1.25;
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
    totalWeightedArea = 0;
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
