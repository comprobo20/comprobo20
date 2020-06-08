function customBoat()
    function myCloseRequest(src,callbackdata)
        % Close request function 
        % to display a question dialog box
        % get rid of subscriptions to avoid race conditions
        clear pauseSvc;
        clear updateModelSvc;
        delete(gcf)
    end
    function setDensityRatio(hObject, eventdata, handles)
        % callback function for the angular velocity slider
        densityRatio = get(hObject, 'Value');
        % reflect the notion that the boat has not been respawned with the
        % current configuration
        spawned = false;
        plotAVSCurve(allPoints);
    end
    function setballastLevel(hObject, eventdata, handles)
        % callback function for the angular velocity slider
        ballastLevel = get(hObject, 'Value');
        % reflect the notion that the boat has not been respawned with the
        % current configuration
        spawned = false;
        set(b,'XData',[minX maxX],'YData',[ballastLevel ballastLevel]);
        plotAVSCurve(allPoints);
    end
    function allPointsAugmented = augmentPoints()
        N = 500;
        allPointsAugmented = [];
        for i = 1:size(allPoints,1)-1
            xAug = allPoints(i,1)*linspace(1,0,N) + (1-linspace(1,0,N))*allPoints(i+1,1);
            yAug = allPoints(i,2)*linspace(1,0,N) + (1-linspace(1,0,N))*allPoints(i+1,2);
            allPointsAugmented = [allPointsAugmented; xAug' yAug'];
        end
    end
    function CoM = plotAVSCurve(allPoints)
        startAVSCurve = tic;
        if size(allPoints,1) <= 3
            return
        end
        % augment points as the waterline function relies on having finer
        % granularity on the polygon sides
        allPointsAugmented = augmentPoints();
        torques = zeros(size(thetas));
        waterlines = zeros(size(thetas));
        poly = polyshape(allPoints);
        if poly.NumRegions ~= 1 | poly.NumHoles ~= 0
            disp("Can't spawn model as it has holes or intersections");
            disp("Use the 'r' key to reset");
            return
        end
        totalWeightedArea = 0;
        xcom = 0;
        ycom = 0;
        % split the polygon at the ballast level
        polySplit = addboundary(poly,[minX,maxX,maxX,minX,minX],[ballastLevel,ballastLevel,ballastLevel+10^-5,ballastLevel+10^-5,ballastLevel]);
        r = polySplit.regions;
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
                % check if the region is in the ballast zone or not
                % (10^-5 is a fudge factor)
                if max(vertices(:,end))<=ballastLevel+10^-5
                    % 1.24 is the density of solid PLA
                    componentDensityRatio = 1.24;
                    [~,xcomRegion,ycomRegion] = cumulativeArea(vertices,area-10^-4);
                else
                    componentDensityRatio = densityRatio;
                    [~,xcomRegion,ycomRegion] = cumulativeArea(vertices,area-10^-4);
                end
                totalWeightedArea = totalWeightedArea + area*componentDensityRatio;
                xcom = xcom + componentDensityRatio*xcomRegion*area;
                ycom = ycom + componentDensityRatio*ycomRegion*area;
            end
        end
        CoM = [xcom ycom]/ totalWeightedArea;
        for i = 1:length(thetas)
            [torques(i),waterlines(i)] = getWaterLineGreensTheorem(thetas(i),boatLength,CoM,totalWeightedArea,allPoints,densityRatio,ballastLevel);
        end
        toc(startAVSCurve);

        set(CoMPlot,'XData',CoM(1),'YData',CoM(2));
        set(avsPlot,'YData',torques);
    end
    function avsClick(axesObj, eventDat)
        if ~spawned
            spawnBoat();
            pause(2);
        end
        x = eventDat.IntersectionPoint(1);
        y = eventDat.IntersectionPoint(2);
        allPointsAugmented = augmentPoints();
        [~,waterline,~] = getWaterLineGreensTheorem(x,boatLength,CoM,totalWeightedArea,allPoints,densityRatio,ballastLevel);
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
            disp('Only clicks in the right side of the figure are used');
            disp('The left side will be the mirror of the right side');
            return;
        end
        figure(gcf);
        userPoints = [userPoints; [x y]];
        % reflect the notion that the boat has not been respawned with the
        % current configuration
        spawned = false;
        updateAllPoints();
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
            flippedUserPoints = [-userPoints(:,1) userPoints(:,2)];
        else
            flippedUserPoints = userPoints;
        end
        allPoints = [startPoint; userPoints; endPoint; flippedUserPoints(end:-1:1,:); startPoint];
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
        % split the polygon at the ballast level
        polySplit = addboundary(poly,[minX,maxX,maxX,minX,minX],[ballastLevel,ballastLevel,ballastLevel+10^-5,ballastLevel+10^-5,ballastLevel]);
        r = polySplit.regions;
        % spawn the model
        doc = com.mathworks.xml.XMLUtils.createDocument('sdf');
        sdfElem = doc.getDocumentElement();
        sdfElem.setAttribute('version', '1.5');
        modelElem = doc.createElement('model');
        sdfElem.appendChild(modelElem);
        modelElem.setAttribute('name', 'customboat');
        baseLink = [];
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
                end
                % check if the region is in the ballast zone or not
                % (10^-5 is a fudge factor)
                if max(vertices(:,end))<=ballastLevel+10^-5
                    % 1.24 is the density of solid PLA
                    componentDensityRatio = 1.24;
                    material = 'Gazebo/Red';
                else
                    componentDensityRatio = densityRatio;
                    material = 'Gazebo/Gray';
                end
                polyline(doc, modelElem, ['polycomponent',num2str(i)], componentDensityRatio, boatLength, vertices, material, true);
                if isempty(baseLink)
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
                spawned = false;
                updateAllPoints();
                set(p,'XData',allPoints(:,1),'YData',allPoints(:,2));
                set(s,'XData',allPoints(:,1),'YData',allPoints(:,2));
         end
    end
    spawned = false;
    minY = -0.5;
    maxY = 1.25;
    minX = -2;
    maxX = 2;
    densityRatio = 0.25;
    ballastLevel = minY;
    boatLength = 2;
	f = figure('CloseRequestFcn',@myCloseRequest);
    subplot(7,1,4:6);
    pauseSvc = rossvcclient('/gazebo/pause_physics');
    updateModelSvc = rossvcclient('gazebo/set_model_state');
    startPoint = [0 0];
    endPoint = [0 1];
    userPoints = [];
    allPoints = [startPoint; endPoint];

    weightedArea = 0;
    s = scatter(allPoints(:,1), allPoints(:,2),'MarkerFaceColor', uint8([128 128 128]), 'MarkerEdgeColor', uint8([64 64 64]));
    hold on;
    p = plot(allPoints(:,1), allPoints(:,2),'k');
    b = plot([minX maxX],[ballastLevel ballastLevel],'k');
    CoM = [(startPoint(1)+endPoint(1))/2, (startPoint(2)+endPoint(2))/2];
    totalWeightedArea = 0;
    CoMPlot = plot(CoM(1),CoM(2),'b.');
    set(CoMPlot,'MarkerSize',25);

    xlabel('x (m)');
    ylabel('y (m)');
    xlim([minX maxX]);
    ylim([minY maxY]);
    grayPatch = patch([minX 0 0 minX]', [minY minY maxY maxY]', [1 0.5 0.5]);
    alpha(grayPatch, 0.5);
    title('s for "spawn", r for "reset".  Crossing lines are not permitted.');
    daspect([1 1 1]);
        sld = uicontrol('Style', 'slider',...
        'Min',0.01,'Max',0.99,'Value',densityRatio,...
        'Position', [20 20 120 20],...
        'Callback', @setDensityRatio);
    % Add a text uicontrol to label the slider.
    txt = uicontrol('Style','text',...
        'Position',[20 45 120 20],...
        'String','Density Ratio');
    daspect([1 1 1]);
    sld = uicontrol('Style', 'slider',...
        'Min',minY,'Max',maxY,'Value',ballastLevel,...
        'Position', [160 20 120 20],...
        'Callback', @setballastLevel);
    % Add a text uicontrol to label the slider.
    txt = uicontrol('Style','text',...
        'Position',[160 45 120 20],...
        'String','ballast Level');
    set(f,'WindowKeyPressFcn', @keyPressedFunction);
    set(gca,'ButtonDownFcn',@mouseDownFunction,'HitTest','on');
    thetas = linspace(0,180,180+1);
    subplot(7,1,1:2);
    avsPlot = plot(thetas,zeros(size(thetas)));
    set(gca,'ButtonDownFcn',@avsClick,'HitTest','on');

    hold on; % draw a line to make the AVS easier to see
    plot(thetas,zeros(size(thetas)),'r');
    xlabel('heel angle (degrees)');
    ylabel('torque');
end
