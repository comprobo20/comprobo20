function customBoat()
    function myCloseRequest(src,callbackdata)
        % Close request function 
        % to display a question dialog box
        % get rid of subscriptions to avoid race conditions
        clear pauseSvc;
        delete(gcf)
    end
    function setDensityRatio(hObject, eventdata, handles)
        % callback function for the angular velocity slider
        densityRatio = get(hObject, 'Value');
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
        flippedUserPoints = [-userPoints(:,1) userPoints(:,2)];
        allPoints = [startPoint; userPoints; endPoint; flippedUserPoints(end:-1:1,:); startPoint];
        poly = polyshape(allPoints);

        if poly.NumRegions ~= 1 | poly.NumHoles ~= 0
            disp('Illegal polygon.  No intersections or duplicate vertices are allowed');
            disp('Resetting');
            userPoints = [];
            allPoints = [startPoint; endPoint; startPoint];
        end
        set(p,'XData',allPoints(:,1),'YData',allPoints(:,2));
        set(s,'XData',allPoints(:,1),'YData',allPoints(:,2));
    end
    function keyPressedFunction(fig_obj, eventDat)
        ck = get(fig_obj, 'CurrentKey');
        switch ck
            case 's'
                % spawn the model
                doc = com.mathworks.xml.XMLUtils.createDocument('sdf');
                sdfElem = doc.getDocumentElement();
                sdfElem.setAttribute('version', '1.5');
                modelElem = doc.createElement('model');
                sdfElem.appendChild(modelElem);
                modelElem.setAttribute('name', 'customboat');
                flippedUserPoints = [-userPoints(:,1) userPoints(:,2)];
                allPoints = [startPoint; userPoints; endPoint; flippedUserPoints(end:-1:1,:); startPoint];
                polyline(doc, modelElem, 'poly', densityRatio, 2, allPoints, 'Gazebo/Red', true);
                msg = rosmessage(pauseSvc);
                call(pauseSvc, msg);
                % setting a yaw of pi/2 causes the boat to orirent in an upright stance
                spawnModel('customboat',xmlwrite(sdfElem), 0, 0, 1, pi/2, 0, 0, true);
            case 'r'
                userPoints = [];
                flippedUserPoints = [];
                allPoints = [startPoint; userPoints; endPoint; flippedUserPoints(end:-1:1,:); startPoint];
                set(p,'XData',allPoints(:,1),'YData',allPoints(:,2));
                set(s,'XData',allPoints(:,1),'YData',allPoints(:,2));
         end
    end
    densityRatio = 0.25;
	f = figure('CloseRequestFcn',@myCloseRequest);
    pauseSvc = rossvcclient('/gazebo/pause_physics');
    startPoint = [0 0];
    endPoint = [0 1];
    userPoints = [];
    allPoints = [startPoint; userPoints; endPoint];
    s = scatter(allPoints(:,1), allPoints(:,2),'MarkerFaceColor', uint8([128 128 128]), 'MarkerEdgeColor', uint8([64 64 64]));
    hold on;
    p = plot(allPoints(:,1), allPoints(:,2),'k');

    xlabel('x (m)');
    ylabel('y (m)');
    xlim([-2 2]);
    ylim([-0.5 1.25]);
    grayPatch = patch([-2 0 0 -2]', [-0.5 -0.5 1.25 1.25]', [1 0.5 0.5]);
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
    set(f,'WindowKeyPressFcn', @keyPressedFunction);
    set(gca,'ButtonDownFcn',@mouseDownFunction,'HitTest','on');
end
