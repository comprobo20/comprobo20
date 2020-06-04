function polyboat()

svc = rossvcclient('/gazebo/pause_physics');
msg = rosmessage(svc);
call(svc, msg);
isBoat = true;
materials = {'Gazebo/Purple', 'Gazebo/Blue', 'Gazebo/Red', 'Gazebo/Green', 'Gazebo/Orange'};
n = linspace(1,5,5);
W = 1;
D = 0.5;
densityRatio = 0.25;
for i = 1:length(n)
    doc = com.mathworks.xml.XMLUtils.createDocument('sdf');
    sdfElem = doc.getDocumentElement();
    sdfElem.setAttribute('version', '1.5');
    modelElem = doc.createElement('model');
    sdfElem.appendChild(modelElem);
    modelElem.setAttribute('name', ['poly',num2str(i)]);

    xs = linspace(-W/2, W/2, 51);
    ys = D.*abs(2.*xs./W).^n(i);
    points = [xs' ys'; xs(1) ys(1)];
    polyline(doc, modelElem, 'poly', densityRatio, 0.6, points, materials{i}, isBoat);
    % setting a yaw of pi/2 causes the boat to orirent in an upright stance
    spawnModel(['poly',num2str(i)],xmlwrite(sdfElem), 3*(i-1)-6, 0, 1, pi/2, 0, 0, true);
end

end