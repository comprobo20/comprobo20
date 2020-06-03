function polyboat()

svc = rossvcclient('/gazebo/pause_physics');
msg = rosmessage(svc);
call(svc, msg);
isBoat = true;
materials = {'Gazebo/Purple', 'Gazebo/Blue', 'Gazebo/Red', 'Gazebo/Green', 'Gazebo/Orange'};
p = linspace(0.5,4.5,5);
for i = 1:length(p)
    doc = com.mathworks.xml.XMLUtils.createDocument('sdf');
    sdfElem = doc.getDocumentElement();
    sdfElem.setAttribute('version', '1.5');
    modelElem = doc.createElement('model');
    sdfElem.appendChild(modelElem);
    modelElem.setAttribute('name', ['poly',num2str(i)]);

    xs = linspace(-1, 1, 51);
    ys = abs(xs).^p(i);
    points = [xs' ys'; xs(1) ys(1)];
    % use a 0.25 density ratio
    polyline(doc, modelElem, 'poly', 0.25, 2, points, materials{i}, isBoat);
    % setting a yaw of pi/2 causes the boat to shift to an upright stance
    spawnModel(['poly',num2str(i)],xmlwrite(sdfElem), 3*(i-1)-6, 0, 1, pi/2, 0, 0, true);
end

end