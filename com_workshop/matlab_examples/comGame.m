function comGame()

svc = rossvcclient('/gazebo/pause_physics');
msg = rosmessage(svc);
call(svc, msg);
materials = {'Gazebo/RedTransparentOverlay','Gazebo/DarkOrangeTransparentOverlay','Gazebo/GreenTransparentOverlay','Gazebo/OrangeTransparentOverlay','Gazebo/BlueTransparentOverlay'};
for i = 1 : 5
    doc = com.mathworks.xml.XMLUtils.createDocument('robot');
    robotElem = doc.getDocumentElement();
    robotElem.setAttribute('name',['base',num2str(i)]);

    redBox(doc, 'balance', 100, 0.1, 0.1, 1);
    spawnModel(['balance',num2str(i)],xmlwrite(robotElem),2.5*(i-1)-5,0,0);
      
    doc = com.mathworks.xml.XMLUtils.createDocument('sdf');
    sdfElem = doc.getDocumentElement();
    sdfElem.setAttribute('version','1.5');
    modelElem = doc.createElement('model');
    sdfElem.appendChild(modelElem);
    modelElem.setAttribute('name',['poly',num2str(i)]);

    xs = linspace(-1, 1, 50);
    ys = abs(xs).^i;
    points = [xs' ys'; xs(1) ys(1)];
    polyline(doc, modelElem, 'poly', 1, 0.05, points, materials{i});

    spawnModel(['poly',num2str(i)],xmlwrite(sdfElem),2.5*(i-1)-5,0,1,0,0,0,true);
end

end
