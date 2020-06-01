function buildTable()
doc = com.mathworks.xml.XMLUtils.createDocument('robot');
robotElem = doc.getDocumentElement;
robotElem.setAttribute('name','base');
robotElem.setAttribute('xmlns:xacro','http://ros.org/wiki/xacro');
macros = xmlread('../urdf/comShapes.xacro');
% import macros until we have the xacro in the container
l = macros.getDocumentElement().getElementsByTagName('xacro:macro');
for i = 1:l.getLength()
    % use 0-based indexing
    macroNode = doc.importNode(l.item(i-1), true);
    robotElem.appendChild(macroNode);
end
box = doc.createElement('xacro:red_box');
box.setAttribute('name','balance');
box.setAttribute('mass','100');
box.setAttribute('length','0.1');
box.setAttribute('width','0.1');
box.setAttribute('height','1');

robotElem.appendChild(box);
xmlwrite('/tmp/test.xml',robotElem);
spawnXacro('balance','/tmp/test.xml',0,0,0);

sheetThickness = 0.1;
doc = com.mathworks.xml.XMLUtils.createDocument('robot');
robotElem = doc.getDocumentElement;
robotElem.setAttribute('name','compound');
robotElem.setAttribute('xmlns:xacro','http://ros.org/wiki/xacro');
macros = xmlread('../urdf/comShapes.xacro');
% import macros until we have the xacro in the container
l = macros.getDocumentElement().getElementsByTagName('xacro:macro');
for i = 1:l.getLength()
    % use 0-based indexing
    macroNode = doc.importNode(l.item(i-1), true);
    robotElem.appendChild(macroNode);
end
box = doc.createElement('xacro:red_box');
box.setAttribute('name','sheet');
box.setAttribute('mass','1');
box.setAttribute('length','1');
box.setAttribute('width','1');
box.setAttribute('height',num2str(sheetThickness));
robotElem.appendChild(box);

% first column is x position relative to sheet
% second column is y position relative to sheet
% third column is disk mass
% fourth column is disk radius
% fifth column is disk thickness
weights = [0.25, 0.25, 0.5, 0.1, 0.1;...
           -0.25, 0.25, 0.5, 0.1, 0.1;...
           0.25, -0.25, 0.5, 0.1, 0.1;...
           -0.25, -0.25, 0.5, 0.1, 0.1];
for i = 1 : size(weights,1)
    weight = doc.createElement('xacro:blue_cylinder');
    weight.setAttribute('name',['weight_',num2str(i)]);
    weight.setAttribute('mass',num2str(weights(i,3)));
    weight.setAttribute('radius',num2str(weights(i,4)));
    weight.setAttribute('length',num2str(weights(i,5)));
    robotElem.appendChild(weight);
    joint = doc.createElement('joint');
    joint.setAttribute('name',['weight_',num2str(i),'_joint']);
    joint.setAttribute('type','fixed');
    
    parentLink = doc.createElement('parent');
    parentLink.setAttribute('link','sheet');
    joint.appendChild(parentLink);
    childLink = doc.createElement('child');
    childLink.setAttribute('link',['weight_',num2str(i)]);
    joint.appendChild(childLink);
    
    jointOrigin = doc.createElement('origin');
    jointOrigin.setAttribute('xyz',[num2str(weights(i,1)), ' ', num2str(weights(i,2)), ' ', num2str(sheetThickness)]);
    joint.appendChild(jointOrigin);
    robotElem.appendChild(joint);
end

xmlwrite('/tmp/test.xml',robotElem);
spawnXacro('sheet','/tmp/test.xml',0,0,1);
end