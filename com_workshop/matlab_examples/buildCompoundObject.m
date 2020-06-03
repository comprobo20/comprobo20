% TODO: 
% Do a game where you have a bunch of shapes and a bunch of posts.  You can
% pause physics, shift the positions until they get to where you want, and
% then unpause and see what happens!

function buildCompoundObject(usePolyline)
if nargin < 1
    usePolyline = false;
end
doc = com.mathworks.xml.XMLUtils.createDocument('robot');
robotElem = doc.getDocumentElement();
robotElem.setAttribute('name','base');

redBox(doc, 'balance', 100, 0.1, 0.1, 1);
spawnModel('balance',xmlwrite(robotElem),0,0,0);

doc = com.mathworks.xml.XMLUtils.createDocument('robot');

sheetThickness = 0.1;
robotElem = doc.getDocumentElement();
robotElem.setAttribute('name','compound');

redBox(doc, 'sheet', 1, 1, 1, sheetThickness);

% first column is x-position relative to sheet
% second column is y-position relative to sheet
% third column is disk mass
% fourth column is disk radius
% fifth column is disk thickness
weights = [0.25, 0.25, 0.5, 0.1, 0.1;...
           -0.25, 0.25, 0.5, 0.1, 0.1;...
           0.25, -0.25, 1.0, 0.1, 0.2;...
           -0.25, -0.25, 0.5, 0.1, 0.1];

for i = 1 : size(weights,1)
    blueCylinder(doc, ['weight_',num2str(i)], weights(i,3), weights(i,4), weights(i,5));
    connectWithFixedJoint(doc, ['weight_',num2str(i),'_joint'], 'sheet', ['weight_',num2str(i)], weights(i,1), weights(i,2), sheetThickness);
end

if ~usePolyline
    spawnModel('sheet',xmlwrite(robotElem),0,0,1);
end

doc = com.mathworks.xml.XMLUtils.createDocument('sdf');
sdfElem = doc.getDocumentElement();
sdfElem.setAttribute('version','1.5');
modelElem = doc.createElement('model');
sdfElem.appendChild(modelElem);
modelElem.setAttribute('name','testpoly');

xs = linspace(-1, 1, 50);
ys = abs(xs).^2;
%ys = exp(-(2*xs).^2);
points = [xs' ys'; xs(1) ys(1)];
polyline(doc, modelElem, 'square', 1, 0.05, points);

if usePolyline
    spawnModel('testpoly',xmlwrite(sdfElem),0,-0.6,1,0,0,0,true);
end 
end
