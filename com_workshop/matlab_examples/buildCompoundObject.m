function buildTable()
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

spawnModel('sheet',xmlwrite(robotElem),0,0,1);

doc = com.mathworks.xml.XMLUtils.createDocument('sdf');
sdfElem = doc.getDocumentElement();
sdfElem.setAttribute('version','1.5');
modelElem = doc.createElement('model');
sdfElem.appendChild(modelElem);
modelElem.setAttribute('name','testpoly');

xs = linspace(-1, 1, 50);
ys = abs(xs);
points = [xs' ys'; xs(1) ys(1)];
polyline(doc, modelElem, 'square', 1, 0.5, points);

spawnModel('testpoly',xmlwrite(sdfElem),0,0,1,true);

    function redBox(doc, name, mass, l, w, h)
        box = doc.createElement('link');
        doc.getDocumentElement().appendChild(box);
        box.setAttribute('name', name);
        visual = doc.createElement('visual');
        box.appendChild(visual);
        visualOrigin = doc.createElement('origin');
        visual.appendChild(visualOrigin);
        visualOrigin.setAttribute('xyz', ['0 0 ',num2str(h/2)]);
        visualGeometry = doc.createElement('geometry');
        visual.appendChild(visualGeometry);
        geometryBox = doc.createElement('box');
        visualGeometry.appendChild(geometryBox);
        geometryBox.setAttribute('size', [num2str(l),' ',num2str(w),' ',num2str(h)]);

        collision = doc.createElement('collision');
        box.appendChild(collision);
        collisionOrigin = doc.createElement('origin');
        collision.appendChild(collisionOrigin);
        collisionOrigin.setAttribute('xyz', ['0 0 ',num2str(h/2)]);
        collisionGeometry = doc.createElement('geometry');
        collision.appendChild(collisionGeometry);
        geometryBox = doc.createElement('box');
        collisionGeometry.appendChild(geometryBox);
        geometryBox.setAttribute('size', [num2str(l),' ',num2str(w),' ',num2str(h)]);

        inertial = doc.createElement('inertial');
        box.appendChild(inertial);
        inertialOrigin = doc.createElement('origin');
        inertial.appendChild(inertialOrigin);
        inertialOrigin.setAttribute('xyz', ['0 0 ',num2str(h/2)]);
        inertialMass = doc.createElement('mass');
        inertial.appendChild(inertialMass);
        inertialMass.setAttribute('value', num2str(mass));
        % https://en.wikipedia.org/wiki/List_of_moments_of_inertia#List_of_3D_inertia_tensors  Note: we are using length, width, height where they use width, height, depth
        inertia = doc.createElement('inertia');
        inertial.appendChild(inertia);
        inertia.setAttribute('ixx',num2str(1/12*mass*(h^2 + w^2)));
        inertia.setAttribute('ixy',num2str(0));
        inertia.setAttribute('ixz',num2str(0));
        inertia.setAttribute('iyy',num2str(1/12*mass*(h^2 + l^2)));
        inertia.setAttribute('iyz',num2str(0));
        inertia.setAttribute('izz',num2str(1/12*mass*(l^2 + w^2)));

        gazebo = doc.createElement('gazebo');
        gazebo.setAttribute('reference', name);
        doc.getDocumentElement().appendChild(gazebo);
        material = doc.createElement('material');
        gazebo.appendChild(material);
        material.setTextContent('Gazebo/Red');
    end

    function blueCylinder(doc, name, mass, r, l)
        cylinder = doc.createElement('link');
        doc.getDocumentElement().appendChild(cylinder);
        cylinder.setAttribute('name', name);
        visual = doc.createElement('visual');
        cylinder.appendChild(visual);
        visualOrigin = doc.createElement('origin');
        visual.appendChild(visualOrigin);
        visualOrigin.setAttribute('xyz', ['0 0 ',num2str(l/2)]);
        visualGeometry = doc.createElement('geometry');
        visual.appendChild(visualGeometry);
        geometryCylinder = doc.createElement('cylinder');
        visualGeometry.appendChild(geometryCylinder);
        geometryCylinder.setAttribute('radius', num2str(r));
        geometryCylinder.setAttribute('length', num2str(l));

        collision = doc.createElement('collision');
        cylinder.appendChild(collision);
        collisionOrigin = doc.createElement('origin');
        collision.appendChild(collisionOrigin);
        collisionOrigin.setAttribute('xyz', ['0 0 ',num2str(l/2)]);
        collisionGeometry = doc.createElement('geometry');
        collision.appendChild(collisionGeometry);
        geometryCylinder = doc.createElement('cylinder');
        collisionGeometry.appendChild(geometryCylinder);
        geometryCylinder.setAttribute('radius', num2str(r));
        geometryCylinder.setAttribute('length', num2str(l));

        inertial = doc.createElement('inertial');
        cylinder.appendChild(inertial);
        inertialOrigin = doc.createElement('origin');
        inertial.appendChild(inertialOrigin);
        inertialOrigin.setAttribute('xyz', ['0 0 ',num2str(l/2)]);
        inertialMass = doc.createElement('mass');
        inertial.appendChild(inertialMass);
        inertialMass.setAttribute('value', num2str(mass));
        % http://scienceworld.wolfram.com/physics/MomentofInertiaCylinder.html
        inertia = doc.createElement('inertia');
        inertial.appendChild(inertia);
        inertia.setAttribute('ixx',num2str(1/12*mass*l^2 + 1/4*mass*r^2));
        inertia.setAttribute('ixy',num2str(0));
        inertia.setAttribute('ixz',num2str(0));
        inertia.setAttribute('iyy',num2str(1/12*mass*l^2 + 1/4*mass*r^2));
        inertia.setAttribute('iyz',num2str(0));
        inertia.setAttribute('izz',num2str(1/2*mass*r^2));

        gazebo = doc.createElement('gazebo');
        gazebo.setAttribute('reference', name);
        doc.getDocumentElement().appendChild(gazebo);
        material = doc.createElement('material');
        gazebo.appendChild(material);
        material.setTextContent('Gazebo/Blue');
    end

    function polyline(doc, modelElem, name, mass, height, points)
        % TODO: allow this to connect to simple shapes via joints
        polyline = doc.createElement('link');
        modelElem.appendChild(polyline);
        polyline.setAttribute('name', name);
        visual = doc.createElement('visual');
        visual.setAttribute('name', 'visual');
        polyline.appendChild(visual);
        visualGeometry = doc.createElement('geometry');
        visual.appendChild(visualGeometry);
        geometryPolyline = doc.createElement('polyline');
        visualGeometry.appendChild(geometryPolyline);
        for i = 1:size(points,1)
            polylinePoint = doc.createElement('point');
            geometryPolyline.appendChild(polylinePoint);
            polylinePoint.setTextContent([num2str(points(i,1)),' ',num2str(points(i,2))]);
        end
        polylineHeight = doc.createElement('height');
        geometryPolyline.appendChild(polylineHeight);
        polylineHeight.setTextContent(num2str(height));

        collision = doc.createElement('collision');
        collision.setAttribute('name', 'collision');
        polyline.appendChild(collision);
        collisionGeometry = doc.createElement('geometry');
        collision.appendChild(collisionGeometry);
        geometryPolyline = doc.createElement('polyline');
        collisionGeometry.appendChild(geometryPolyline);
        for i = 1:size(points,1)
            polylinePoint = doc.createElement('point');
            geometryPolyline.appendChild(polylinePoint);
            polylinePoint.setTextContent([num2str(points(i,1)),' ',num2str(points(i,2))]);
        end
        polylineHeight = doc.createElement('height');
        geometryPolyline.appendChild(polylineHeight);
        polylineHeight.setTextContent(num2str(height));

        surface = doc.createElement('surface');
        collision.appendChild(surface);
        contact = doc.createElement('contact');
        surface.appendChild(contact);
        collideBitmask = doc.createElement('collide_bitmask');
        contact.appendChild(collideBitmask);
        collideBitmask.setTextContent('0xffff');
        friction = doc.createElement('friction');
        surface.appendChild(friction);
        odeNode = doc.createElement('ode');
        friction.appendChild(odeNode);
        mu = doc.createElement('mu');
        odeNode.appendChild(mu);
        mu.setTextContent('100');
        mu2 = doc.createElement('mu2');
        odeNode.appendChild(mu2);
        mu2.setTextContent('50');
        
        inertial = doc.createElement('inertial');
        polyline.appendChild(inertial);
        inertialPose = doc.createElement('pose');
        inertial.appendChild(inertialPose);
        [xcom,ycom,area] = polyCOM(points);
        com = [xcom ycom height/2];  % need to determine this x and y
        inertialPose.setTextContent([num2str(com(1)),' ',num2str(com(2)),' ',num2str(com(3)),' 0 0 0']);
        inertialMass = doc.createElement('mass');
        inertial.appendChild(inertialMass);
        inertialMass.setTextContent(num2str(mass));
        inertia = doc.createElement('inertia');
        inertial.appendChild(inertia);
        % need to figure out these parameters (for now just set them to 1
        % along ixx, iyy, izz
        ixx = doc.createElement('ixx');
        inertia.appendChild(ixx);
        ixx.setTextContent('1');

        ixy = doc.createElement('ixy');
        inertia.appendChild(ixy);
        ixy.setTextContent('0');
        
        ixz = doc.createElement('ixz');
        inertia.appendChild(ixz);
        ixz.setTextContent('0');
        
        iyy = doc.createElement('iyy');
        inertia.appendChild(iyy);
        iyy.setTextContent('1');
        
        iyz = doc.createElement('iyz');
        inertia.appendChild(iyz);
        iyz.setTextContent('1');
        
        izz = doc.createElement('izz');
        inertia.appendChild(izz);
        izz.setTextContent('1');
        % TODO: add material
    end

    function connectWithFixedJoint(doc, jointName, parentLinkName, childLinkName, x, y, z)
        joint = doc.createElement('joint');
        doc.getDocumentElement().appendChild(joint);
        joint.setAttribute('name',jointName);
        joint.setAttribute('type','fixed');
    
        parentLink = doc.createElement('parent');
        parentLink.setAttribute('link', parentLinkName);
        joint.appendChild(parentLink);
        childLink = doc.createElement('child');
        childLink.setAttribute('link', childLinkName);
        joint.appendChild(childLink);
    
        jointOrigin = doc.createElement('origin');
        jointOrigin.setAttribute('xyz',[num2str(x),' ',num2str(y),' ',num2str(z)]);
        joint.appendChild(jointOrigin);
    end

    function [xcom, ycom, area] = polyCOM(points)
        area = 0;
        for i = 1:size(points,1)-1
            area = area + 0.5*(points(i,1)*points(i+1,2) - points(i+1,1)*points(i,2));
        end
        xcom = 0;
        ycom = 0;
        for i = 1:size(points,1)-1
            xcom = xcom + 1/(6*area)*(points(i,1)+points(i+1,1))*(points(i,1)*points(i+1,2) - points(i+1,1)*points(i,2));
            ycom = ycom + 1/(6*area)*(points(i,2)+points(i+1,2))*(points(i,1)*points(i+1,2) - points(i+1,1)*points(i,2));
        end
    end
end