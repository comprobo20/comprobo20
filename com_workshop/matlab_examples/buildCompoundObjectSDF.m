function buildCompoundObjectSDF()
% unfortunately the Gazebo GUI doesn't show the CoM of the whole model

doc = com.mathworks.xml.XMLUtils.createDocument('sdf');
sdfElem = doc.getDocumentElement();
sdfElem.setAttribute('version','1.5');
modelElem = doc.createElement('model');
sdfElem.appendChild(modelElem);
modelElem.setAttribute('name','balance');

redBox(doc, modelElem, 'balance', 0, 0, 0, 100, 0.1, 0.1, 1);
spawnModel('balance',xmlwrite(doc),0,0,0,true);

doc = com.mathworks.xml.XMLUtils.createDocument('sdf');
sdfElem = doc.getDocumentElement();
sdfElem.setAttribute('version','1.5');
modelElem = doc.createElement('model');
sdfElem.appendChild(modelElem);
modelElem.setAttribute('name','compound');

sheetThickness = 0.1;

redBox(doc, modelElem, 'sheet',  0, 0, 0, 1, 1, 1, sheetThickness);

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
    blueCylinder(doc, modelElem, ['weight_',num2str(i)], weights(i,1), weights(i,2), sheetThickness, weights(i,3), weights(i,4), weights(i,5));
    connectWithFixedJoint(doc, modelElem, ['weight_',num2str(i),'_joint'], 'sheet', ['weight_',num2str(i)], weights(i,1), weights(i,2), sheetThickness);
end

spawnModel('sheet',xmlwrite(doc),0,0,1,true);

doc = com.mathworks.xml.XMLUtils.createDocument('sdf');
sdfElem = doc.getDocumentElement();
sdfElem.setAttribute('version','1.5');
modelElem = doc.createElement('model');
sdfElem.appendChild(modelElem);
modelElem.setAttribute('name','testpoly');

xs = linspace(-1, 1, 50);
%ys = abs(xs).^8;
ys = exp(-(2*xs).^2);
points = [xs' ys'; xs(1) ys(1)];
polyline(doc, modelElem, 'square', 1, 0.5, points);

%spawnModel('testpoly',xmlwrite(doc),0,0,1,true);

    function redBox(doc, modelElem, name, x, y, z, mass, l, w, h)
        box = doc.createElement('link');
        modelElem.appendChild(box);
        box.setAttribute('name', name);
        poseElem = doc.createElement('pose');
        box.appendChild(poseElem);
        poseElem.setTextContent([num2str(x),' ',num2str(y),' ',num2str(z),' 0 0 0']);

        visual = doc.createElement('visual');
        box.appendChild(visual);
        visual.setAttribute('name', java.util.UUID.randomUUID.toString()); 

        visualPose = doc.createElement('pose');
        visual.appendChild(visualPose);
        visualPose.setTextContent(['0 0 ',num2str(h/2), ' 0 0 0']);
        visualGeometry = doc.createElement('geometry');
        visual.appendChild(visualGeometry);
        geometryBox = doc.createElement('box');
        visualGeometry.appendChild(geometryBox);
        sizeBox = doc.createElement('size');
        geometryBox.appendChild(sizeBox);
        sizeBox.setTextContent([num2str(l),' ',num2str(w),' ',num2str(h)]);

        visualMaterial = doc.createElement('material');
        visual.appendChild(visualMaterial);
        scriptMaterial = doc.createElement('script');
        visualMaterial.appendChild(scriptMaterial);
        uriMaterial = doc.createElement('uri');
        scriptMaterial.appendChild(uriMaterial);
        uriMaterial.setTextContent('file://media/materials/scripts/gazebo.material');
        nameMaterial = doc.createElement('name');
        scriptMaterial.appendChild(nameMaterial);
        nameMaterial.setTextContent('Gazebo/Red');       
 
        collision = doc.createElement('collision');
        box.appendChild(collision);
        collision.setAttribute('name', java.util.UUID.randomUUID.toString()); 
        collisionPose = doc.createElement('pose');
        collision.appendChild(collisionPose);
        collisionPose.setTextContent(['0 0 ',num2str(h/2), ' 0 0 0']);
        collisionGeometry = doc.createElement('geometry');
        collision.appendChild(collisionGeometry);
        geometryBox = doc.createElement('box');
        collisionGeometry.appendChild(geometryBox);
        sizeBox = doc.createElement('size');
        geometryBox.appendChild(sizeBox);
        sizeBox.setTextContent([num2str(l),' ',num2str(w),' ',num2str(h)]);

        inertial = doc.createElement('inertial');
        box.appendChild(inertial);
        inertialPose = doc.createElement('pose');
        inertial.appendChild(inertialPose);
        inertialPose.setTextContent(['0 0 ',num2str(h/2),' 0 0 0']);

        inertialMass = doc.createElement('mass');
        inertial.appendChild(inertialMass);
        inertialMass.setTextContent(num2str(mass));
        inertia = doc.createElement('inertia');
        inertial.appendChild(inertia);

        % https://en.wikipedia.org/wiki/List_of_moments_of_inertia#List_of_3D_inertia_tensors  Note: we are using length, width, height where they use width, height, depth
        ixxElem = doc.createElement('ixx');
        inertia.appendChild(ixxElem);
        ixxElem.setTextContent(num2str(1/12*mass*(h^2 + w^2)));

        ixyElem = doc.createElement('ixy');
        inertia.appendChild(ixyElem);
        ixyElem.setTextContent(num2str(0));

        ixzElem = doc.createElement('ixz');
        inertia.appendChild(ixzElem);
        ixzElem.setTextContent(num2str(0));

        iyyElem = doc.createElement('iyy');
        inertia.appendChild(iyyElem);
        iyyElem.setTextContent(num2str(1/12*mass*(h^2 + l^2)));

        ixxElem = doc.createElement('iyz');
        inertia.appendChild(ixxElem);
        ixxElem.setTextContent(num2str(0));

        izzElem = doc.createElement('izz');
        inertia.appendChild(izzElem);
        izzElem.setTextContent(num2str(1/12*mass*(l^2 + w^2)));
    end

    function blueCylinder(doc, modelElem, name, x, y, z, mass, r, l)
        cylinder = doc.createElement('link');
        modelElem.appendChild(cylinder);
        cylinder.setAttribute('name', name);
        poseElem = doc.createElement('pose');
        cylinder.appendChild(poseElem);
        poseElem.setTextContent([num2str(x),' ',num2str(y),' ',num2str(z),' 0 0 0']);

        visual = doc.createElement('visual');
        cylinder.appendChild(visual);
        visual.setAttribute('name', java.util.UUID.randomUUID.toString()); 

        visualPose = doc.createElement('pose');
        visual.appendChild(visualPose);
        visualPose.setTextContent(['0 0 ',num2str(l/2), ' 0 0 0']);
        visualGeometry = doc.createElement('geometry');
        visual.appendChild(visualGeometry);
        geometryCylinder = doc.createElement('cylinder');
        visualGeometry.appendChild(geometryCylinder);
        lengthCylinder = doc.createElement('length');
        geometryCylinder.appendChild(lengthCylinder);
        lengthCylinder.setTextContent(num2str(l));
        radiusCylinder = doc.createElement('radius');
        geometryCylinder.appendChild(radiusCylinder);
        radiusCylinder.setTextContent(num2str(r));

        visualMaterial = doc.createElement('material');
        visual.appendChild(visualMaterial);
        scriptMaterial = doc.createElement('script');
        visualMaterial.appendChild(scriptMaterial);
        uriMaterial = doc.createElement('uri');
        scriptMaterial.appendChild(uriMaterial);
        uriMaterial.setTextContent('file://media/materials/scripts/gazebo.material');
        nameMaterial = doc.createElement('name');
        scriptMaterial.appendChild(nameMaterial);
        nameMaterial.setTextContent('Gazebo/Blue');
 
        collision = doc.createElement('collision');
        cylinder.appendChild(collision);
        collision.setAttribute('name', java.util.UUID.randomUUID.toString()); 
        collisionPose = doc.createElement('pose');
        collision.appendChild(collisionPose);
        collisionPose.setTextContent(['0 0 ',num2str(l/2), ' 0 0 0']);
        collisionGeometry = doc.createElement('geometry');
        collision.appendChild(collisionGeometry);
        geometryCylinder = doc.createElement('cylinder');
        collisionGeometry.appendChild(geometryCylinder);
        lengthCylinder = doc.createElement('length');
        geometryCylinder.appendChild(lengthCylinder);
        lengthCylinder.setTextContent(num2str(l));
        radiusCylinder = doc.createElement('radius');
        geometryCylinder.appendChild(radiusCylinder);
        radiusCylinder.setTextContent(num2str(r));

        inertial = doc.createElement('inertial');
        cylinder.appendChild(inertial);
        inertialPose = doc.createElement('pose');
        inertial.appendChild(inertialPose);
        inertialPose.setTextContent(['0 0 ',num2str(l/2),' 0 0 0']);

        inertialMass = doc.createElement('mass');
        inertial.appendChild(inertialMass);
        inertialMass.setTextContent(num2str(mass));
        inertia = doc.createElement('inertia');
        inertial.appendChild(inertia);

        % https://en.wikipedia.org/wiki/List_of_moments_of_inertia#List_of_3D_inertia_tensors  Note: we are using length, width, height where they use width, height, depth
        ixxElem = doc.createElement('ixx');
        inertia.appendChild(ixxElem);
        ixxElem.setTextContent(num2str(1/12*mass*l^2 + 1/4*mass*r^2));

        ixyElem = doc.createElement('ixy');
        inertia.appendChild(ixyElem);
        ixyElem.setTextContent(num2str(0));

        ixzElem = doc.createElement('ixz');
        inertia.appendChild(ixzElem);
        ixzElem.setTextContent(num2str(0));

        iyyElem = doc.createElement('iyy');
        inertia.appendChild(iyyElem);
        iyyElem.setTextContent(num2str(1/12*mass*l^2 + 1/4*mass*r^2));

        ixxElem = doc.createElement('iyz');
        inertia.appendChild(ixxElem);
        ixxElem.setTextContent(num2str(0));

        izzElem = doc.createElement('izz');
        inertia.appendChild(izzElem);
        izzElem.setTextContent(num2str(1/2*mass*r^2));
    end

    function polyline(doc, modelElem, name, mass, height, points)
        % TODO: allow this to connect to simple shapes via joints
        polyline = doc.createElement('link');
        modelElem.appendChild(polyline);
        polyline.setAttribute('name', name);
        visual = doc.createElement('visual');
        % using a unique ID fixes a caching problem with the gazebo GUI
        visual.setAttribute('name', java.util.UUID.randomUUID.toString());
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
        visualMaterial = doc.createElement('material');
        visual.appendChild(visualMaterial);
        scriptMaterial = doc.createElement('script');
        visualMaterial.appendChild(scriptMaterial);
        uriMaterial = doc.createElement('uri');
        scriptMaterial.appendChild(uriMaterial);
        uriMaterial.setTextContent('file://media/materials/scripts/gazebo.material');
        nameMaterial = doc.createElement('name');
        scriptMaterial.appendChild(nameMaterial);
        nameMaterial.setTextContent('Gazebo/OrangeTransparentOverlay');
        
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
    end

    function connectWithFixedJoint(doc, modelElem, jointName, parentLinkName, childLinkName, x, y, z)
        joint = doc.createElement('joint');
        modelElem.appendChild(joint);
        joint.setAttribute('name',jointName);
        joint.setAttribute('type','fixed');
    
        parentLink = doc.createElement('parent');
        joint.appendChild(parentLink);
        parentLink.setTextContent(parentLinkName);
        childLink = doc.createElement('child');
        joint.appendChild(childLink);
        childLink.setTextContent(childLinkName);
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