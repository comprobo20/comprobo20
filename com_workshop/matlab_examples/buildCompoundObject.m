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
end