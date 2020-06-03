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