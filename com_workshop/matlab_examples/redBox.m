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