function polyline(doc, modelElem, name, mass, height, points, material)
    if nargin < 7
        material = 'Gazebo/OrangeTransparentOverlay';
    end
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
    nameMaterial.setTextContent(material);

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