function polyline(doc, modelElem, name, massOrDensityRatio, height, points, material, isBoat)
    if nargin < 7
        material = 'Gazebo/OrangeTransparentOverlay';
    end
    if nargin < 8
        isBoat = false;
    end
    % if we have a boat, the massOrDensityRatio is interpreted as a density
    % to water (1000 kg/m^3).  If it isn't a boat, then we just assume that
    % it is the mass itself
    [xcom, ycom, area, Ix, Iy, Ixy] = polyCOM(points);

    if isBoat
        mass = area*height*1000*massOrDensityRatio;
    else
        mass = massOrDensityRatio;
    end
    density = mass/(area*height);
    Ix = Ix*height*density;
    Iy = Iy*height*density;
    Ixy = Ixy*height*density;
    Ixz = 0;
    Iz = Ix + Iy;
    Iyz = 0;
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
    com = [xcom ycom height/2];  % need to determine this x and y
    inertialPose.setTextContent([num2str(com(1)),' ',num2str(com(2)),' ',num2str(com(3)),' 0 0 0']);
    inertialMass = doc.createElement('mass');
    inertial.appendChild(inertialMass);
    inertialMass.setTextContent(num2str(mass));
    inertia = doc.createElement('inertia');
    inertial.appendChild(inertia);

    ixx = doc.createElement('ixx');
    inertia.appendChild(ixx);
    ixx.setTextContent(num2str(Ix));

    ixy = doc.createElement('ixy');
    inertia.appendChild(ixy);
    ixy.setTextContent(num2str(Ixy));

    ixz = doc.createElement('ixz');
    inertia.appendChild(ixz);
    ixz.setTextContent(num2str(Ixz));

    iyy = doc.createElement('iyy');
    inertia.appendChild(iyy);
    iyy.setTextContent(num2str(Iy));

    iyz = doc.createElement('iyz');
    inertia.appendChild(iyz);
    iyz.setTextContent(num2str(Iyz));

    izz = doc.createElement('izz');
    inertia.appendChild(izz);
    izz.setTextContent(num2str(Iz));
    
    if isBoat
        % add the buoyancy
        plugin = doc.createElement('plugin');
        modelElem.appendChild(plugin);
        plugin.setAttribute('filename','libbuoyancy_gazebo_plugin.so');
        plugin.setAttribute('name','BuoyancyPlugin.so');
        fluidDensity = doc.createElement('fluid_density');
        plugin.appendChild(fluidDensity);
        fluidDensity.setTextContent('1000');
        fluidLevel = doc.createElement('fluid_level');
        plugin.appendChild(fluidLevel);
        fluidLevel.setTextContent('0.0');
        linearDrag = doc.createElement('linear_drag');
        plugin.appendChild(linearDrag);
        linearDrag.setTextContent('5');
        angularDrag = doc.createElement('angular_drag');
        plugin.appendChild(angularDrag);
        angularDrag.setTextContent('2.0');
        buoyancy = doc.createElement('buoyancy');
        plugin.appendChild(buoyancy);
        buoyancy.setAttribute('name','buoyancy');
        linkName = doc.createElement('link_name');
        buoyancy.appendChild(linkName);
        linkName.setTextContent(name);
       
        buoyancyGeometry = doc.createElement('geometry');
        buoyancy.appendChild(buoyancyGeometry);
        geometryPolyline = doc.createElement('polyline');
        buoyancyGeometry.appendChild(geometryPolyline);
        for i = 1:size(points,1)
            polylinePoint = doc.createElement('point');
            geometryPolyline.appendChild(polylinePoint);
            polylinePoint.setTextContent([num2str(points(i,1)),' ',num2str(points(i,2))]);
        end
        polylineHeight = doc.createElement('height');
        polylineHeight.setTextContent(num2str(height));
        geometryPolyline.appendChild(polylineHeight);
    end
    
    function [xcom, ycom, area, Ix, Iy, Ixy] = polyCOM(points)
        area = 0;
        for i = 1:size(points,1)-1
            xi = points(i,1);
            xiplus1 = points(i+1,1);
            yi = points(i,2);
            yiplus1 = points(i+1,2);
            area = area + 0.5*(xi*yiplus1 - xiplus1*yi);
        end
        xcom = 0;
        ycom = 0;
        Ix = 0;
        Iy = 0;
        Ixy = 0;
        for i = 1:size(points,1)-1
            xi = points(i,1);
            xiplus1 = points(i+1,1);
            yi = points(i,2);
            yiplus1 = points(i+1,2);
            xcom = xcom + 1/(6*area)*(xi+xiplus1)*(xi*yiplus1 - xiplus1*yi);
            ycom = ycom + 1/(6*area)*(yi+yiplus1)*(xi*yiplus1 - xiplus1*yi);
            Ix = Ix + 1/12*(xi*yiplus1 - xiplus1*yi)*(xi^2 + xi*xiplus1 + xiplus1^2);
            Iy = Iy + 1/12*(xi*yiplus1 - xiplus1*yi)*(yi^2 + yi*yiplus1 + yiplus1^2);
            Ixy = Ixy + 1/24*(xi*yiplus1 - xiplus1*yi)*(xi*yiplus1 + 2*xi*yi + 2*xiplus1*yiplus1 + xiplus1*yi);
        end
    end
end