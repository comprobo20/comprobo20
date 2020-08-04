function polyline(doc, modelElem, name, componentDensityRatios, height, regions, materials, isBoat, Z, meshBoatSTLURI)
    % TODO: need to refactor the part about the mesh.  The reason for the
    % overlap currently is from the need to calculate the intertial
    % parameters
    if nargin < 7
        materials = {'Gazebo/OrangeTransparentOverlay'};
    end
    if nargin < 8
        isBoat = false;
    end
    if nargin < 9
        meshBoatSTLURI = '';
    end
    useSTLOverride = ~isempty(meshBoatSTLURI);
    % precision of output (extra digits help when polygon points are close
    % together)
    num2strPrecision = 10;
    % drag in case it's a boat
    linearDrag = .2;
    angularDrag = .5;
    if ~iscell(regions)
        % assume one region with the Z base of 0
        regions = {regions};
        materials = {materials};
        Z = 0;
    end

    zcomOverall = 0;
    xcomOverall = 0;
    ycomOverall = 0;
    mass = 0;
    regionMass = [];

    for r = 1:length(regions)
        [xcom, ycom, area, Ix, Iy, Ixy] = polyCOM(regions{r});

        zcom = Z(r) + height/2;
        % if we have a boat, the componentDensityRatios is interpreted as a density
        % to water ratio (assumed ot be 1000 kg/m^3).  If it isn't a boat, then we
        % just assume that it is the mass itself
        if ~isBoat
            regionMass(end+1) = componentDensityRatios(r);
        else
            regionMass(end+1) = height*area*componentDensityRatios(r)*1000;
        end
        mass = mass + regionMass(r);
        xcomOverall = xcomOverall + xcom*regionMass(r);
        ycomOverall = ycomOverall + ycom*regionMass(r);
        zcomOverall = zcomOverall + zcom*regionMass(r);
    end
    xcomOverall = xcomOverall / mass;
    ycomOverall = ycomOverall / mass;
    zcomOverall = zcomOverall / mass;
    disp(['Total mass ', num2str(mass, num2strPrecision)]);

    % now that we have the overall CoM, we can compute the moment of
    % inertia tensor
    IOverall = zeros(3);
    for r = 1:length(regions)
        [xcomRegion, ycomRegion, ~, Ix, Iy, Ixy] = polyCOM(regions{r});
        zcomRegion = Z(r) + height/2;
        regionDensity = 1000*componentDensityRatios(r);
        IxxAboutRegionCoM = Ix*height*regionDensity  + 1/12*regionMass(r)*height^2;
        IyyAboutRegionCoM = Iy*height*regionDensity  + 1/12*regionMass(r)*height^2;
        IxyAboutRegionCoM = -Ixy*height*regionDensity;
        IzzAboutRegionCoM = (Ix + Iy)*height*regionDensity;
        IxzAboutRegionCoM = 0;
        IyzAboutRegionCoM = 0;
        % https://books.google.com/books?id=9vt9DwAAQBAJ&pg=PA457&lpg=PA457&dq=shifting+inertia+tensor+to+a+new+point&source=bl&ots=9oD13LyQiQ&sig=ACfU3U2JB5Pt7y7vgtPQnBnaKyQp4O3G9g&hl=en&sa=X&ved=2ahUKEwiVitKFx4nqAhWySjABHXTkAPMQ6AEwD3oECAoQAQ#v=onepage&q=shifting%20inertia%20tensor%20to%20a%20new%20point&f=false
        Ic = [IxxAboutRegionCoM IxyAboutRegionCoM IxzAboutRegionCoM;...
              IxyAboutRegionCoM IyyAboutRegionCoM IyzAboutRegionCoM;...
              IxzAboutRegionCoM IyzAboutRegionCoM IzzAboutRegionCoM];
        shift = [xcomOverall - xcomRegion; ycomOverall - ycomRegion; zcomOverall - zcomRegion];
        AugmentationDueToNewCoM = regionMass(r)*[shift(2)^2+shift(3)^2, -shift(1)*shift(2), -shift(1)*shift(3);...
                                                 -shift(1)*shift(2), shift(1)^2+shift(3)^2, -shift(2)*shift(3);...
                                                 -shift(1)*shift(3), -shift(2)*shift(3), shift(1)^2+shift(2)^2];
        Ishifted = Ic + AugmentationDueToNewCoM;
        IOverall = IOverall + Ishifted;
    end

    Ixx = IOverall(1,1);
    Ixy = IOverall(1,2);
    Ixz = IOverall(1,3);
    Iyy = IOverall(2,2);
    Iyz = IOverall(2,3);
    Izz = IOverall(3,3);
    polyline = doc.createElement('link');
    modelElem.appendChild(polyline);
    polyline.setAttribute('name', name);

    if useSTLOverride
        visual = doc.createElement('visual');
        visual.setAttribute('name', java.util.UUID.randomUUID.toString());
        polyline.appendChild(visual);
        visualGeometry = doc.createElement('geometry');
        visual.appendChild(visualGeometry);
        meshElem = doc.createElement('mesh');
        visualGeometry.appendChild(meshElem);
        uriElem = doc.createElement('uri');
        meshElem.appendChild(uriElem);
        uriElem.setTextContent(meshBoatSTLURI);
        visualMaterial = doc.createElement('material');
        visual.appendChild(visualMaterial);
        scriptMaterial = doc.createElement('script');
        visualMaterial.appendChild(scriptMaterial);
        uriMaterial = doc.createElement('uri');
        scriptMaterial.appendChild(uriMaterial);
        uriMaterial.setTextContent('file://media/materials/scripts/gazebo.material');
        nameMaterial = doc.createElement('name');
        scriptMaterial.appendChild(nameMaterial);
        nameMaterial.setTextContent('Gazebo/WoodPallet')
        collision = doc.createElement('collision');
        collision.setAttribute('name', java.util.UUID.randomUUID.toString());
        polyline.appendChild(collision);
        collisionGeometry = doc.createElement('geometry');
        collision.appendChild(collisionGeometry);
        collisionMesh = doc.createElement('mesh');
        collisionGeometry.appendChild(collisionMesh);
        collisionUriElem = doc.createElement('uri');
        collisionMesh.appendChild(collisionUriElem);
        collisionUriElem.setTextContent(meshBoatSTLURI);

        surface = doc.createElement('surface');
        collision.appendChild(surface);
        % TODO: factor out contact
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
    else
        for i = 1:length(regions)
            points = regions{i};
            visual = doc.createElement('visual');
            % using a unique ID fixes a caching problem with the gazebo GUI
            visual.setAttribute('name', java.util.UUID.randomUUID.toString());
            polyline.appendChild(visual);
            visualPose = doc.createElement('pose');
            visual.appendChild(visualPose);
            visualPose.setTextContent(['0 0 ',num2str(Z(i), num2strPrecision),' 0 0 0']);
            visualGeometry = doc.createElement('geometry');
            visual.appendChild(visualGeometry);
            geometryPolyline = doc.createElement('polyline');
            visualGeometry.appendChild(geometryPolyline);
            for j = 1:size(points,1)
                polylinePoint = doc.createElement('point');
                geometryPolyline.appendChild(polylinePoint);
                polylinePoint.setTextContent([num2str(points(j,1), num2strPrecision),' ',num2str(points(j,2), num2strPrecision)]);
            end
            polylineHeight = doc.createElement('height');
            geometryPolyline.appendChild(polylineHeight);
            polylineHeight.setTextContent(num2str(height, num2strPrecision));
            visualMaterial = doc.createElement('material');
            visual.appendChild(visualMaterial);
            scriptMaterial = doc.createElement('script');
            visualMaterial.appendChild(scriptMaterial);
            uriMaterial = doc.createElement('uri');
            scriptMaterial.appendChild(uriMaterial);
            uriMaterial.setTextContent('file://media/materials/scripts/gazebo.material');
            nameMaterial = doc.createElement('name');
            scriptMaterial.appendChild(nameMaterial);
            nameMaterial.setTextContent(materials{i});
        end
        for i=1:length(regions)
            points = regions{i};
            collision = doc.createElement('collision');
            collision.setAttribute('name', java.util.UUID.randomUUID.toString());
            polyline.appendChild(collision);
            collisionPose = doc.createElement('pose');
            collision.appendChild(collisionPose);
            collisionPose.setTextContent(['0 0 ',num2str(Z(i), num2strPrecision),' 0 0 0']);
            collisionGeometry = doc.createElement('geometry');
            collision.appendChild(collisionGeometry);
            geometryPolyline = doc.createElement('polyline');
            collisionGeometry.appendChild(geometryPolyline);
            for j = 1:size(points,1)
                polylinePoint = doc.createElement('point');
                geometryPolyline.appendChild(polylinePoint);
                polylinePoint.setTextContent([num2str(points(j,1), num2strPrecision),' ',num2str(points(j,2), num2strPrecision)]);
            end
            polylineHeight = doc.createElement('height');
            geometryPolyline.appendChild(polylineHeight);
            polylineHeight.setTextContent(num2str(height, num2strPrecision));

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
        end
    end
    inertial = doc.createElement('inertial');
    polyline.appendChild(inertial);
    inertialPose = doc.createElement('pose');
    inertial.appendChild(inertialPose);
    com = [xcomOverall ycomOverall zcomOverall];
    inertialPose.setTextContent([num2str(com(1), num2strPrecision),' ',num2str(com(2), num2strPrecision),' ',num2str(com(3), num2strPrecision),' 0 0 0']);
    inertialMass = doc.createElement('mass');
    inertial.appendChild(inertialMass);
    inertialMass.setTextContent(num2str(mass, num2strPrecision));
    inertia = doc.createElement('inertia');
    inertial.appendChild(inertia);

    ixx = doc.createElement('ixx');
    inertia.appendChild(ixx);
    ixx.setTextContent(num2str(Ixx, num2strPrecision));

    ixy = doc.createElement('ixy');
    inertia.appendChild(ixy);
    ixy.setTextContent(num2str(Ixy, num2strPrecision));

    ixz = doc.createElement('ixz');
    inertia.appendChild(ixz);
    ixz.setTextContent(num2str(Ixz, num2strPrecision));

    iyy = doc.createElement('iyy');
    inertia.appendChild(iyy);
    iyy.setTextContent(num2str(Iyy, num2strPrecision));

    iyz = doc.createElement('iyz');
    inertia.appendChild(iyz);
    iyz.setTextContent(num2str(Iyz, num2strPrecision));

    izz = doc.createElement('izz');
    inertia.appendChild(izz);
    izz.setTextContent(num2str(Izz, num2strPrecision));
    if isBoat
        plugin = doc.createElement('plugin');
        modelElem.appendChild(plugin);
        plugin.setAttribute('filename','libbuoyancy_gazebo_plugin.so');
        plugin.setAttribute('name',java.util.UUID.randomUUID.toString());
        fluidDensity = doc.createElement('fluid_density');
        plugin.appendChild(fluidDensity);
        fluidDensity.setTextContent('1000');
        fluidLevel = doc.createElement('fluid_level');
        plugin.appendChild(fluidLevel);
        fluidLevel.setTextContent('0.0');
        linearDragElem = doc.createElement('linear_drag');
        plugin.appendChild(linearDragElem);
        linearDragElem.setTextContent(num2str(linearDrag, num2strPrecision));
        angularDragElem = doc.createElement('angular_drag');
        plugin.appendChild(angularDragElem);
        angularDragElem.setTextContent(num2str(angularDrag, num2strPrecision));
        if useSTLOverride
            buoyancy = doc.createElement('buoyancy');
            plugin.appendChild(buoyancy);
            buoyancy.setAttribute('name',java.util.UUID.randomUUID.toString());
            linkName = doc.createElement('link_name');
            buoyancy.appendChild(linkName);
            linkName.setTextContent(name);
            massElem = doc.createElement('mass');
            buoyancy.appendChild(massElem);
            massElem.setTextContent(num2str(mass, num2strPrecision));
            buoyancyGeometry = doc.createElement('geometry');
            buoyancy.appendChild(buoyancyGeometry);
            buoyancyMesh = doc.createElement('mesh');
            buoyancyGeometry.appendChild(buoyancyMesh);
            buoyancyUriElem = doc.createElement('uri');
            buoyancyMesh.appendChild(buoyancyUriElem);
            buoyancyUriElem.setTextContent(meshBoatSTLURI);
        else
            for i=1:length(regions)
                points = regions{i};
                buoyancy = doc.createElement('buoyancy');
                plugin.appendChild(buoyancy);
                buoyancy.setAttribute('name',java.util.UUID.randomUUID.toString());
                linkName = doc.createElement('link_name');
                buoyancy.appendChild(linkName);
                linkName.setTextContent(name);
                massElem = doc.createElement('mass');
                buoyancy.appendChild(massElem);
                massElem.setTextContent(num2str(regionMass(i), num2strPrecision));

                buoyancyGeometry = doc.createElement('geometry');
                buoyancy.appendChild(buoyancyGeometry);
                geometryPolyline = doc.createElement('polyline');
                buoyancyGeometry.appendChild(geometryPolyline);

                for j = 1:size(points,1)
                    polylinePoint = doc.createElement('point');
                    geometryPolyline.appendChild(polylinePoint);
                    polylinePoint.setTextContent([num2str(points(j,1), num2strPrecision),' ',num2str(points(j,2), num2strPrecision)]);
                end
                polylineHeight = doc.createElement('height');
                polylineHeight.setTextContent(num2str(height, num2strPrecision));
                geometryPolyline.appendChild(polylineHeight);
                polylineZShift = doc.createElement('zshift');
                polylineZShift.setTextContent(num2str(Z(i), num2strPrecision));
                geometryPolyline.appendChild(polylineZShift);
            end
        end
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
        end
        % shift by xcom, ycom
        for i = 1:size(points,1)-1
            xi = points(i,1) - xcom;
            xiplus1 = points(i+1,1) - xcom;
            yi = points(i,2) - ycom;
            yiplus1 = points(i+1,2) - ycom;
            Ix = Ix + 1/12*(xi*yiplus1 - xiplus1*yi)*(yi^2 + yi*yiplus1 + yiplus1^2);
            Iy = Iy + 1/12*(xi*yiplus1 - xiplus1*yi)*(xi^2 + xi*xiplus1 + xiplus1^2);
            Ixy = Ixy + 1/24*(xi*yiplus1 - xiplus1*yi)*(xi*yiplus1 + 2*xi*yi + 2*xiplus1*yiplus1 + xiplus1*yi);
        end
    end
end