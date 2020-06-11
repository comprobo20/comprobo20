function connectWithContinuousJoint(doc, jointName, parentLinkName, childLinkName, x, y, z, offsetX, offsetY, offsetZ)
    if nargin > 7
        masslessLink = doc.createElement('link');
        doc.getDocumentElement().appendChild(masslessLink);
        intermediateLinkName = [parentLinkName,'_',childLinkName,'_dummy'];
        masslessLink.setAttribute('name', intermediateLinkName);
        connectWithFixedJoint(doc, [jointName,'_fixed'], intermediateLinkName, childLinkName, offsetX, offsetY, offsetZ);
        childLinkName = intermediateLinkName;
    end
    joint = doc.createElement('joint');
    doc.getDocumentElement().appendChild(joint);
    joint.setAttribute('name',jointName);
    joint.setAttribute('type','continuous');

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