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