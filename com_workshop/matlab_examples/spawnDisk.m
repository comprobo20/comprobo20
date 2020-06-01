function spawnBox(modelName, x, y, z, m, r, l)
% TODO: if isunix, then just use xacro directly
[exit_code, output] = system(['/usr/local/bin/docker run  -v ~/Software/comprobo18/com_workshop/urdf/blueCylinder.xacro:/tmp/blueCylinder.xacro osrf/ros:melodic-desktop-full-bionic xacro /tmp/blueCylinder.xacro',...
    ' name:=',modelName,...
        ' mass:=',num2str(m),...
        ' length:=',num2str(l),...
        ' radius:=',num2str(r)]);

svc = rossvcclient('/gazebo/delete_model')
msg = rosmessage(svc);
msg.ModelName = modelName;
call(svc, msg)
clear svc;

svc = rossvcclient('/gazebo/spawn_urdf_model')
msg = rosmessage(svc);      
msg.ModelName = modelName;
msg.ModelXml = output;

msg.InitialPose.Position.X = x;
msg.InitialPose.Position.Y = y;
msg.InitialPose.Position.Z = z;
call(svc, msg)
clear svc;
end