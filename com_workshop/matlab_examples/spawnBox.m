function spawnBox(modelName, x, y, z, m, l, w, h)
% TODO: if isunix, then just use xacro directly
[exit_code, output] = system(['/usr/local/bin/docker run  -v ~/Software/comprobo18/com_workshop/urdf/redBox.xacro:/tmp/redBox.xacro osrf/ros:melodic-desktop-full-bionic xacro /tmp/redBox.xacro',...
    ' name:=',modelName,...
        ' mass:=',num2str(m),...
        ' length:=',num2str(l),...
        ' width:=',num2str(w),...
        ' height:=',num2str(h)]);

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
