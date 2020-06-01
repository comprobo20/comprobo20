function spawnXacro(modelName,fileName,x,y,z)
% TODO: if isunix, then just use xacro directly
[exit_code, output] = system(['/usr/local/bin/docker run  -v ',fileName,':/tmp/tospawn.xacro osrf/ros:melodic-desktop-full-bionic xacro /tmp/tospawn.xacro']);

svc = rossvcclient('/gazebo/delete_model')
msg = rosmessage(svc);
msg.ModelName = modelName;
call(svc, msg)
clear svc;

svc = rossvcclient('/gazebo/spawn_urdf_model')
msg = rosmessage(svc);

% system(['/usr/local/bin/docker run osrf/ros:melodic-desktop-full-bionic -it --rm rosrun xacro xacro ~/catkin_ws/src/comprobo18/com_workshop/urdf/redBox.xacro',...
%         
msg.ModelName = modelName;
msg.ModelXml = output;

msg.InitialPose.Position.X = x;
msg.InitialPose.Position.Y = y;
msg.InitialPose.Position.Z = z;
call(svc, msg)
clear svc;
end