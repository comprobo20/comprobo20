function spawnModel(modelName,xml,x,y,z,roll,pitch,yaw,useSDF)
if nargin < 6
    roll = 0;
    pitch = 0;
    yaw = 0;
end
if nargin < 9
    useSDF = false;
end
svc = rossvcclient('/gazebo/delete_model')
msg = rosmessage(svc);
msg.ModelName = modelName;
call(svc, msg)
clear svc;

if useSDF
    svc = rossvcclient('/gazebo/spawn_sdf_model');
else
    svc = rossvcclient('/gazebo/spawn_urdf_model');
end

msg = rosmessage(svc);   
msg.ModelName = modelName;
msg.ModelXml = xml;

msg.InitialPose.Position.X = x;
msg.InitialPose.Position.Y = y;
msg.InitialPose.Position.Z = z;

quat = eul2quat([yaw pitch roll]);
msg.InitialPose.Orientation.W = quat(1);
msg.InitialPose.Orientation.X = quat(2);
msg.InitialPose.Orientation.Y = quat(3);
msg.InitialPose.Orientation.Z = quat(4);
call(svc, msg)
clear svc;
end