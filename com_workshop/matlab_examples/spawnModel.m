function spawnModel(modelName,xml,x,y,z,useSDF)
if nargin < 6
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
call(svc, msg)
clear svc;
end