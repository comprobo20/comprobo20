function spawnModel(modelName,xml,x,y,z,roll,pitch,yaw,useSDF,deleteSvc,spawnModelSvc)
if nargin < 6
    roll = 0;
    pitch = 0;
    yaw = 0;
end
if nargin < 9
    useSDF = false;
end
if nargin < 10
    deleteSvc = rossvcclient('/gazebo/delete_model');
end
msg = rosmessage(deleteSvc);
msg.ModelName = modelName;
% TODO DEBUG: in Docker on Mac this hangs until you delete the model from
% the gzweb GUI
call(deleteSvc, msg)
if nargin < 10
    clear deleteSvc;
end

if nargin < 11
    if useSDF
        spawnModelSvc = rossvcclient('/gazebo/spawn_sdf_model');
    else
        spawnModelSvc = rossvcclient('/gazebo/spawn_urdf_model');
    end
end

msg = rosmessage(spawnModelSvc);
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
call(spawnModelSvc, msg)
if nargin < 11
    clear spawnModelSvc;
end
end
