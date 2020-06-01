function spawnXacro(modelName,xml,x,y,z)
svc = rossvcclient('/gazebo/delete_model')
msg = rosmessage(svc);
msg.ModelName = modelName;
call(svc, msg)
clear svc;

svc = rossvcclient('/gazebo/spawn_urdf_model')

msg = rosmessage(svc);   
msg.ModelName = modelName;
msg.ModelXml = xml;

msg.InitialPose.Position.X = x;
msg.InitialPose.Position.Y = y;
msg.InitialPose.Position.Z = z;
call(svc, msg)
clear svc;
end