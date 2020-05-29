function spawnBox(modelName)
% TODO: controrl width height length and start position
system(['source ~/catkin_ws/devel/setup.bash; rosrun xacro xacro ~/catkin_ws/src/comprobo18/com_workshop/urdf/redBox.xacro | rosrun gazebo_ros spawn_model -stdin -model ',modelName,' -urdf']);
end