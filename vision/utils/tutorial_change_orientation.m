
% change the endeffector oeientation
robotArm = RobotRaconteur.Connect('tcp://192.168.1.108:4567/KinovaJointServer/Kinova');

% get ori: [x, y, z, w]
init_ori = robotArm.getOri();
% ori: [w, x, y, z]
init_ori = [init_ori(4); init_ori(1:3)];

% do rotation, the frame is base. 
R = axang2rotm([0 0 1 0.2*pi]);
result = R * quat2rotm(init_ori');
result = rotm2quat(result)';

% ori: [x, y, z, w]
result = [result(2:4); result(1)];
robotArm.cartesian_pose_client(robotArm.getPos(), result, 0);
