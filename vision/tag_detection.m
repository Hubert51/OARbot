camera = RobotRaconteur.Connect('tcp://localhost:2345/KinovaCameraServer/Camera');
robotArm = RobotRaconteur.Connect('tcp://localhost:4567/KinovaJointServer/Kinova');
init_pos = [-0.0515; -0.4863; 0.1694 ];
init_ori = [    0.6935; -0.0466; -0.0242; 0.7185];

% robotArm.cartesian_pose_client(init_pos, init_ori, 0);


pos = camera.getPos();
ori = camera.getOri();
robotArm.cartesian_pose_client([0; 0; 0], [1; 0; 0; 0], 1);

% robotArm.cartesian_pose_client([0; 0; 0], ori, 1);


