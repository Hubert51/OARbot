
camera = RobotRaconteur.Connect('tcp://localhost:2345/KinovaCameraServer/Camera');
robotArm = RobotRaconteur.Connect('tcp://localhost:4567/KinovaJointServer/Kinova');
robotArm.closeFinger([0.0; 0.0; 0.0])

deltaTheta = 10/180*pi ;
r = 0.305; % The radius of the fridge door

init_pos = [0.0885479301214; -0.545893788338; 0.13304695487];
init_ori = [0.716; -0.029; 0.016; 0.697];
fridge_pos = [0.1599; -0.6973; 0.1732];
fridge_ori = [0.5160; 0.4518; -0.5073; 0.5218];

init2gripper = [0.085; -0.165; 0.05];


robotArm.cartesian_pose_client(init_pos, init_ori, 0);

pause(2)
% robotArm.cartesian_pose_client([-0.093; 0.0; 0], init_ori, 1);

%% approach the door
R = axang2rotm([1 0 0 0.5*pi]);
result = R * quat2rotm(init_ori');
robotArm.cartesian_pose_client(init_pos, rotm2quat(result)', 0);

R = axang2rotm([0 1 0 -0.5*pi/9]);
result = R * quat2rotm(robotArm.getOri()');
% robotArm.cartesian_pose_client(init_pos, rotm2quat(result)', 0);

robotArm.cartesian_pose_client(init_pos+[0.150; -0.135; 0.055], rotm2quat(result)', 0);

robotArm.closeFinger([6000.0; 6000.0; 6000.0])

%% open the door
fix_pos = robotArm.getPos();
for j = 1:10
    R = axang2rotm([0 0 1 0.5*pi/9]);
    result = R * quat2rotm(robotArm.getOri()');
    position_temp = fix_pos + [-r*(1-cos(deltaTheta*(j))); r*(sin(deltaTheta*(j) ));  0];
    robotArm.cartesian_pose_client(position_temp, rotm2quat(result)', 0);


end
robotArm.closeFinger([0.0; 0.0; 0.0])
pause(1)
robotArm.cartesian_pose_client(move_out_pos1, move_out_ori1, 0);
robotArm.cartesian_pose_client(move_out_pos2, move_out_ori2, 0);
robotArm.cartesian_pose_client(move_out_pos3, move_out_ori3, 0);
robotArm.cartesian_pose_client(move_out_pos4, move_out_ori4, 0);
%robotArm.cartesian_pose_client([-0.1; 0; 0.02], move_out_ori4, 1);

pause(4)

robotArm.cartesian_pose_client(move_out_pos5+[0;0;0.03], move_out_ori5, 0);
pause(0.5)
robotArm.closeFinger([6000.0; 6000.0; 6000.0])
pause(0.5)
robotArm.cartesian_pose_client(move_out_pos4, move_out_ori4, 0);
robotArm.cartesian_pose_client(move_out_pos6, move_out_ori6, 0);
pause(2)

robotArm.closeFinger([0.0; 0.0; 0.0])




%% 0.09 -0.02

