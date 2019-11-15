function [outputArg1,outputArg2] = retrive_object(inputArg1,inputArg2)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
robotArm = RobotRaconteur.Connect('tcp://192.168.1.108:4567/KinovaJointServer/Kinova');
peripherals = RobotRaconteur.Connect('tcp://192.168.1.108:1234/KinovaPeripheralsServer/peripherals');

%% simple test
% record the current position and orientation. Use the joystick to move the
% arm. And let the arm go back to the origin position
% points = {};
% points{1}.pos = robotArm.getPos();
% points{1}.ori = robotArm.getOri();
% robotArm.cartesianPathTraj(points);
% robotArm.execute(10);

%% test the scene
% pos = robotArm.getPos();
% ori = robotArm.getOri();
% % addBox(self, name, dim, pos)
% robotArm.addBox('box1', [0.15; 0.15; 0.15], pos + [0; 0.3; 0])

%% move arm in the scenes. Need the data about the tag
% init position:
% robotArm.cartesian_pose_client([0.; -0.1; 0.05], init_ori, 1);

init_pos = [ -0.1418; 0.4378; 0.2991];
init_ori = [0.0251; 0.7279; 0.6834; 0.0502];
robotArm.cartesian_pose_client(init_pos, init_ori, 0);
points = {};

% detect the food
points{1}.pos = robotArm.getPos() + [-0.05; -0.05; 0.1];
points{1}.ori = init_ori;
% robotArm.cartesian_pose_client(points{1}.pos, points{1}.ori, 0);

%points{1}.pos = init_pos - [0; 0.15; 0];
%points{1}.ori = init_ori;
%robotArm.cartesianPathTraj(points')

pause(3)
tag_pose = peripherals.lookUptransforms();
pos = tag_pose.position;
tag_ori = tag_pose.quaternion;
disp('the tag position')
disp(pos)
target_pos = [-0.1491; 0.7076; 0.5581];

robotArm.cartesian_pose_client(pos+[-0.05; -0.20; 0.1], -tag_ori, 0);

objects = ["water"];
load('model/rcnn_3objects.mat')
rcnn = rcnn_food_water;
result = objLocalization(rcnn, objects);

% move the object sideway
robotArm.cartesian_pose_client(robotArm.getPos()+[0.2; 0.0; 0.], robotArm.getOri(), 0);
robotArm.closeFinger([1700.0; 1700.0; 1700.0])


robotArm.cartesian_pose_client(robotArm.getPos()+[-0.1; -0.15; 0.], robotArm.getOri(), 0);

objects = ["cola"];
result = objLocalization(rcnn, objects);

robotArm.cartesian_pose_client(robotArm.getPos()+[0.; -0.3; 0.1], robotArm.getOri(), 0);

% points{2}.pos = points{1}.pos;
% points{2}.pos(1) = tag_pose.position(1) - 0.125;
% points{2}.ori = points{1}.ori;
% 
% points{3}.pos = points{2}.pos;
% points{3}.pos(2) = tag_pose.position(2) +0.05 ;
% points{3}.ori = points{1}.ori;
% 
% points{4}.pos = points{3}.pos + [0.175; 0; 0];
% points{4}.ori = points{1}.ori;
% 
% points{5}.pos = points{4}.pos + [-0.10; -0.15; 0];
% points{5}.ori = points{1}.ori;
% 
% points{6}.pos = points{5}.pos ;
% points{6}.pos(1) = points{2}.pos(1);
% points{6}.ori = points{2}.ori;
% 
% points{7}.pos = points{6}.pos;
% points{7}.pos(2) = tag_pose.position(2) +0.13 ;
% points{7}.ori = points{6}.ori;



end

