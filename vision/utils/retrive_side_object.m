function [outputArg1,outputArg2] = retrive_side_object(inputArg1,inputArg2)
%UNTITLED2 Summary of this function goes here
% if side:
% 	1. move arm to the limit position in x direction. 
% 	2. shift arm in y direction, (adjust z a little bit) (we do not need to worry about collision 
% 	   between finger and objects)
% 	3. turn the orientation of the arm.
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
target_pos = [-0.1491; 0.7076; 0.5581];

robotArm.cartesian_pose_client(pos+[-0.1; -0.20; 0.1], -tag_ori, 0);

% shift in y direction
robotArm.cartesian_pose_client([0.; 0.06; 0.03], robotArm.getOri(), 1);

end

