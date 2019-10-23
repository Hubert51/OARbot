function [outputArg1,outputArg2] = deliver(hObject)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

robotArm = RobotRaconteur.Connect('tcp://192.168.1.117:4567/KinovaJointServer/Kinova');
% handles = guidata(hObject);
% cur_location = lower(handles.cur_location);
% if contains(cur_location , 'table')
%     pos = handles.locations.deliever.table(1:3);
%     ori = handles.locations.deliever.table(4:7);
% else
% %     robotArm.moveBase([0.0; 0.15])
% %     pause(10)
% end

% move away from the fridge.
robotArm.moveBase([0.0; 0.15])
pause(4)
robotArm.moveBase([0.0; 0.])


% middle stage
robotArm.cartesian_pose_client([-0.2478; 0.4260; 0.3789], [0.3346; -0.6231; -0.6135; 0.3513], 0);
pause(1)

% first demo end point
robotArm.cartesian_pose_client([  -0.345; -0.4037; 0.3724], [0.7063; -0.0969; -0.0651; 0.6982], 0);
pause(1)

% for this position, the robot will block the human. So we decide use
% previous one
% robotArm.cartesian_pose_client([  0.2103; -0.3125; 0.3898], [ 0.657; 0.2940; 0.4077; 0.5615], 0);
% pause(1)
end

