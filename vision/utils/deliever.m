function [outputArg1,outputArg2] = deliever(hObject)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

robotArm = RobotRaconteur.Connect('tcp://localhost:4567/KinovaJointServer/Kinova');
handles = guidata(hObject);
cur_location = lower(handles.cur_location);
if contains(cur_location , 'table')
    pos = handles.locations.deliever.table(1:3);
    ori = handles.locations.deliever.table(4:7);
else
    pos = handles.locations.deliever.table(1:3);
    ori = handles.locations.deliever.table(4:7);
end
robotArm.cartesian_pose_client(pos, ori, 0);
pause(1)
robotArm.closeFinger([0.0; 0.0; 0.0])
pause(1)
end

