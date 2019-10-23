function [my_rcnn] = init_sys(hObject)
    disp('in init')
    %UNTITLED Summary of this function goes here
    %   Detailed explanation goes here
    handles = guidata(hObject);
    robotArm = RobotRaconteur.Connect('tcp://192.168.1.117:4567/KinovaJointServer/Kinova');
    load('model/rcnn_3objects.mat')
    my_rcnn = rcnn_food_water;
    robotArm.closeFinger([0.0; 0.0; 0.0]);
    location = handles.listbox1.String(handles.listbox1.Value);
    location = location{1};
    if contains(location,'table')
        init_pos = handles.locations.initial.table(1:3);
        init_ori = handles.locations.initial.table(4:7);
        handles.cur_location = 'table';
        robotArm.cartesian_pose_client(init_pos, init_ori, 0);

    else
        handles.cur_location = 'fridge';
        
        % intermediate step to move the arm approach the fridge
%         init_pos = [-0.425; -0.0278; 0.4978 ];
%         init_ori = [ 0.5849; -0.4320; -0.2887; 0.6228];
%         robotArm.cartesian_pose_client(init_pos, init_ori, 0);
        
        init_pos = [-0.1401; 0.5012; -0.0236];
        init_ori = [0.0026; 0.7361; 0.6761; 0.0310];
        % fridge_pos = [0.1599; -0.6973; 0.1732];
        % fridge_ori = [0.5160; 0.4518; -0.5073; 0.5218];
        robotArm.cartesian_pose_client(init_pos, init_ori, 0);
    end
    
    first_grab = [ 0.0004; -0.8686; 0.0735];
    second_grab = [ 0.0004; -0.8686; 0.3135];
    second_pos = [ -0.0297; -0.5745; 0.285];
    second_ori = init_ori;
    grab_ori = [0.7057; 0.0761; -0.0410; 0.7032];
    init = 1;
    guidata(hObject, handles);

end

