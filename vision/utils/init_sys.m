function [my_rcnn] = init_sys(location)
    %UNTITLED Summary of this function goes here
    %   Detailed explanation goes here
    camera = RobotRaconteur.Connect('tcp://localhost:2345/KinovaCameraServer/Camera');
    robotArm = RobotRaconteur.Connect('tcp://localhost:4567/KinovaJointServer/Kinova');
    load('model/rcnn_water_food_box.mat')
    my_rcnn = rcnn;
    robotArm.closeFinger([0.0; 0.0; 0.0]);

    if strcmp(location, 'on the table')
        init_pos = [-0.0515; -0.4863; 0.1694 ];
        init_ori = [0.6935; -0.0466; -0.0242; 0.7185];
    else
        init_pos = [-0.0515; -0.4863; 0.1694 ];
        init_ori = [0.6935; -0.0466; -0.0242; 0.7185];
    end
    
    first_grab = [ 0.0004; -0.8686; 0.0735];
    second_grab = [ 0.0004; -0.8686; 0.3135];
    second_pos = [ -0.0297; -0.5745; 0.285];
    second_ori = init_ori;
    grab_ori = [0.7057; 0.0761; -0.0410; 0.7032];
    robotArm.cartesian_pose_client(init_pos, init_ori, 0);
    init = 1;
end

