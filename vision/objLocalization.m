function [result_shift] = objLocalization(rcnn, bbox1)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
    camera_shift = 0.02;
    height_shift = 0.105;
    camera = RobotRaconteur.Connect('tcp://localhost:2345/KinovaCameraServer/Camera');
    robotArm = RobotRaconteur.Connect('tcp://localhost:4567/KinovaJointServer/Kinova');
    cur_pos = robotArm.getPos();
    cur_ori = robotArm.getOri();
    % bbox1 = objDetection(rcnn, 8);
    offset = [0.05; 0; 0];

    if bbox1(1)+bbox1(3)/2 < 320
        dir = 1;
    else
        dir = -1;
    end

    robotArm.cartesian_pose_client(offset*dir, cur_ori, 1);
    bbox2 = objDetection(rcnn, 8);
    shift = bbox2 - bbox1;
    
    % mvoe back
    robotArm.cartesian_pose_client(-offset*dir, cur_ori, 1);
    pixel_len = abs(0.05 / shift(1))
    img_center = [640 / 2; 0; 480/2] ;
    obj_center = [bbox1(1) + bbox1(3)/2; 0; bbox1(2) + bbox1(4)/2];
    
    result_shift = (obj_center-img_center)*pixel_len .* [-1*dir*dir; 0; 1];
    result_shift = result_shift + [camera_shift; 0; height_shift];
    % result_shift = result_shift + [camera_shift; -camera.getDepth(bbox1')/1000+0.008; height_shift];


end

