function [outputArg1,outputArg2] = add_fridge2(inputArg1,inputArg2)
    robotArm = RobotRaconteur.Connect('tcp://192.168.1.108:4567/KinovaJointServer/Kinova');
    peripherals = RobotRaconteur.Connect('tcp://192.168.1.108:1234/KinovaPeripheralsServer/peripherals');
% 
    tag_pose = peripherals.lookUptransforms();
    pos = tag_pose.position;
    tag_ori = tag_pose.quaternion;
    % pos = [ 0.4774; -0.3028; 0.6491 ];
    
    center_x = pos(1);
    center_y = pos(2) + (0.305-0.12)/2;
    center_z = pos(3) + (0.315-0.155)/2;
    
    depth = 0.43;
    width = 0.47;
    height = 0.54;    
    
    % add fridge into the rviz
    
    % front door
%     center_y = H_matrix(2, 4) - 0.03;
%     center_z = H_matrix(3, 4) + 0.0225;
%     center_x = H_matrix(1, 4) + 0.4;
    %joint_names = robotArm.joint_names;
    %attached_joint = joint_names(7);
%     temp_center_x = center_x - 0.5 * depth;
%     robotArm.addBox('fridge_front', [0.07; width; height], [temp_center_x; center_y; center_z]);
%     robotArm.attachBox('right_gripper', 'fridge_front');

    % test the rotate the object
    R = axang2rotm([0 0 1 0.5*pi]);
    result = R * quat2rotm([0,0,0,1]);
    rotm2quat(result)'
    robotArm.removeScene('fridge_inner')

    % left
    left_x = center_x - 0.5 * width;
    left_width = 0.015;
    robotArm.addBox('fridge_left2', [depth; left_width; height], [left_x;center_y; center_z]);
    
    % right 
    right_x = center_x + 0.5 * width;
    robotArm.addBox('fridge_right2', [depth; left_width; height], [right_x; center_y; center_z]);
    
    % upper
    upper_z = center_z + 0.5*height;
    upper_height = 0.01;
    robotArm.addBox('fridge_upper', [depth; width; upper_height], [center_x; center_y; upper_z]);
    
    % lower
    lower_z = center_z - 0.5*height;
    robotArm.addBox('fridge_lower', [depth; width; upper_height], [center_x; center_y; lower_z]);
    
    % inner part
    
    robotArm.addBox('fridge_inner', [0.31; width; 0.22], [center_x; center_y+0.08; lower_z+0.1]);

    
end

