function [ ] = add_fridge(base2tag)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    robotArm = RobotRaconteur.Connect('tcp://localhost:4568/KinovaJointServer/Kinova');

    depth = 0.8;
    width = 0.48;
    height = 0.255;
    % add fridge into the rviz
    
    
    
    % front door
    
    center_y = base2tag.position(2) - 0.03;
    center_z = base2tag.position(3) + 0.0225;
    center_x = base2tag.position(1) + 0.4;
    %joint_names = robotArm.joint_names;
    %attached_joint = joint_names(7);
    temp_center_x = center_x - 0.5 * depth;
    robotArm.addBox('fridge_front', [width; 0.07; height], [temp_center_x; center_y; center_z]);
    robotArm.attachBox('j2n6s300_end_effector', 'fridge_front');
    
end