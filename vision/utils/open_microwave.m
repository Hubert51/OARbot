function [outputArg1,outputArg2] = open_microwave(hObject,inputArg2)
    % open fridge
%     handles = guidata(hObject);
%     handles.text2.String = 'Open the Fridge door';

    deltaTheta = 10/180*pi ;
    r = 0.305; % The radius of the fridge door
    r = 0.38; % radius of the microwave
    robotArm = RobotRaconteur.Connect('tcp://192.168.1.117:4567/KinovaJointServer/Kinova');
    peripherals = RobotRaconteur.Connect('tcp://192.168.1.117:1234/KinovaPeripheralsServer/peripherals');
    
    % move to the left 
    robotArm.moveBase2([-0.15; -0.])
    pause(8)
    robotArm.moveBase2([-0.0005; 0])
    
    init_pos = [0.08; 0.4834; 0.2499];
    init_ori = [0.0026; 0.7361; 0.6761; 0.0310];
    robotArm.cartesian_pose_client(init_pos, init_ori, 0);

    pause(3)
    % the orientation and position of tag reference base frame of kinova
    tag_pose = peripherals.lookUptransforms();
    % adjust the position to open the door
    grab_pos = tag_pose.position + [0.163; -0.2; 0.02] ;
    grab_ori = tag_pose.quaternion;
    robotArm.cartesian_pose_client(grab_pos, -grab_ori, 0);
    pause(0.5)
    % adjust the y direction
    robotArm.cartesian_pose_client([0; 0.11; 0], -grab_ori, 1);
    
    %% open the microwave door
    result = robotArm.getOri()';
    fix_pos = robotArm.getPos() + [0; 0.1; 0];
    robotArm.closeFinger([5500.0; 5500.0; 5500.0])
%     pause(0.5)
%     R = axang2rotm([0 -1 0 1*pi/9]);
%     result = R * quat2rotm(robotArm.getOri()');
%     robotArm.cartesian_pose_client([0; -0.055; 0], rotm2quat(result)', 1);

    for j = 1:9
        if j > 9
            R = axang2rotm([0 -1 0 0.*pi/9]);
        else
            R = axang2rotm([0 -1 0 0.5*pi/9]);
        end
        result = R * quat2rotm(robotArm.getOri()');
        position_temp = fix_pos + [-r*(1-cos(deltaTheta*(j))); -r*(sin(deltaTheta*(j) ));  0];
        position_temp = position_temp + [-0.1*sin(deltaTheta*j); -0.1*cos(deltaTheta*(j)); 0];
        robotArm.cartesian_pose_client(position_temp, rotm2quat(result)', 0);
    end
    % frac = robotArm.cartesianPathTraj(points);
    % robotArm.execute(1);
    % robotArm.removeAttachedObject('j2n6s300_end_effector', '1');
    % robotArm.poseTargetTraj(init_pos+[0.1; 0; 0], init_ori);
    % obotArm.execute(0);
    %robotArm.cartesian_pose_client([-0.15; -0; 0.0], robotArm.getOri(), 1);
    % robotArm.cartesian_pose_client([0.1708; 0.3253; -0.0609], robotArm.getOri(), 0);
    
    robotArm.closeFinger([0.0; 0.0; 0.0])
    p1 = robotArm.getPos();
    ori1 = robotArm.getOri();

    % move away from fridge door
    robotArm.moveBase2([-0.15; -0.])
    pause(4)
    robotArm.moveBase2([-0.000; 0])
    pause(1)
    robotArm.cartesian_pose_client([0.; -0; 0.35], robotArm.getOri(), 1);
    
    % move the base back 
    robotArm.moveBase2([0.15; -0.])
    pause(12)
    robotArm.moveBase2([-0.000; 0])
    % move back to the front of the fridge
    robotArm.cartesian_pose_client(init_pos+[-0.2; 0.;0], init_ori, 0);

end

