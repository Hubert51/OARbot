function [outputArg1,outputArg2] = open_fridge(hObject,inputArg2)
    % open fridge
    handles = guidata(hObject);

    handles.text2.String = 'Open the Fridge door';

    deltaTheta = 10/180*pi ;
    r = 0.305; % The radius of the fridge door
    robotArm = RobotRaconteur.Connect('tcp://192.168.1.117:4567/KinovaJointServer/Kinova');
    peripherals = RobotRaconteur.Connect('tcp://192.168.1.117:1234/KinovaPeripheralsServer/peripherals');
    
    init_pos = [-0.1401; 0.5012; -0.0236];
    init_ori = [0.0026; 0.7361; 0.6761; 0.0310];
    robotArm.cartesian_pose_client(init_pos, init_ori, 0);

    pause(3)
    tag_pose = peripherals.lookUptransforms();
    tag_pos = tag_pose.position + [0; -0.3; -0.05] + [-0.105; 0.255; 0.018];
    tag_ori = tag_pose.quaternion;
    % adjust the ori
    robotArm.cartesian_pose_client(robotArm.getPos(), -tag_ori, 0);
    R = axang2rotm([1 0 0 0.5*pi]);
    result = R * quat2rotm(init_ori');
    R = axang2rotm([0 1 0 -0.5*pi/9]);
    result = R * result;
    robotArm.cartesian_pose_client(robotArm.getPos(), rotm2quat(result)', 0);
    
    % go to the fridge
    robotArm.cartesian_pose_client(tag_pos, rotm2quat(result)', 0);
    R = axang2rotm([0 1 0 0.5*pi/9]);
    result = R * quat2rotm(robotArm.getOri()');
    robotArm.cartesian_pose_client(robotArm.getPos()+[0;0;0.02], rotm2quat(result)', 0);
    
    %% open the door
    result = robotArm.getOri()';
    fix_pos = robotArm.getPos();
    robotArm.closeFinger([6000.0; 6000.0; 6000.0])

    for j = 1:9
        if j > 9
            R = axang2rotm([0 0 1 0.*pi/9]);
        else
            R = axang2rotm([0 0 1 0.5*pi/9]);
        end
        result = R * quat2rotm(robotArm.getOri()');
        position_temp = fix_pos + [r*(1-cos(deltaTheta*(j))); -r*(sin(deltaTheta*(j) ));  0];
        robotArm.cartesian_pose_client(position_temp, rotm2quat(result)', 0);
    end
    % frac = robotArm.cartesianPathTraj(points);
    % robotArm.execute(1);
    % robotArm.removeAttachedObject('j2n6s300_end_effector', '1');
    % robotArm.poseTargetTraj(init_pos+[0.1; 0; 0], init_ori);
    % obotArm.execute(0);
    %robotArm.cartesian_pose_client([-0.15; -0; 0.0], robotArm.getOri(), 1);
    robotArm.cartesian_pose_client([0.1708; 0.3253; -0.0609], robotArm.getOri(), 0);
    
    handles.locations.endOpenFridge = [robotArm.getPos(); robotArm.getOri()];

    robotArm.closeFinger([0.0; 0.0; 0.0])
    pause(1)
    % move away from fridge door
    robotArm.cartesian_pose_client([0.1; -0; 0.0], robotArm.getOri(), 1);
    % move up the kinova arm and rotate the arm
    R = axang2rotm([1 0 0 -0.5*pi]);
    quat = rotm2quat(R * quat2rotm(robotArm.getOri()'))';
    robotArm.cartesian_pose_client([0.; 0.05; 0.40], quat, 1);

    % move back to the front of the fridge
    robotArm.cartesian_pose_client([-0.40; 0.1; 0.], init_ori, 1);
    robotArm.cartesian_pose_client(init_pos+[-0.06; 0.;0], init_ori, 0);
    guidata(hObject, handles);

% 
%     robotArm.moveBase([0.0; -0.15])
%     pause(3)
%     robotArm.moveBase([-0.0005; 0])
end

