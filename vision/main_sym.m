% bbox is [y x dy dx];

camera = RobotRaconteur.Connect('tcp://localhost:2345/KinovaCameraServer/Camera');
robotArm = RobotRaconteur.Connect('tcp://localhost:4567/KinovaJointServer/Kinova');
load('rcnn_fridge_box.mat')
rcnn_model_water_food_in_fridge = rcnn;
% img = imread('test_data.png');
robotArm.closeFinger([0.0; 0.0; 0.0]);

% img = imread('../test_data/test6.png');
init_pos = [-0.0515; -0.4863; 0.1694 ];

init_ori = [    0.6935; -0.0466; -0.0242; 0.7185];
first_grab = [ 0.0004; -0.8686; 0.0735];
second_grab = [ 0.0004; -0.8686; 0.3135];
second_pos = [ -0.0297; -0.5745; 0.285];
second_ori = init_ori;
robotArm.cartesian_pose_client(init_pos, init_ori, 0);
init = 1;

camera_shift = 0.015;
height_shift = 0.08;

while 1
    robotArm.closeFinger([0.0; 0.0; 0.0])

    imgSub = rossubscriber('/camera/color/image_raw');
    imgMsg = receive(imgSub);
    img = readImage(imgMsg);

    % [bbox, score, label] = detect(rcnn, img, 'MiniBatchSize', 32);
    % [score, idx] = max(score);

    % bbox = bbox(idx, :);
    bbox = objDetection(rcnn, 8);
    if sum(bbox) == 0
        % in first
        if init == 1
            robotArm.cartesian_pose_client(second_pos, init_ori, 0);
            init = 0;
        else
            robotArm.cartesian_pose_client(init_pos, init_ori, 0);
            init = 1;
        end
    

    else
        if init == 1
            debug_depth = -camera.getDepth(bbox')/1000; 
            trans = objLocalization(rcnn, bbox) + [camera_shift; -camera.getDepth(bbox')/1000+0.004; height_shift];
%             center = bbox(1) + bbox(3)/2 
            robotArm.cartesian_pose_client(trans, init_ori, 1);
            % robotArm.cartesian_pose_client([0; -camera.getDepth(bbox')/1000+0.004; 0], init_ori, 1);
            
            pause(0.5)
            robotArm.closeFinger([6000.0; 6000.0; 6000.0])
            pause(1)
            robotArm.cartesian_pose_client(init_pos, init_ori, 0);
            robotArm.cartesian_pose_client(second_pos, init_ori, 0);
            robotArm.cartesian_pose_client(second_grab+[0;0;0.05], init_ori, 0);
            robotArm.closeFinger([0.0; 0.0; 0.0])
            pause(1)
            robotArm.closeFinger([0.0; 0.0; 0.0])
            pause(1)
            robotArm.cartesian_pose_client(second_pos, init_ori, 0);
            robotArm.cartesian_pose_client(init_pos, init_ori, 0);
        elseif init == 0
            robotArm.cartesian_pose_client(second_grab, init_ori, 0);
            robotArm.closeFinger([6000.0; 6000.0; 6000.0])
            pause(1)
            robotArm.cartesian_pose_client(second_pos, init_ori, 0);
            robotArm.cartesian_pose_client(init_pos, init_ori, 0);
            robotArm.cartesian_pose_client(first_grab+[0;0;0.05], init_ori, 0);
            robotArm.closeFinger([0.0; 0.0; 0.0])
            pause(1)
            robotArm.closeFinger([0.0; 0.0; 0.0])
            pause(1)
            robotArm.cartesian_pose_client(init_pos, init_ori, 0);
            robotArm.cartesian_pose_client(second_pos, init_ori, 0);
        end
            

% 
%         annotation = sprintf('%s: (Confidence = %f)', label(idx), score);
% 
%         detectedImg = insertObjectAnnotation(img, 'rectangle', bbox, annotation);
% 
%         drawnow
% 
%         imshow(detectedImg)
%         pause(0.1)
    end
end

robotArm.closeFinger([0.0; 0.0; 0.0])

deltaTheta = 10/180*pi ;
r = 0.305; % The radius of the fridge door

init_pos = [0.0885479301214; -0.545893788338; 0.13304695487];
init_ori = [0.716; -0.029; 0.016; 0.697];
fridge_pos = [0.1599; -0.6973; 0.1732];
fridge_ori = [0.5160; 0.4518; -0.5073; 0.5218];

init2gripper = [0.085; -0.165; 0.05];


robotArm.cartesian_pose_client(robotArm.getPos(), init_ori, 0);

pause(2)
% robotArm.cartesian_pose_client([-0.093; 0.0; 0], init_ori, 1);

%% approach the door
R = axang2rotm([1 0 0 0.5*pi]);
result = R * quat2rotm(init_ori');
robotArm.cartesian_pose_client(init_pos, rotm2quat(result)', 0);

R = axang2rotm([0 1 0 -0.5*pi/9]);
result = R * quat2rotm(robotArm.getOri()');
% robotArm.cartesian_pose_client(init_pos, rotm2quat(result)', 0);

robotArm.cartesian_pose_client(init_pos+[0.150; -0.135; 0.055], rotm2quat(result)', 0);

robotArm.closeFinger([6000.0; 6000.0; 6000.0])

%% open the door
fix_pos = robotArm.getPos();
for j = 1:10
    R = axang2rotm([0 0 1 0.5*pi/9]);
    result = R * quat2rotm(robotArm.getOri()');
    position_temp = fix_pos + [-r*(1-cos(deltaTheta*(j))); r*(sin(deltaTheta*(j) ));  0];
    robotArm.cartesian_pose_client(position_temp, rotm2quat(result)', 0);


end
robotArm.closeFinger([0.0; 0.0; 0.0])
pause(1)
robotArm.cartesian_pose_client(move_out_pos1, move_out_ori1, 0);
robotArm.cartesian_pose_client(move_out_pos2, move_out_ori2, 0);
robotArm.cartesian_pose_client(move_out_pos3, move_out_ori3, 0);
robotArm.cartesian_pose_client(move_out_pos4, move_out_ori4, 0);
%robotArm.cartesian_pose_client([-0.1; 0; 0.02], move_out_ori4, 1);

pause(4)

robotArm.cartesian_pose_client(move_out_pos5+[0;0;0.03], move_out_ori5, 0);
pause(0.5)
robotArm.closeFinger([6000.0; 6000.0; 6000.0])
pause(0.5)
robotArm.cartesian_pose_client(move_out_pos4, move_out_ori4, 0);
robotArm.cartesian_pose_client(move_out_pos6, move_out_ori6, 0);
pause(2)

robotArm.closeFinger([0.0; 0.0; 0.0])




%% 0.09 -0.02

