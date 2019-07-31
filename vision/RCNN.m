% %%
% % videoLabeler('output2.mp4')
% 
% % training
% load('train_data2.mat')
% foodGroundTruth = selectLabels(gTruth,'food');
% %box2GroundTruth = selectLabels(gTruth,'Box2');
% 
% % save the images into the files.
% % the trainingdata2 stores the file path and the object location
% % create a folder here
% cd train_data2
% trainingFood = objectDetectorTrainingData(foodGroundTruth);
% save('summary.mat', trainingFood)
% % load('summary.mat')
% 
% load('rcnnStopSigns.mat', 'stopSigns', 'layers')
% 
% % imDir = fullfile(matlabroot, 'toolbox', 'vision', 'visiondata','stopSignImages');
% % addpath(imDir);
% 
% layers = [ ...
%     imageInputLayer([32 32 3])
%     convolution2dLayer([5 5],10)
%     reluLayer
%     fullyConnectedLayer(2)
%     softmaxLayer
%     classificationLayer];
% 
% options = trainingOptions('sgdm', ...
%   'MiniBatchSize', 32, ...
%   'InitialLearnRate', 1e-6, ...
%   'MaxEpochs', 10);
% 
% rcnn = trainRCNNObjectDetector(trainingFood, layers, options, 'NegativeOverlapRange', [0 0.3]);

load('rcnn_model_water.mat')
rcnn_water = rcnn;
load('rcnn_model.mat')
rcnn_food = rcnn;
load('rcnn_food_water.mat')
rcnn_food_water;
camera = RobotRaconteur.Connect('tcp://localhost:2345/KinovaCameraServer/Camera');
robotArm = RobotRaconteur.Connect('tcp://localhost:4567/KinovaJointServer/Kinova');

% img = imread('train_data/output00001.png');/
% % img = imread('../test_data/test6.png');
% 
% [bbox, score, label] = detect(rcnn, img, 'MiniBatchSize', 32);
% [score, idx] = max(score);
% 
% bbox = bbox(idx, :);
% annotation = sprintf('%s: (Confidence = %f)', label(idx), score);
% 
% detectedImg = insertObjectAnnotation(img, 'rectangle', bbox, annotation);
% 
% figure
% imshow(detectedImg)

% mode1: resolution is  960 * 600


%%
% Testing

% rosshutdown
% rosinit('011303P0004.local', 'NodeHost', '192.161.90.152')

% load('count.mat');

% define constant
% object dimension
obj_width = 7;
obj_height = 10;
offset = [-0, 0];
hardcode_ori = [-0.5141; -0.5357; -0.4807; -0.4666];

while 1
    imgSub = rossubscriber('/camera/color/image_raw');
    imgMsg = receive(imgSub);
    img = readImage(imgMsg);
    % imshow(img);
%     s1 = 'train_data2_baxter_camera/';
%     s2 = int2str(count);
%     s3 = '.png';
%     s = strcat(s1,s2, s3);
%     imwrite(img, s)
%     [bboxes, scores, label] = detect(rcnn_food_water, img, 'MiniBatchSize', 32);
%     for i = 1:size(scores)
%         annotation = sprintf('%s: (Confidence = %f)', label(i), scores(i));
%         img = insertObjectAnnotation(img, 'rectangle', bboxes(i,:), annotation);
%     end 
%     imshow(img)

    detectedImg = objDetection(rcnn_water, img, ['food', 'water']);
    imshow(detectedImg)

%     detectedImg = objDetection(rcnn_food, detectedImg);
%     imshow(detectedImg)
%     prompt = 'Do you agree this detection ';
    str = input(prompt,'s');
    if isempty(str)
        a = camera.getDepth(bbox') / 1000;
        break
    end
    % drawnow
    pause(0.1)
    
%     s1 = 'demo_data/';
%     s = strcat(s1,s2, s3);
%     imwrite(detectedImg, s)
%     count = count + 1;
%     save('count.mat', 'count');
end

% move the arm
% move arm in y z direction
pix_len = (obj_width/bbox(3) + obj_height/bbox(4)) / 2;
desired = [640, 480] / 2 - [bbox(3), bbox(4)]/2;
% adjust same direction with error.
error = ( (desired-[bbox(1), bbox(2)])*pix_len + offset ) / 100;
a = camera.getDepth(bbox') / 1000;

pos = robotArm.getPos();
pos(2) = pos(2) + error(1);
depth = sqrt(a^2 - error(1)^2)
% robotArm.cartesian_pose_client(pos, hardcode_ori);
pause(1.5)
% move arm in x direction
pos = robotArm.getPos() + [depth-0.005; 0; 0.0];
robotArm.cartesian_pose_client(pos, hardcode_ori);

pause(1.5)
robotArm.closeFinger([3600.0; 3600.0; 3600.0])
pause(1.5)

pos = robotArm.getPos() + [0; 0; 0.2];
robotArm.cartesian_pose_client(pos, hardcode_ori);

