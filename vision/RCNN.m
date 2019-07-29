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

load('rcnn_model.mat')
camera = RobotRaconteur.Connect('tcp://localhost:2345/KinovaCameraServer/Camera');
robotArm = RobotRaconteur.Connect('tcp://localhost:4567/KinovaJointServer/Kinova');

% img = imread('train_data/output00001.png');
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
obj_width = 8;
obj_height = 8;
offset = [4, 0];
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

    [bbox, score, label] = detect(rcnn, img, 'MiniBatchSize', 32);
    [score, idx] = max(score);

    bbox = bbox(idx, :);
    disp(bbox)
    pix_len = (obj_width/bbox(3) + obj_height/bbox(4)) / 2;
    desired = [640, 480] / 2 - [bbox(3), bbox(4)]/2;
    % adjust same direction with error.
    error = ( (desired-[bbox(1), bbox(2)])*pix_len + offset ) / 100;
    pos = robotArm.getPos();
    pos(2) = pos(2) + error(1);
    robotArm.cartesian_pose_client(pos, hardcode_ori);
    
    a = camera.getDepth(bbox') / 1000;
    pos = robotArm.getPos() + [a-0.022; 0; 0.1];
    robotArm.cartesian_pose_client(pos, hardcode_ori);


    annotation = sprintf('%s: (Confidence = %f)', label(idx), score);

    detectedImg = insertObjectAnnotation(img, 'rectangle', bbox, annotation);
%     s1 = 'demo_data/';
%     s = strcat(s1,s2, s3);
%     imwrite(detectedImg, s)
%     count = count + 1;
%     save('count.mat', 'count');


    
    imshow(detectedImg)
    drawnow

    pause(0.1)
end
