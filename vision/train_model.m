% load('test_label.mat')
% foodGroundTruth = selectLabels(gTruth,'food');
%box2GroundTruth = selectLabels(gTruth,'Box2');

% save the images into the files.
% the trainingdata2 stores the file path and the object location
% create a folder here
% cd test_label
% trainingFood = objectDetectorTrainingData(gTruth);
% save('summary.mat', 'trainingFood')

% summary is picture location and bound box
load('train_data/3objects/summary.mat')
load('rcnnStopSigns.mat', 'stopSigns', 'layers')

% imDir = fullfile(matlabroot, 'toolbox', 'vision', 'visiondata','stopSignImages');
% addpath(imDir);

layers = [ ...
    imageInputLayer([32 32 3])
    convolution2dLayer([5 5],10)
    reluLayer
    fullyConnectedLayer(4)
    softmaxLayer
    classificationLayer];

options = trainingOptions('sgdm', ...
  'MiniBatchSize', 32, ...
  'InitialLearnRate', 1e-6, ...
  'MaxEpochs', 4, ...
  'ExecutionEnvironment','parallel',...
  'CheckpointPath', '/home/ruijie/kinova_ws/src/vision/check_point');

rcnn_food_water = trainRCNNObjectDetector(trainingFood, layers, options, 'NegativeOverlapRange', [0 0.3]);
save('train_data/3objects/rcnn_3objects.mat', 'rcnn_food_water')
save('model/rcnn_3objects.mat', 'rcnn_food_water')

