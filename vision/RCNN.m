% %%
% % videoLabeler('output2.mp4')
% folder = '3objects';
% % training
% load( strcat('train_data/', folder, '/', folder, '_labels.mat') )
% % %foodGroundTruth = selectLabels(gTruth,'food');
% % %box2GroundTruth = selectLabels(gTruth,'Box2');
% % 
% % % save the images into the files.
% % % the trainingdata2 stores the file path and the object location
% % % create a folder here
% cd( strcat('train_data/', folder, '/img') )
% trainingFood = objectDetectorTrainingData(gTruth);
% save('../summary.mat', 'trainingFood')
% load('summary.mat')

load('rcnnStopSigns.mat', 'stopSigns', 'layers')
trainingFood = removevars(trainingFood,{'cola'});
% imDir = fullfile(matlabroot, 'toolbox', 'vision', 'visiondata','stopSignImages');
% addpath(imDir);
load('../../water_in_box/rcnn_water_in_box.mat')
layers = rcnn.Network.Layers;
% layers = [ ...
%     imageInputLayer([32 32 3])
%     convolution2dLayer([5 5],10)
%     reluLayer
%     fullyConnectedLayer(2)
%     softmaxLayer
%     classificationLayer];

options = trainingOptions('sgdm', ...
  'MiniBatchSize', 32, ...
  'InitialLearnRate', 1e-6, ...
  'MaxEpochs', 10);

rcnn = trainRCNNObjectDetector(trainingFood, layers, options, 'NegativeOverlapRange', [0 0.3]);
cd('..')
save('rcnn_water_food_box.mat', 'rcnn')


%% 
% rcnn_water_in_box = rcnn_model_water_food_in_fridge + water_in_box
% rcnn_water_food_box = rcnn_water_in_box + 3objects(do not use cola now)



