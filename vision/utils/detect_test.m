function [] = detect_test()
%detect_food Summary of this function goes here
%   function to help gui handles stage 2
% handles = guidata(hObject);
objects = ["water", "food", "cola"];
load('model/rcnn_3objects.mat')
rcnn = rcnn_food_water;

detect_result = objDetection(rcnn, objects, 8);

obj_name = [];
for i=1:length(detect_result)
    if  detect_result(i).bbox ~= [0 0 0 0]
        obj_name = [obj_name; detect_result(i).name];
    end
end


end
