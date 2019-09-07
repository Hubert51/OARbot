function [handles] = detect_stage(handles)
%detect_food Summary of this function goes here
%   function to help gui handles stage 2
% handles = guidata(hObject);
objects = ["water", "food", "cola"];

handles.text2.String = 'Detecting the Food';
handles.listbox1.String = [];
detect_result = objDetection(handles.rcnn, objects, 8);

handles.text2.String = 'Detected the food, please choose the food you want';
obj_name = [];
for i=1:length(detect_result)
    if  detect_result(i).bbox ~= [0 0 0 0]
        obj_name = [obj_name; detect_result(i).name];
    end
end
obj_name = ["Re-detect"; obj_name];
handles.listbox1.String = obj_name;
handles.update_flag = 0;
handles.detect_result = detect_result;
handles.cur_img = get_img_from_rr();
handles.listbox1.Value = 1;
end

