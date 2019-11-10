function [result] = post_recognition(result)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
%   this function tries to combine the bounding box if the bounding box is
%   overlapping. And then change the name of the object to unknown. Let the
%   uesr to recognize the object in next step.


i = 1;
while i <= length(result)
    j= i + 1;
    while j <= length(result)
        A = result(i).bbox;
        B = result(j).bbox;
        intersection = rectint(A, B);
        if intersection > 0.3*min(A(3)*A(4), B(3)*B(4))
            new_rect = [ 
                min(A(1), B(1)) 
                min(A(2), B(2))
                max(A(1)+A(3), B(1)+B(3)) - min(A(1), B(1)) 
                max(A(2)+A(4), B(2)+B(4)) - min(A(2), B(2)) 
            ];
            new_rect = new_rect';
            result(i).bbox = new_rect;
            result(i).name = 'Unknown';
            result(j) = [];
            j = i + 1;
            continue
            % result(i) = []
        end
        j = j + 1;
    end
    i = i + 1;
end



