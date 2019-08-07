function [result] = objDetection(model, objects, iteration)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
    count = 0;
    final_score = 0;
    bbox = [0 0 0 0];
    result = [];
    for j=1:length(objects) 
        object.name = objects(j);
        object.bbox = [0 0 0 0];
        object.count = 0;
        result = [result, object];
    end
    for i = 1:iteration
        imgSub = rossubscriber('/camera/color/image_raw');
        imgMsg = receive(imgSub);
        img = readImage(imgMsg);
        [bboxes, scores, label] = detect(model, img, 'MiniBatchSize', 32);

        %% water
        for j=1:length(objects) 
            index = label==objects(j);
            mask=bboxes > [0 0 80 80];
            mask = prod(mask, 2);
            [score, idx] = max(mask .* scores .* index);


            if score >0.5
%                 annotation = sprintf('%s: (Confidence = %f)', label(idx), score);
%                 final_score = final_score + score;
%                 img = insertObjectAnnotation(img, 'rectangle', bboxes(idx,:), annotation);
%                 bboxes(idx,:)
%                 drawnow
%                 imshow(img)
                result(j).count = result(j).count + 1;
                result(j).bbox = result(j).bbox + bboxes(idx, : );
    %           annotation = sprintf('%s: (Confidence = %f)', label(idx), score);
    %         img = insertObjectAnnotation(img, 'rectangle', bbox, annotation);
    % 
             % imshow(img)
            end
        end
    end
    
    for j = 1:length(result)
        if result(j).count >= iteration/3
            result(j).bbox = result(j).bbox / result(j).count;
        else
            result(j).bbox = [0 0 0 0];
        end
    end
    
    for j = 1:length(result)
        annotation = sprintf('%s: (Confidence = %f)', result(j).name, 1.00);
        img = insertObjectAnnotation(img, 'rectangle', result(j).bbox, annotation);
    end

    imshow(img)
    drawnow

end

%     while ~(bbox(3) > 50 && bbox(4) > 50)
%         scores(idx) = [];
%         bboxes(idx,:) = [];
%         [score, idx] = max(scores);
%         bbox = bboxes(idx, : );
%     end
    % show model output