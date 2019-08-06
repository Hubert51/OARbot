function [bbox] = objDetection(model, iteration)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
    count = 0;
    final_score = 0;
    bbox = [0 0 0 0];
    for i = 1:iteration
        imgSub = rossubscriber('/camera/color/image_raw');
        imgMsg = receive(imgSub);
        img = readImage(imgMsg);
        [bboxes, scores, label] = detect(model, img, 'MiniBatchSize', 32);
        %% food:
    %     index = label=='food';
    %     mask=bboxes > [0 0 50 50];
    %     mask = prod(mask, 2);
    %     [score, idx] = max(mask .* scores .* index);
    %     bbox = bboxes(idx, : );
    %     annotation = sprintf('%s: (Confidence = %f)', label(idx), score);
    %     img = insertObjectAnnotation(img, 'rectangle', bbox, annotation);

        %% water
        % index = label=='water';
        mask=bboxes > [0 0 80 80];
        mask = prod(mask, 2);
        [score, idx] = max(mask .* scores);


        if score >0.5
            annotation = sprintf('%s: (Confidence = %f)', label(idx), score);
            final_score = final_score + score;
            img = insertObjectAnnotation(img, 'rectangle', bboxes(idx,:), annotation);
            bboxes(idx,:)
            drawnow
            imshow(img)
            count = count + 1;
            bbox = bbox + bboxes(idx, : );
%           annotation = sprintf('%s: (Confidence = %f)', label(idx), score);
%         img = insertObjectAnnotation(img, 'rectangle', bbox, annotation);
% 
         % imshow(img)
        end
    end
    if count >= iteration/3
        bbox = bbox / count;
    else
        bbox = [0 0 0 0];
    end
    annotation = sprintf('%s: (Confidence = %f)', label(idx), final_score/count);
    img = insertObjectAnnotation(img, 'rectangle', bbox, annotation);
    
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