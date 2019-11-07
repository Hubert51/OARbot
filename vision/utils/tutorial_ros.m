imgSub = rossubscriber('/camera/color/image_raw');
while 1
    imgMsg = receive(imgSub);

    img = readImage(imgMsg);
    imshow(img,[]);
    % guidata(hObject, handles);
end