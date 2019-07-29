imgSub = rossubscriber('/camera/depth/image_rect_raw');
imgMsg = receive(imgSub);
img = readImage(imgMsg);

imshow(img);

