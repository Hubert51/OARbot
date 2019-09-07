function [img] = get_img_from_rr()

camera = RobotRaconteur.Connect('tcp://192.168.1.117:2345/KinovaCameraServer/Camera');
rawimg = camera.getImg();
r = reshape(rawimg.data(1:3:end), rawimg.width, rawimg.height );
g = reshape(rawimg.data(2:3:end), rawimg.width, rawimg.height );
b = reshape(rawimg.data(3:3:end), rawimg.width, rawimg.height );
img = cat(3, r,g,b);
img = imrotate(img, 270);
img = flipdim(img, 2);           %# horizontal flip
