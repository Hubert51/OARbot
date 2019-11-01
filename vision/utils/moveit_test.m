robotArm = RobotRaconteur.Connect('tcp://129.161.90.73:4567/KinovaJointServer/Kinova');

points = {};
points{1}.pos = robotArm.getPos();
points{1}.ori = robotArm.getOri();

% robotArm.cartesianPathTraj(points);
% robotArm.execute(10);

%% test the scene
pos = robotArm.getPos();
ori = robotArm.getOri();

% addBox(self, name, dim, pos)

robotArm.addBox('box1', [0.15; 0.15; 0.15], pos + [0; 0.3; 0])