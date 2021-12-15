function pcdMovingRegisted = PPFRegiste(pcdMoving,pcdFixed,girdStep)

% % single to double
% pcdMoving = pointCloud(double(pcdMoving.Location));
% pcdFixed= pointCloud(double(pcdFixed.Location));

% PPF input pcd, output pcd
% Sample
pcdMovingSampled = pcdownsample(pcdMoving,'gridAverage',girdStep);
pcdFixedSampled = pcdownsample(pcdFixed,'gridAverage',girdStep);

% pcd2mesh
meshMovingSampled = pcd2mesh(pcdMovingSampled);
meshFixedSampled = pcd2mesh(pcdFixedSampled);

% initialize detector
dt = PPF3DDetector(0.05,-1,30);
dt = dt.trainModel(meshMovingSampled);
[result, ~, ~] = dt.match(meshFixedSampled,1/5,true,false,true,true, -1, -1);

% Adjust the pose of the model
xyzMovingPPF = TransformPose(pcdMoving.Location,result{1}.pose);

% xyz2pcd
% pcdMovingRegisted = pointCloud(single(xyzMovingPPF));
pcdMovingRegisted = pointCloud(xyzMovingPPF);

end