function [pcdMovingRegisted,rmse] = ICPRegister(pcdMoving,pcdFixed,gridStep,maxIterations,inlierRatio)

% sample
pcdMovingSample = pcdownsample(pcdMoving,'gridAverage',gridStep);
pcdFixedSample = pcdownsample(pcdFixed,'gridAverage',gridStep);

% Apply the rigid registration
[tform,~,rmse] = pcregistericp(pcdMovingSample, pcdFixedSample, 'Extrapolate', true,'MaxIterations',maxIterations,'InlierRatio',inlierRatio);

% transform
pcdMovingRegisted = pctransform(pcdMoving, tform);

end

