function [pcdMovingRegisted,rmse] = ICPRegister(pcdMoving,pcdFixed,gridStep,maxIterations)

rmseLast = 0;
isRmseLimit = false;
% sample
pcdMovingSample = pcdownsample(pcdMoving,'gridAverage',gridStep);
pcdFixedSample = pcdownsample(pcdFixed,'gridAverage',gridStep);

while ~isRmseLimit
    % Apply the rigid registration
    [tform,~,rmse] = pcregistericp(pcdMovingSample, pcdFixedSample, 'Extrapolate', true,'MaxIterations',maxIterations);
    
    % transform
    pcdMovingSample = pctransform(pcdMovingSample, tform);
   
    % loop ICP    
    if abs(rmse-rmseLast)<0.01
        isRmseLimit = true;
    else
        rmseLast = rmse;
    end
end

pcdMovingRegisted = pcdMoving;

end

