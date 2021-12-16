function [xyzMovingRegisted,isRegsiteErr] = registePCD(xyzMoving,xyzFixed) 
% Parameter setting
% parameter in PPF
% Minimum sample grid size
girdStepPPF = 1;
% Large rmse When RMSE is greater than this value£¬ICP treatment
bigRmse = 3;

% parameter in icp
% Minimum sample grid size
girdStepICP = 1; 
% Maximum number of iterations
maxIterations = 50;
% Percentage of inliers 
% Take fixed proportion of point pairs according to distance to remove distant point pairs;
inlierRatioMinMax = [0.05,0.5];
inlierRatioStep = 0.05;
% Maximum acceptable error
acceptableMaxRmse = 0.15;

% loop times
% Maximum adjustment times
numAdjustMax = 2;
% Maximum PPF execution times
numInPPFMax = 3;



% xyz2pcd
pcdMoving = pointCloud(xyzMoving);
pcdFixed = pointCloud(xyzFixed);
pcdTemp = pcdMoving;

rmseLast = nan;
isRmseLimit = false;
inlierRatio = inlierRatioMinMax(2);
numAdjust = 0;
numInPPF = 0;
while ~isRmseLimit
    % ICP input pcd, output pcd
    % disp('ICP...');
    [pcdTemp,rmse] = ICPRegister(pcdTemp,pcdFixed,girdStepICP,maxIterations,inlierRatio);
    isBigRmse = rmse > bigRmse;
    disp(['Regsited. accuarcy:',num2str(rmse),'  inlierRatio:',num2str(inlierRatio)]);
        
    % PPF input pcd, output pcd
    if isBigRmse
        disp('PPF...');
        numInPPF = numInPPF + 1;
        pcdTemp =  PPFRegiste(pcdTemp,pcdFixed,girdStepPPF);      
    end
    
    % Registration status flag
    % Registration error:
        % 1. Too many PPF executions
        % 2. Too many parameter adjustments
        % 3. During ICP matching, "RMSE" does not decrease but increases;
    isRegsiteErr = (numInPPF >= numInPPFMax) || (numAdjust >= numAdjustMax) || (rmse > rmseLast && ~isBigRmse);  
    % Error correction is allowed by dynamically adjusting parameters
    isNotGoodInlierRatioMax = (isRegsiteErr && ~isBigRmse);
    isMinInlierRatio = round(inlierRatio*100) == round(inlierRatioMinMax(1)*100);
    isEndRegsite = (rmse <= acceptableMaxRmse || abs(rmseLast-rmse)<0.01);

    if isNotGoodInlierRatioMax
        disp('Registration error: Adjust inlierRatinMax and retry icp');
        rmseLast = nan;
        numInPPF = 0;
        numAdjust = numAdjust+1;
        inlierRatio = inlierRatioMinMax(2) - numAdjust*inlierRatioStep;
        pcdTemp = pcdMoving;
    elseif isEndRegsite || isRegsiteErr
       isRmseLimit = true;
    else
        if ~isMinInlierRatio
            inlierRatio = inlierRatio - inlierRatioStep;
        end
        rmseLast = rmse;
    end
end

% out xyz
pcdMovingRegisted = pcdTemp;
xyzMovingRegisted = pcdMovingRegisted.Location;

end