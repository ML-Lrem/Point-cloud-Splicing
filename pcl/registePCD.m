function [xyzMovingRegisted,rmse] = registePCD(xyzMoving,xyzFixed) 
%-------------------------------Seting-------------------------------------
% 参数设置,@ Todo 数据类型可优化
% parameter in PPF
% Minimum sample grid size
girdStepPPF = 1;
% Large rmse 表征当Rmse大于该值时，ICP无力处理，需要PPF
bigRmse = 3;

% parameter in icp
% Minimum sample grid size
girdStepICP = 1; 
% Maximum number of iterations
maxIterations = 50;
%  Percentage of inliers 按距离排序取固定比例的点对，以去除较远的点对;
inlierRatioMinMax = [0.05,0.5];
inlierRatioStep = 0.05;
% Maximum acceptable error
acceptableMaxRmse = 0.15;

% xyz2pcd
pcdMoving = pointCloud(xyzMoving);
pcdFixed = pointCloud(xyzFixed);
pcdTemp = pcdMoving;

rmseLast = nan;
isRmseLimit = false;
inlierRatio = inlierRatioMinMax(2);
numInAdjust=0;
while ~isRmseLimit
    % ICP input pcd, output pcd
    % disp('ICP...');
    [pcdTemp,rmse] = ICPRegister(pcdTemp,pcdFixed,girdStepICP,maxIterations,inlierRatio);
    isBigRmse = rmse > bigRmse;
    disp(['Regsited. accuarcy:',num2str(rmse),'  inlierRatio:',num2str(inlierRatio)]);
        
    % PPF input pcd, output pcd
    if isBigRmse
        disp('PPF...');
        pcdTemp =  PPFRegiste(pcdTemp,pcdFixed,girdStepPPF);      
    end
    
    % Registration status flag
    isNotGoodInlierRatioMax = (rmseLast-rmse<0 && ~isBigRmse && rmse>0.2);
    isMinInlierRatio = round(inlierRatio*100) == round(inlierRatioMinMax(1)*100);
    isEndRegsite = (rmse <= acceptableMaxRmse || abs(rmseLast-rmse)<0.01);
    
    if isNotGoodInlierRatioMax
        disp('需要调整inlierRatinMax,并重启icp');
        numInAdjust = numInAdjust+1;
        rmseLast = nan;
        inlierRatio = inlierRatioMinMax(2) -numInAdjust*inlierRatioStep;
        pcdTemp = pcdMoving;
    elseif isEndRegsite 
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


