function [xyzMovingRegisted,rmse,isRegsiteErr] = registePCD(xyzMoving,xyzFixed) 
%-------------------------------Seting-------------------------------------
% 参数设置,@ Todo 数据类型可优化
% parameter in PPF
% Minimum sample grid size
girdStepPPF = 1;
% Large rmse 表征当Rmse大于该值时，ICP无力处理，需要PPF  %3
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

% icp+ppf loop times
% 最大参数调整次数
numAdjustMax = 5;
% 最大PPF执行次数
numInPPFMax = 5;



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
    % 匹配错误:1.PPF执行次数过多；2.参数调整次数过多；2.ICP匹配时出现rmse”不减反增的情况"
    isRegsiteErr = (numInPPF >= numInPPFMax) || (numAdjust >= numAdjustMax) || (rmse > rmseLast && ~isBigRmse);  
    % 允许通过动态调整参数，进行错误修正
    isNotGoodInlierRatioMax = (isRegsiteErr && ~isBigRmse);
    isMinInlierRatio = round(inlierRatio*100) == round(inlierRatioMinMax(1)*100);
    isEndRegsite = (rmse <= acceptableMaxRmse || abs(rmseLast-rmse)<0.01);

    if isNotGoodInlierRatioMax
        disp('匹配错误,调整inlierRatinMax,并重启icp');
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