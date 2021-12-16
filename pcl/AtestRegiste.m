function [isRegsiteErr,E100] = AtestRegiste(xyzMoving,xyzFixed,r0) 
% Parameter setting
% parameter in PPF
% Minimum sample grid size
girdStepPPF = 1;
% Large rmse When RMSE is greater than this value，ICP treatment
bigRmse = 3;

% parameter in icp
% Minimum sample grid size 需要动态调整 ToDo
girdStepICP = 1; 
% Maximum number of iterations
maxIterations = 100;
% Percentage of inliers
% Take fixed proportion of point pairs according to distance to remove distant point pairs;
% 根据距离取固定比例的点对
inlierRatioMax = 1;
% 递减步长决定了匹配成功或失败的评价参考线
inlierRatioStep = 0.1;

% loop times
% Maximum PPF execution times
numAdjusteInlierRatioMax = 3;
% Maximum acceptable error
acceptableMaxRmse = 0.2;


% xyz2pcd
pcdMoving = pointCloud(xyzMoving);
pcdFixed = pointCloud(xyzFixed);
pcdTemp = pcdMoving;

% 根据两个点云特性计算大致的girdStepICP 估计
% girdStepICP = girdStepICP - Step;
% estimateGirdStepICP(xyzMoving,xyzFixed,girdStepICP);

inlierRatio = inlierRatioMax;
numAdjusteInlierRatio = 0;
rmseLog = [];
isRmseLimit = false;
isFirst = true;
isMinInlierRatio = false;
isAdjusted = false;
isRegsiteErr = false;
% % PPF first,ICP after
while ~isRmseLimit
    if isFirst
        % 全配准过程PPF只执行一次
        disp('PPF+ICP...');
        isFirst=false;
        pcdTemp =  PPFRegiste(pcdTemp,pcdFixed,girdStepPPF);
        pcdTempPPF = pcdTemp;
    end
    
    % ICP input pcd, output pcd
    [pcdTemp,rmse] = ICPRegister(pcdTemp,pcdFixed,girdStepICP,maxIterations,inlierRatio);
    if isAdjusted
        Er0 = rmse;
        break;
    end
    rmseLog(end+1,:) = [rmse,inlierRatio];
    isBigRmse = rmse > bigRmse;    
    
    % Registration status flag
    % Registration error:
        % 1. Too many PPF executions
        % 2. Too many parameter adjustments
        % 3. During ICP matching, "RMSE" does not decrease but increases;
    isEndRegsite = (rmse <= acceptableMaxRmse || isMinInlierRatio);
    isRegsiteErr = (numAdjusteInlierRatio >= numAdjusteInlierRatioMax) && isBigRmse; 
    if size(rmseLog,1)==3
        isRegsiteErr = judgeRegsiteErr(rmseLog);
    end
    
    if isEndRegsite || isRegsiteErr
       isRmseLimit = true;
    else
        % icp内部循环微调:目的在于修正小误差,并且根据迭代规律可以判断是否匹配成功(所需时间应该有明显的递减趋势)
        if ~isMinInlierRatio && ~isBigRmse
            E100 = rmse;
            break;
            Er0 = 0;
            break;
%             inlierRatio = inlierRatio - inlierRatioStep;
        % PPF第一次大幅调整
        % 如果在经过一次PPF和ICP后还是得到大误差，那么很可能该组点云存在很多距离特别大的点对
        % 因此需要剔除更多这样的点: @@一个可能的改进方案是：剔除距离出现跳变的点对
        % 这里30的取值与两个点云的某些信息有关：比如点云数量、一次PPF+ICP之后rmse，这里存在一个可研究、可估计的函数运算
        % inlierRatio的计算函数的研究是PPF+ICP结合的重点,如果能在ICP前置操作中得到一个合适的inlierRatio，那么将大幅提高计算速度
        % 但是这就意味着需要获取新的参数以替代rmse的指导作用
        elseif ~isAdjusted && isBigRmse
             E100 = rmse;
             break
            rmseLog = [];
            pcdTemp = pcdTempPPF;
            % 测试r0
            inlierRatio = inlierRatio - rmse*r0;
            isAdjusted = true;
            numAdjusteInlierRatio = numAdjusteInlierRatio+1;
            disp(['inlierRatio第一次大幅调整:',num2str(inlierRatio)])
        % PPF内部循环微调
        elseif isAdjusted && isBigRmse
            rmseLog = [];
            numAdjusteInlierRatio = numAdjusteInlierRatio+1;
            pcdTemp = pcdTempPPF;
            inlierRatio = inlierRatio - inlierRatioStep;
            disp(['inlierRatio微调:',num2str(inlierRatio)])
        end
        isMinInlierRatio = round(inlierRatio*100)<=round(inlierRatioStep*100);
    end
end

end