function [xyzMovingRegisted,rmse,isRegsiteErr] = registePCD(xyzMoving,xyzFixed) 
%-------------------------------Seting-------------------------------------
% ��������,@ Todo �������Ϳ��Ż�
% parameter in PPF
% Minimum sample grid size
girdStepPPF = 1;
% Large rmse ������Rmse���ڸ�ֵʱ��ICP����������ҪPPF  %3
bigRmse = 3;

% parameter in icp
% Minimum sample grid size
girdStepICP = 1; 
% Maximum number of iterations
maxIterations = 50;
%  Percentage of inliers ����������ȡ�̶������ĵ�ԣ���ȥ����Զ�ĵ��;
inlierRatioMinMax = [0.05,0.5];
inlierRatioStep = 0.05;
% Maximum acceptable error
acceptableMaxRmse = 0.15;

% icp+ppf loop times
% ��������������
numAdjustMax = 5;
% ���PPFִ�д���
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
    % ƥ�����:1.PPFִ�д������ࣻ2.���������������ࣻ2.ICPƥ��ʱ����rmse���������������"
    isRegsiteErr = (numInPPF >= numInPPFMax) || (numAdjust >= numAdjustMax) || (rmse > rmseLast && ~isBigRmse);  
    % ����ͨ����̬�������������д�������
    isNotGoodInlierRatioMax = (isRegsiteErr && ~isBigRmse);
    isMinInlierRatio = round(inlierRatio*100) == round(inlierRatioMinMax(1)*100);
    isEndRegsite = (rmse <= acceptableMaxRmse || abs(rmseLast-rmse)<0.01);

    if isNotGoodInlierRatioMax
        disp('ƥ�����,����inlierRatinMax,������icp');
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