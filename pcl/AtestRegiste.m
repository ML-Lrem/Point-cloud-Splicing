function [isRegsiteErr,E100] = AtestRegiste(xyzMoving,xyzFixed,r0) 
% Parameter setting
% parameter in PPF
% Minimum sample grid size
girdStepPPF = 1;
% Large rmse When RMSE is greater than this value��ICP treatment
bigRmse = 3;

% parameter in icp
% Minimum sample grid size ��Ҫ��̬���� ToDo
girdStepICP = 1; 
% Maximum number of iterations
maxIterations = 100;
% Percentage of inliers
% Take fixed proportion of point pairs according to distance to remove distant point pairs;
% ���ݾ���ȡ�̶������ĵ��
inlierRatioMax = 1;
% �ݼ�����������ƥ��ɹ���ʧ�ܵ����۲ο���
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

% ���������������Լ�����µ�girdStepICP ����
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
        % ȫ��׼����PPFִֻ��һ��
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
        % icp�ڲ�ѭ��΢��:Ŀ����������С���,���Ҹ��ݵ������ɿ����ж��Ƿ�ƥ��ɹ�(����ʱ��Ӧ�������Եĵݼ�����)
        if ~isMinInlierRatio && ~isBigRmse
            E100 = rmse;
            break;
            Er0 = 0;
            break;
%             inlierRatio = inlierRatio - inlierRatioStep;
        % PPF��һ�δ������
        % ����ھ���һ��PPF��ICP���ǵõ�������ô�ܿ��ܸ�����ƴ��ںܶ�����ر��ĵ��
        % �����Ҫ�޳����������ĵ�: @@һ�����ܵĸĽ������ǣ��޳������������ĵ��
        % ����30��ȡֵ���������Ƶ�ĳЩ��Ϣ�йأ��������������һ��PPF+ICP֮��rmse���������һ�����о����ɹ��Ƶĺ�������
        % inlierRatio�ļ��㺯�����о���PPF+ICP��ϵ��ص�,�������ICPǰ�ò����еõ�һ�����ʵ�inlierRatio����ô�������߼����ٶ�
        % ���������ζ����Ҫ��ȡ�µĲ��������rmse��ָ������
        elseif ~isAdjusted && isBigRmse
             E100 = rmse;
             break
            rmseLog = [];
            pcdTemp = pcdTempPPF;
            % ����r0
            inlierRatio = inlierRatio - rmse*r0;
            isAdjusted = true;
            numAdjusteInlierRatio = numAdjusteInlierRatio+1;
            disp(['inlierRatio��һ�δ������:',num2str(inlierRatio)])
        % PPF�ڲ�ѭ��΢��
        elseif isAdjusted && isBigRmse
            rmseLog = [];
            numAdjusteInlierRatio = numAdjusteInlierRatio+1;
            pcdTemp = pcdTempPPF;
            inlierRatio = inlierRatio - inlierRatioStep;
            disp(['inlierRatio΢��:',num2str(inlierRatio)])
        end
        isMinInlierRatio = round(inlierRatio*100)<=round(inlierRatioStep*100);
    end
end

end