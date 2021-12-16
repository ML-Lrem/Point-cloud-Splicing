clear
clc
tic

% data type: XYZpoint
% setting
% Maximum error allowed to splice two scenes
maxAcceptRmse = nan;
% 场景连续丢失次数
numSceneContinueDiscard = 0;
% 最大允许连续丢失次数
numSceneContinueDiscardMax = 3;

% add function path
addpath(genpath('.\ppf'));
addpath(genpath('.\mex'));
addpath(genpath('.\pcl'));
addpath(genpath('.\icp'));

% load dataPath
pcType = 'ply';
path = './data/';
file = dir(strcat(path,'*.',pcType));
fileNames = sortNat({file.name})';
numPCFile = size(fileNames,1);
filePaths = cell(numPCFile,1);
for n = 1:numPCFile
    filePaths{n} = strcat(path,fileNames{n});
end
clear path file fileNames

% record accuracy log
accuracyLog = zeros(numPCFile-1,2);

% full scene splicing
for n = 1:numPCFile-1
    disp(['Splicing:' num2str(n) ' of '  num2str(numPCFile-1)] )  
    isFirst = (n==1);
    if isFirst
        xyzNowScene =  getPointCloud(filePaths{n});
    end
    xyzNextScene = getPointCloud(filePaths{n+1});

    % PPF + ICP (Match the current scene to the next scene)
    [xyzNowSceneRegisted,isRegsiteErr] = registePCD(xyzNowScene,xyzNextScene);
    
    % compute the splicing accuracy and evaluate the splicing quality 
    % @Todo: how to compute the rmse of two pcd which just have a part area be Regstied
    [rmse,isQualified]=computeAccuracy(xyzNowSceneRegisted,xyzNextScene,maxAcceptRmse);
    accuracyLog(n,:) = [rmse,double(isQualified)];

    % Merge point cloud (Delete coincident point clouds)
    if ~isRegsiteErr && isQualified 
        % @ToDo 考虑将合并操作在拼接进程的最后进行
        xyzNowScene = mergeTwoPCD(xyzNextScene,xyzNowSceneRegisted);
        numSceneContinueDiscard = 0;
    elseif isRegsiteErr
        disp(['Registration failed, discard the -No',num2str(n+1),'- scene']);
        numSceneContinueDiscard = numSceneContinueDiscard + 1;
    else        
        disp(['Registration quality is not up to standard, discard the No',num2str(n+1),'scene']);
        numSceneContinueDiscard = numSceneContinueDiscard + 1;
    end
    isWorkingErr = numSceneContinueDiscard >= numSceneContinueDiscardMax;
    if isWorkingErr
        disp('连续丢失了太多场景,拼接失败...')
        break;
    end
end

% save pcdSpliced
% xyzOut = double(xyzNowScene);
xyzOut = xyzNowScene;
save 'outXYZ.txt' -ascii xyzOut;
disp('Save done')
disp(['accuracy:',num2str(rmse)])
toc;
