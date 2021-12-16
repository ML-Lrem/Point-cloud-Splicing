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
        step = 0;
        xyzNowScene =  getPointCloud(filePaths{n});
    end
    xyzNextScene = getPointCloud(filePaths{n+1});
    
    isBigStep = true;
    while 1 % find a best girdStep
        % PPF + ICP (Match the current scene to the next scene)
        [xyzNowSceneRegisted,isRegsiteErr] = registePCD(xyzNowScene,xyzNextScene,step);

        % Merge point cloud (Delete coincident point clouds)
        if ~isRegsiteErr
            % @ToDo 考虑将合并操作在拼接进程的最后进行
            xyzNowScene = mergeTwoPCD(xyzNextScene,xyzNowSceneRegisted);
            break
%             if isBigStep
%                 step = step-1+0.1;
%                 isBigStep = false;
%             else
%                 break;
%             end
        else
            disp(['Registration failed, discard the -No',num2str(n+1),'- scene']);
            step = step + 1;
%             if isBigStep
%                 step = step + 1;
%             else
%                 step = step + 0.1;
%             end
        end
    end
end

% save pcdSpliced
% xyzOut = double(xyzNowScene);
xyzOut = xyzNowScene;
save 'outXYZ.txt' -ascii xyzOut;
disp('Save done')
toc;
