clear
clc
tic

% data type: XYZpoint

% add sub function path
addpath(genpath('.\ppf'));
addpath(genpath('.\mex'));
addpath(genpath('.\pcl'));
addpath(genpath('.\icp'));

% load dataPath
pcType = 'txt';
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

    % PPF + ICP (Match the current scene to the next scene) @待办事项
    [xyzNowSceneRegisted,rmse] = registePCD(xyzNowScene,xyzNextScene);
    
    % compute the splicing accuracy and evaluate the splicing quality @待办事项
    [accuracy,isQualified]=computeAccuracy(xyzNowSceneRegisted,xyzNextScene);
    accuracyLog(n,:) = [accuracy,double(isQualified)];
    
    % Merge point cloud (Delete coincident point clouds)
    if isQualified
        xyzNowScene = mergePointCloud(xyzNextScene,xyzNowSceneRegisted);
    end
end

% save pcdSpliced
xyzOut = xyzNowScene;
save 'outXYZ.txt' -ascii xyzOut;
disp('Save done')
disp(['accuracy:',num2str(rmse)])
toc;
