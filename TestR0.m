function Er0Log = TestR0(r0)

% add function path
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

Er0Log = [];

% full scene splicing
for n = 1:numPCFile-1

    
    xyzNowScene =  getPointCloud(filePaths{n});
    xyzNextScene = getPointCloud(filePaths{n+1});
    
    [~,E100] = AtestRegiste(xyzNowScene,xyzNextScene,r0);
    disp(['E100' num2str(n) ' and '  num2str(n+1),'  E100:',num2str(E100)])  
end

end

