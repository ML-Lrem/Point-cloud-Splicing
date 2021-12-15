function xyzMerge = mergePointCloud(xyzA,xyzB)

gridStep = 0.1; % 体素下采样格网边长
pcdA = pointCloud(xyzA);
pcdB = pointCloud(xyzB);
pcdMerge = pcmerge(pcdA,pcdB,gridStep);

% 从点云中删除冗余点，基于网格采样方法
gridStepDelete = 0.5; 
pcdMerge = pcdownsample(pcdMerge,'gridAverage',gridStepDelete);

% pcd2xyz
xyzMerge = pcdMerge.Location;

end

