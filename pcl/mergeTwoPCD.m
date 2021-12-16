function xyzMerge = mergeTwoPCD(xyzA,xyzB)

% Merge point clouds
gridStep = 0.1;
pcdA = pointCloud(xyzA);
pcdB = pointCloud(xyzB);
pcdMerge = pcmerge(pcdA,pcdB,gridStep);


% Remove overlapping points @ToDo
% pcdMerge = delteOverlapPCD(pcdMerge);

% Delete outer discrete points @ToDo
% pcdMerge = delteOutDiscretePCD(pcdMerge);

% sample base on PCD density
gridStepDelete = 0.5; 
pcdMerge = pcdownsample(pcdMerge,'gridAverage',gridStepDelete);

% pcd2xyz
xyzMerge = pcdMerge.Location;

end

