function xyzMerge = mergePointCloud(xyzA,xyzB)

gridStep = 0.1; % �����²��������߳�
pcdA = pointCloud(xyzA);
pcdB = pointCloud(xyzB);
pcdMerge = pcmerge(pcdA,pcdB,gridStep);

% �ӵ�����ɾ������㣬���������������
gridStepDelete = 0.5; 
pcdMerge = pcdownsample(pcdMerge,'gridAverage',gridStepDelete);

% pcd2xyz
xyzMerge = pcdMerge.Location;

end

