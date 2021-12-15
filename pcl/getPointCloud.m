function xyz = getPointCloud(path)

% Judge loading mode
[~,~,type] = fileparts(path);
isXYZ = any(contains(['.txt','.xyz','.asc','.pts','.neu','.csv'],type));
isPCDorPLY = any(contains(['.pcd','.ply'],type));

if isXYZ
    xyzPoints = importdata(path);
    if size(xyzPoints,2)>3
        xyz = xyzPoints(:,1:3);
    else
        disp('error! This file is not recognized')
    end

elseif isPCDorPLY
    pcdPoints = pcread(path);
    xyz = double(pcdPoints.Location);

% error
else
    disp('error-->this type is not supported')
    xyz = [];
end

% xyz = single(xyz);

end

