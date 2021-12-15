function mesh = pcd2mesh(pcd)

% computeNormal
normal = pcnormals(pcd);
mesh = [pcd.Location,normal];

end

