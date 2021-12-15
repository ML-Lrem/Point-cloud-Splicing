function [xyzMovingRegisted,rmse] = registePCD(xyzMoving,xyzFixed) 
%-------------------------------Seting-------------------------------------
% 参数设置,@ Todo 数据类型可优化
% in PPF
% % Minimum sample grid size
girdStepPPF = 0.01;

% in icp
% Minimum sample grid size
girdStepICP = 0.5; 
% Maximum number of iterations
maxIterations = 20;

% xyz2pcd
pcdMoving = pointCloud(xyzMoving);
pcdFixed = pointCloud(xyzFixed);

pcdTemp = pcdMoving;
rmseLast = 0;
isRmseLimit = false;
while ~isRmseLimit
    % ICP input pcd, output pcd
    [pcdTemp,rmse] = ICPRegister(pcdTemp,pcdFixed,girdStepICP,maxIterations);
%     if rmse>2
%         % PPF input pcd, output pcd
%         pcdTemp =  PPFRegiste(pcdTemp,pcdFixed,girdStepPPF);      
%     end
    if rmse<2 || abs(rmse-rmseLast)<0.01
       isRmseLimit = true;
    else
        rmseLast = rmse;
    end
end
% out xyz
pcdMovingRegisted = pcdTemp;
xyzMovingRegisted = pcdMovingRegisted.Location;

end


