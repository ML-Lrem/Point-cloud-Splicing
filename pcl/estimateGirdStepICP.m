function girdStep = estimateGirdStepICP(xyzMoving,xyzFixed,girdStepICP)
pNumMoving = size(xyzMoving,1);
pNumFixed = size(xyzFixed,1);


% 通过knnsearch即‘k'临近计算xyz集合里面每个点离xyz集合里面的最近点
% 由于第一个搜索的点是他本身，所以k临近个数选择了2
[~,dMoving] = knnsearch(xyzMoving,xyzMoving,'k',2);
% idx是最邻近点的索引项，用不到；d是每个点离它最邻近点的距离值
% 计算平均距离，即距离密度，单位：m
pMoving = mean(dMoving(:,2));

[~,dFixed] = knnsearch(xyzFixed,xyzFixed,'k',2);
% idx是最邻近点的索引项，用不到；d是每个点离它最邻近点的距离值
% 计算平均距离，即距离密度，单位：m
pFixed = mean(dFixed(:,2));

disp(['pNumMoving',';  densityMoving',';  pNumFixed',';  densityFixed',';  girdStepICP'])
disp([num2str(pNumMoving),'  ',num2str(pMoving),'  ',num2str(pNumFixed),'  ',num2str(pFixed),'  ',num2str(girdStepICP)])

end

