function girdStep = estimateGirdStepICP(xyzMoving,xyzFixed,girdStepICP)
pNumMoving = size(xyzMoving,1);
pNumFixed = size(xyzFixed,1);


% ͨ��knnsearch����k'�ٽ�����xyz��������ÿ������xyz��������������
% ���ڵ�һ�������ĵ�������������k�ٽ�����ѡ����2
[~,dMoving] = knnsearch(xyzMoving,xyzMoving,'k',2);
% idx�����ڽ����������ò�����d��ÿ�����������ڽ���ľ���ֵ
% ����ƽ�����룬�������ܶȣ���λ��m
pMoving = mean(dMoving(:,2));

[~,dFixed] = knnsearch(xyzFixed,xyzFixed,'k',2);
% idx�����ڽ����������ò�����d��ÿ�����������ڽ���ľ���ֵ
% ����ƽ�����룬�������ܶȣ���λ��m
pFixed = mean(dFixed(:,2));

disp(['pNumMoving',';  densityMoving',';  pNumFixed',';  densityFixed',';  girdStepICP'])
disp([num2str(pNumMoving),'  ',num2str(pMoving),'  ',num2str(pNumFixed),'  ',num2str(pFixed),'  ',num2str(girdStepICP)])

end

