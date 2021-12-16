function isRegsiteErr = judgeRegsiteErr(rmseLog)

isRegsiteErr = false;
k = -0.7;
b = 2.7;
% ²Î¿¼Ïß£ºy = -0.7*(Step/0.1)*n + 2.7 
for n = 2:3
    rmseRefer = k*n + b;
    isRegsiteErr = rmseLog(n) > rmseRefer;
    if isRegsiteErr
        break;
    end
end


end

