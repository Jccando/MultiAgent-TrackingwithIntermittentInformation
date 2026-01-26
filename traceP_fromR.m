%% 辅助函数：迹(tr(P))，P ≈ σ_v^2 (R + reg I)^-1，输入为回归矩阵、测量噪声标准差、正则化，
%             输出为参数协方差矩阵的迹，即信息增益
function trP = traceP_fromR(R, sigma_v, reg_R)
    n = size(R,1);
    Rr = (R + R')/2 + reg_R*eye(n);
    P  = (sigma_v^2) * inv(Rr);
    trP = trace(P) / n;
end