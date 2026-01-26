%% 辅助函数：分配决策的"预期信息贡献" S_i->j，输入为无人机编号、目标编号、无人机真实位置、目标真实状态、上次观测的回归矩阵、压缩感知矩阵、回归向量噪声
%             输出为对应编号无人机加入对应编号目标后带来的R增量，即如果无人机i去追目标j能带来多少信息
function S = infoContribution(i, j, X_drones, X_targets, p, ...
                              useCompression, Phi, PPt, p_est, L, sigma_u, U_last)
    Uc = U_last{i,j};

    if ~isempty(Uc)
        S = Uc * Uc.';
        return;
    end

    if p >= 6
        rel = X_targets(1:3,j) - X_drones(1:3,i);
        rel = rel / (norm(rel) + 1e-6);
        base = [rel; zeros(3,1)];
        if p > 6
            base = [base; zeros(p-6,1)];
        end
    else
        base = zeros(p,1);
    end

    if ~useCompression
        S = L * (base*base.' + (sigma_u^2)*eye(p));
    else
        base_c = Phi * base;
        S = L * (base_c*base_c.' + (sigma_u^2)*PPt);
        if size(S,1) ~= p_est
            S = (sigma_u^2)*eye(p_est);
        end
    end
end