%% 辅助函数：基于目标轨迹附近生成圆柱体障碍物，输入为目标轨迹、 障碍物数量、障碍物几何参数
function obstacles = generateCylindersNearTraj(traj3xMxT, K, obs)
    [~,M,T] = size(traj3xMxT);
    obstacles = zeros(K,5);
    kk = 1;
    maxTry = 20000;
    tryCnt = 0;
    while kk <= K && tryCnt < maxTry
        tryCnt = tryCnt + 1;
        j = randi(M);
        t = randi(T);
        p = traj3xMxT(:,j,t);
        % 在轨迹点附近采样偏移
        dx = obs.offset_xy_sigma * randn();
        dy = obs.offset_xy_sigma * randn();
        if hypot(dx,dy) < obs.min_dist_to_traj
            continue;
        end
        cx = p(1) + dx;
        cy = p(2) + dy;
        r  = randInRange(obs.radius_range);
        h  = randInRange(obs.height_range);
        zmin = obs.z_base;
        zmax = obs.z_base + h;
        % 轻微的去重/间距（避免柱子太挤）
        ok = true;
        for q=1:kk-1
            if hypot(cx - obstacles(q,1), cy - obstacles(q,2)) < (r + obstacles(q,3) + 0.5)
                ok = false; break;
            end
        end
        if ~ok, continue; end
        obstacles(kk,:) = [cx cy r zmin zmax];
        kk = kk + 1;
    end
    if kk <= K
        obstacles = obstacles(1:kk-1,:);
        warning('障碍物生成未达到K=%d，仅生成了 %d 个（可增大maxTry或放宽间距约束）', K, size(obstacles,1));
    end
end