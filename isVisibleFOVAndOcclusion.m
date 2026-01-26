%% 辅助函数：基于FOV锥体 + 圆柱遮挡判断可见性，输入为无人机位置、朝向单位向量、目标位置、fov视场参数、场地内的障碍物
function tf = isVisibleFOVAndOcclusion(uavPos, forwardUnit, tgtPos, fov, obstacles)
    v = tgtPos - uavPos;
    dist = norm(v);
    if dist < fov.minRange || dist > fov.maxRange
        tf = false; return;
    end
    % 建立无人机相机坐标系
    z_cam = forwardUnit(:);                 % 前方
    up_world = [0;0;1];
    if abs(dot(z_cam,up_world)) > 0.95
        up_world = [0;1;0];
    end
    x_cam = cross(up_world, z_cam);   x_cam = x_cam/norm(x_cam);   % 右
    y_cam = cross(z_cam, x_cam);      % 上
    % 目标在相机坐标系中的方向
    dir = v / (dist+1e-12);
    x = dot(dir, x_cam);    % 水平方向
    y = dot(dir, y_cam);    % 垂直方向
    z = dot(dir, z_cam);    % 前向
    if z <= 0
        tf = false; return;
    end
    % 水平 & 垂直角
    angH = atan2(x, z);
    angV = atan2(y, z);
    if abs(angH) > deg2rad(fov.halfHAngleDeg) || ...
       abs(angV) > deg2rad(fov.halfVAngleDeg)
        tf = false; return;
    end
    % 遮挡检测
    for k=1:size(obstacles,1)
        if segmentIntersectsCylinder(uavPos, tgtPos, obstacles(k,:))
            tf = false; return;
        end
    end
    tf = true;
end
