%% 辅助函数：线段与竖直圆柱相交（圆柱轴向为z，中心(cx,cy)，半径r，高度[zmin,zmax]），输入为无人机位置、目标位置、其中一个障碍物参数
function hit = segmentIntersectsCylinder(p0, p1, cyl)
% cyl: [cx cy r zmin zmax]
    cx = cyl(1); cy = cyl(2); r = cyl(3); zmin = cyl(4); zmax = cyl(5);
    d = p1 - p0;
    % 在XY平面求与圆的交点参数 t（0~1）
    ox = p0(1) - cx; oy = p0(2) - cy;
    dx = d(1);       dy = d(2);
    a = dx*dx + dy*dy;
    b = 2*(ox*dx + oy*dy);
    c = ox*ox + oy*oy - r*r;
    if a < 1e-12
        % 几乎竖直线：只检查XY是否在圆内
        if (ox*ox + oy*oy) > r*r
            hit = false; return;
        end
        % 再看z是否穿过高度区间
        z0 = p0(3); z1 = p1(3);
        zLow = min(z0,z1); zHigh = max(z0,z1);
        hit = ~(zHigh < zmin || zLow > zmax);
        return;
    end
    disc = b*b - 4*a*c;
    if disc < 0
        hit = false; return;
    end
    sqrtDisc = sqrt(disc);
    t1 = (-b - sqrtDisc)/(2*a);
    t2 = (-b + sqrtDisc)/(2*a);
    hit = false;
    % 只要存在 t in [0,1] 且对应z落在[zmin,zmax]就算遮挡
    if t1 >= 0 && t1 <= 1
        z = p0(3) + t1*d(3);
        if z >= zmin && z <= zmax
            hit = true; return;
        end
    end
    if t2 >= 0 && t2 <= 1
        z = p0(3) + t2*d(3);
        if z >= zmin && z <= zmax
            hit = true; return;
        end
    end
end