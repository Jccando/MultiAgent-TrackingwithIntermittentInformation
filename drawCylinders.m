%% 辅助函数：绘制圆柱体障碍物
function drawCylinders(obstacles)
    if isempty(obstacles), return; end
    nTheta = 24;
    theta = linspace(0,2*pi,nTheta);
    for k=1:size(obstacles,1)
        cx = obstacles(k,1); cy = obstacles(k,2); r = obstacles(k,3);
        zmin = obstacles(k,4); zmax = obstacles(k,5);
        x = cx + r*cos(theta);
        y = cy + r*sin(theta);
        % 侧面
        X = [x; x];
        Y = [y; y];
        Z = [zmin*ones(1,nTheta); zmax*ones(1,nTheta)];
        surf(X,Y,Z,'FaceAlpha',0.25,'EdgeColor','none');
        % 顶/底圈
        plot3(x,y,zmin*ones(1,nTheta),'k-','LineWidth',0.5);
        plot3(x,y,zmax*ones(1,nTheta),'k-','LineWidth',0.5);
    end
end