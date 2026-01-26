%% 辅助函数：采用3D的无人机模型，输入参数为位置、姿态矩阵、参数
function drawQuadrotorModel(pos, R, params)
    L  = params.L;
    Rr = params.Rr;
    Wr = params.Wr;
    H_body = params.H_body;
    D_body = params.D_body;
    bodyColor  = params.bodyColor;
    edgeColor  = params.edgeColor;
    bladeColor = params.bladeColor;
    bodyAlpha  = params.bodyAlpha;
    bladeAlpha = params.bladeAlpha;
    % 旋翼中心（X型）
    body_pts = [ ...
         L,  L, 0;
        -L,  L, 0;
        -L, -L, 0;
         L, -L, 0 ]';
    % 机身立方体（机体系）
    body_cube = [
        -D_body, -D_body,  D_body,  D_body, -D_body, -D_body,  D_body,  D_body;
        -D_body,  D_body,  D_body, -D_body, -D_body,  D_body,  D_body, -D_body;
        -H_body/2, -H_body/2, -H_body/2, -H_body/2, H_body/2, H_body/2, H_body/2, H_body/2];
    faces = [1 2 6 5;
             2 3 7 6;
             3 4 8 7;
             4 1 5 8;
             1 4 3 2;
             5 6 7 8];
    body_cube_world = R * body_cube + pos;
    patch('Vertices', body_cube_world', 'Faces', faces, ...
          'FaceColor', bodyColor, 'EdgeColor', edgeColor, ...
          'FaceAlpha', bodyAlpha, 'SpecularStrength', 0.5);
    % 机臂
    arms_body = [ body_pts(:,1), body_pts(:,3), NaN(3,1), body_pts(:,2), body_pts(:,4) ];
    arms_world = R * arms_body + pos;
    plot3(arms_world(1,:), arms_world(2,:), arms_world(3,:), 'k','LineWidth',3);
    % 桨叶
    for i = 1:4
        blade1_body = [ ...
            -Rr,  Rr,  Rr, -Rr;
            -Wr, -Wr,  Wr,  Wr;
             0,   0,   0,   0 ];
        blade2_body = [ ...
            -Wr, -Wr,  Wr,  Wr;
            -Rr,  Rr,  Rr, -Rr;
             0,   0,   0,   0 ];
        blade1_body = blade1_body + body_pts(:,i);
        blade2_body = blade2_body + body_pts(:,i);
        blade1_world = R * blade1_body + pos;
        blade2_world = R * blade2_body + pos;
        patch(blade1_world(1,:), blade1_world(2,:), blade1_world(3,:), ...
              bladeColor, 'FaceAlpha', bladeAlpha, 'EdgeColor', 'none');
        patch(blade2_world(1,:), blade2_world(2,:), blade2_world(3,:), ...
              bladeColor, 'FaceAlpha', bladeAlpha, 'EdgeColor', 'none');
    end
    plot3(pos(1), pos(2), pos(3), 'ko', 'MarkerFaceColor','k', 'MarkerSize',4);
end