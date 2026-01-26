%% 辅助函数：视场绘制，输入位无人机位置、视场朝向、fov参数、色彩参数
function drawFOVCone(pos, forward, fov, varargin)
    % 解析参数
    p = inputParser;
    addParameter(p,'Color',[0 1 0]);
    addParameter(p,'Alpha',0.15);
    parse(p,varargin{:});
    C = p.Results.Color;
    A = p.Results.Alpha;
    % 角度
    hAng = deg2rad(fov.halfHAngleDeg);
    vAng = deg2rad(fov.halfVAngleDeg);
    % 近远距离
    z1 = fov.minRange;
    z2 = fov.maxRange;
    % 近截面半宽半高
    w1 = z1 * tan(hAng);
    h1 = z1 * tan(vAng);
    % 远截面半宽半高
    w2 = z2 * tan(hAng);
    h2 = z2 * tan(vAng);
    % 局部坐标系下8个顶点（z轴为forward）
    % 近面
    Pn = [ -w1  w1  w1 -w1;
           -h1 -h1 h1  h1;
            z1  z1  z1  z1 ];
    % 远面
    Pf = [ -w2  w2  w2 -w2;
           -h2 -h2 h2  h2;
            z2  z2  z2  z2 ];
    % 旋转矩阵：z轴 → forward
    ez = [0;0;1];
    f = forward(:)/norm(forward);
    v = cross(ez,f);
    s = norm(v);
    c = dot(ez,f);
    if s < 1e-6
        R = eye(3);
    else
        vx = [  0   -v(3)  v(2);
              v(3)   0   -v(1);
             -v(2) v(1)   0 ];
        R = eye(3) + vx + vx*vx*((1-c)/(s^2));
    end
    % 旋转并平移
    Pn = R*Pn + pos(:);
    Pf = R*Pf + pos(:);
    % 顶点编号：
    % 1-4: 近面  5-8: 远面
    V = [Pn Pf];
    faces = [
        1 2 6 5;   % 下
        2 3 7 6;   % 右
        3 4 8 7;   % 上
        4 1 5 8;   % 左
        5 6 7 8;   % 远面
        1 2 3 4];  % 近面
    % 绘制
    for i = 1:size(faces,1)
        fidx = faces(i,:);
        patch( V(1,fidx), V(2,fidx), V(3,fidx), C, ...
               'FaceAlpha',A,'EdgeColor','none');
    end
end
