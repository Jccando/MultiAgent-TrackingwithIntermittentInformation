%% 与addcontrolmainfina.m区别在于可在任意时候插入一个新目标

%% ======================== 变量与符号说明 =========================
%  一、规模与基本参数
%  N            : 无人机数量
%  M            : 目标数量
%  T            : 仿真总时间步数
%  dt           : 离散时间步长
%  p            : 目标状态向量维数（待估参数维数）
%  sigma_v      : 回归观测噪声标准差
%  Qw           : 目标真实状态演化的过程噪声协方差
%
%  二、压缩扩散
%  p_thresh     : 启用压缩估计的维数阈值
%  comp_ratio   : 压缩比例d/p
%  useCompression : 是否启用压缩扩散
%  d            : 压缩后的参数维数
%  Phi          : 随机测量/投影矩阵，行归一化
%  s_omp        : OMP重构时的稀疏度参数
%  PPt          : Phi*Phi'，用于压缩域信息矩阵计算
%
%  三、扩散自适应估计算法
%  useNLMS      : 是否使用NLMS(否则为 LMS)
%  mu           : 步长参数(NLMS可较大，LMS需较小)
%  delta        : NLMS的正则项(防止除零)
%  p_est        : 实际估计维数(p或d)
%  w_hat        : 融合后的参数估计(p_est×M×N)
%  psi          : Adapt步后的中间估计
%  W_est_c      : 压缩域/原始域估计历史
%  W_est_full   : 原始参数域(full)估计历史
%
%  四、通信拓扑与扩散权重
%  Adj          : 无向通信邻接矩阵(N×N)
%  deg          : 各无人机节点度
%  A            : Metropolis组合权重矩阵
%  Adj_eff_time : 不同目标子群在各时刻的有效通信拓扑
%
%  五、目标分配与博弈变量
%  assignment           : 当前UAV→目标分配结果(N×1)
%  assignment_history   : 分配历史(N×T)
%  switch_info          : 记录无人机目标切换信息
%  eta_info             : 信息增益权重
%  lambda               : 负载均衡惩罚系数
%  eta_switch           : 切换代价权重
%
%  六、回归与协同信息条件
%  L            : 每个时间步的回归样本数
%  sigma_u      : 回归向量噪声尺度
%  U_cache      : 回归数据缓存 {UAV, 目标, 时间}
%  U_last       : 最近一次有效回归数据
%  W            : 协同信息滑动窗口长度
%  eps_PE       : 持续激励(PE)判定阈值
%  lambda_min_R : 协同信息矩阵最小特征值
%  PE_satisfied : PE条件是否满足
%  joint_conn   : 目标子群在滑窗内是否联合连通
%
%  七、真实状态与估计误差评估
%  X_targets            : 目标真实状态(p×M)
%  X_targets_history    : 目标真实轨迹历史
%  X_drones             : 无人机扩展状态(用于估计初值)
%  RMSE                 : 目标位置估计误差(RMSE)
%  innovation           : 局部更新创新量||psi-w_old||
%
%  八、可见性、盲区与视场（FOV）
%  fov                  : 无人机视场参数(角度、距离)
%  visible              : UAV–目标是否可见
%  blind_record         : 盲区记录(不可见为 true)
%
%  九、障碍物与环境建模
%  obstacles            : 圆柱障碍物集合 [cx cy r zmin zmax]
%  obs                  : 障碍物生成相关参数
%
%  十、控制与运动学
%  q, v         : 无人机真实位置/速度(3×N×T)
%  q_des        : 期望位置(由目标估计得到)
%  ctrl         : 控制与人工势场参数
%% ==================================================================

clc; clear; close all;
%% ====== 参数设置 ======
N = 20;
M = 3;       % === 最大目标数（含后续出现的） ===
M0 = 2;      % === 初始已知目标数 ===
T = 100;
dt = 1;
p = 6;          % 待估参数维数（目标状态向量维数）
sigma_v = 0.1;  % 回归噪声标准差 v
Qw = 0.01 * eye(p); % 目标运动噪声（用于生成 w*(t) 的真实演化）
% 目标出现与激活状态
t_birth = [1, 1, 30];      % 第3个目标在 t=30 出现
target_active = false(1,M);
target_active(1:M0) = true;   % 初始只激活前2个

% 感知模式：inactive / exposed / normal
target_mode = strings(1,M);
target_mode(:) = "inactive";
target_mode(1:M0) = "normal";

%% ======= 维数阈值：自动启用压缩方案 =======
p_thresh   = 100;
comp_ratio = 0.35;
useCompression = (p > p_thresh);
if useCompression
    d = max(1, ceil(comp_ratio * p));
    if d >= p
        error('压缩维数 d 必须小于 p；请调小 comp_ratio 或调大 p_thresh。');
    end
    Phi = randn(d, p);   % 行归一化的随即测量矩阵
    Phi = Phi ./ (vecnorm(Phi) + 1e-12);
    s_omp = min( max(5, ceil(0.1 * p)), p );  % OMP的稀疏度
    fprintf('[INFO] p=%d > %d，启用压缩扩散：d=%d, comp_ratio=%.2f, OMP稀疏度=%d\n', ...
        p, p_thresh, d, comp_ratio, s_omp);
else
    Phi = [];
    d = p;
    s_omp = 0;
    fprintf('[INFO] p=%d <= %d，使用原始扩散（不压缩）\n', p, p_thresh);
end

%% ======== 扩散 NLMS/LMS 参数 ===========
useNLMS = true; % true=NLMS, false=LMS
mu = 1.5;       % NLMS时取大，过大会发散；LMS需小
delta = 1e-3;   % NLMS 防止除0
if ~useNLMS
    mu = 0.05;
end

%% ======== 协同信息条件 ==========
W = 5;             % 滑动窗口长度
eps_PE = 1e-2;      % 阈值：lambda_min(Rcoop) >= eps_PE 视为满足

joint_conn = zeros(T,M);
lambda_min_R = zeros(T,M);
PE_satisfied = false(T,M);
innovation = zeros(N, M, T);   % 用于绘制谁在喂信息谁在吃信息

%% =========== 通信拓扑 + Metropolis 权重 ===============
Adj = zeros(N,N);
for i = 1:N
    Adj(i, mod(i-2,N)+1) = 1;
    Adj(i, mod(i,N)+1)   = 1;
end
Adj = Adj | Adj';   % 保证无向图对称性
deg = sum(Adj,2);
A = zeros(N,N);
for i = 1:N
    for j = 1:N
        if Adj(i,j) == 1
            A(i,j) = 1 / (max(deg(i),deg(j)) + 1);
        end
    end
    A(i,i) = 1 - sum(A(i,:));
end

%% ============ 目标分配初始化 ============
assignment = randi(M0, N, 1);   % === 只在已知目标中分配 ===
assignment_history = zeros(N,T);
assignment_history(:,1) = assignment;

%% ========= 初始化无人机与目标真实状态（w*(t)）============
X_drones  = rand(p,N)*10;   % 仅用于“初值”和状态扩展容器
X_targets = rand(p,M)*10;
if useCompression
    for j=1:M
        x = zeros(p,1);
        base_support = 1:min(6,p);
        extra = randperm(p, min(s_omp, p));
        support = unique([base_support(:); extra(:)]);
        x(support) = 10*randn(length(support),1);
        X_targets(:,j) = x;
    end
end
X_drones_history  = zeros(p,N,T); X_drones_history(:,:,1)  = X_drones;
X_targets_history = zeros(p,M,T); X_targets_history(:,:,1) = X_targets;

%% ================== 基于目标轨迹生成圆柱体障碍物的相关参数 ==================
Kobs_total = 50;    % 障碍物数量
obs.radius_range = [0.6, 1.2];  % 障碍物半径范围
obs.height_range = [3.0, 30.0]; % 障碍物高度范围
obs.offset_xy_sigma = 3.0;      % 障碍物相对轨迹的水平随机偏移标准差
obs.min_dist_to_traj = 1;       % 障碍物与目标轨迹的最小距离（防止直接覆盖轨迹）
obs.z_base = 0.0;               % 障碍物底部高度（地面）
obstacles = [];                 % 障碍物集合，每行: [cx cy r zmin zmax]

%% ========= NLMS/LMS 状态（按压缩/非压缩自动切换维数） =============
p_est = d;
w_hat = zeros(p_est,M,N);   % 融合后的参数估计
psi   = zeros(p_est,M,N);   % Adapt阶段更新结果
W_est_c    = zeros(p_est,M,N,T);  % 压缩域/原始域估计历史
W_est_full = zeros(p,M,N,T);      % 原始参数域估计历史

%% ======== 参数估计初值：用无人机位置/状态作为粗初值 ===========
init_noise_std = 0.2;
for i = 1:N
    w0_full = X_drones(:,i) + init_noise_std * randn(p,1);
    for j = 1:M
        if ~useCompression
            w_hat(:,j,i) = w0_full; % 原始域直接初始化
        else
            w_hat(:,j,i) = Phi * w0_full; % 投影到压缩域
        end
    end
end

W_est_c(:,:,:,1) = w_hat;
for i=1:N
    for j=1:M
        if ~useCompression
            W_est_full(:,j,i,1) = w_hat(:,j,i);
        else
            W_est_full(:,j,i,1) = omp(Phi, w_hat(:,j,i), s_omp);
        end
    end
end

%% ================== 潜在博弈切换记录 ==================
switch_info = cell(N,1);

%% ================== 回归数据缓存 ===================
U_cache = cell(N,M,T);      % 每个无人机-目标-时间的回归矩阵
U_last  = cell(N,M);        % 最近一次有效回归记录
Adj_eff_time = false(N,N,M,T);

%% ================== 误差评估（RMSE用原域full估计的1:3维）==================
RMSE = zeros(T,M);

%% ======== 分配效用参数 =========
eta_info   = 10.0;   % 信息增益权重
reg_R      = 1e-2;  % 信息矩阵正则
lambda     = 30;    % 负载均衡惩罚
eta_switch = 0.5;  % 切换代价系数
sigma_u    = 0.8;   % 回归噪声尺寸
L          = 3;     % 每步回归样本数

if useCompression
    PPt = Phi * Phi.';
else
    PPt = [];
end


%% ======== 博弈分配过程可视化：记录容器 ========
util_all        = nan(N, M, T);   % 每个(i,j)的候选效用
info_gain_all   = nan(N, M, T);   % ΔtrP（信息增益）
load_pen_all    = nan(N, M, T);   % 负载惩罚（全局项）
switch_cost_all = nan(N, M, T);   % 切换代价
best_util       = nan(N, T);      % 每个i最终选择的效用
best_choice     = nan(N, T);      % 每个i最终选择的目标

% 近似的“潜在函数”（把所有无人机最终选择的效用加总）
potential       = nan(T,1);
potential_info  = nan(T,1);
potential_load  = nan(T,1);
potential_sw    = nan(T,1);


%% ========================================================================
%  1. 生成完整的目标轨迹
%% ========================================================================
for t = 2:T
    for j=1:M
        if p >= 6
            ax = 0.5*randn(); ay = 0.5*randn(); az = 0.5*randn();
            vx = X_targets(4,j) + dt*ax;
            vy = X_targets(5,j) + dt*ay;
            vz = X_targets(6,j) + dt*az;
            v_mag = norm([vx; vy; vz]); 
            max_target_v = 5;
            if v_mag > max_target_v
                vx = vx/v_mag*max_target_v;
                vy = vy/v_mag*max_target_v;
                vz = vz/v_mag*max_target_v;
            end
            % 额外扰动，让目标运动更真实，也让观测带有所谓传感器噪声
            vx = vx + 0.1*randn();
            vy = vy + 0.1*randn();
            vz = vz + 0.05*randn();
            X_targets(1:3,j) = X_targets(1:3,j) + [vx; vy; vz];
            X_targets(4:6,j) = [vx; vy; vz];
        else
            % 低维状态直接简单随机游走
            X_targets(:,j) = X_targets(:,j) + 0.1*randn(p,1);
        end
        % 增加过程噪声
        X_targets(:,j) = X_targets(:,j) + chol(Qw,'lower')*randn(p,1);
        z_min = 0;     % 让目标在地面及以上运动 
        z_max = 25;    % 让目标在高度25m以下运动
        if p >= 3
            if X_targets(3,j) < z_min
                X_targets(3,j) = z_min;
                if p >= 6, X_targets(6,j) = 0; end
            elseif X_targets(3,j) > z_max
                X_targets(3,j) = z_max;
                if p >= 6, X_targets(6,j) = 0; end
            end
        end
    end
    X_targets_history(:,:,t) = X_targets;
end

% 基于轨迹生成障碍物
obstacles = generateCylindersNearTraj(X_targets_history(1:3,:,:), Kobs_total, obs);

%% =============== 控制/势场参数 ==============
ctrl.mass    = 1.0;  % 等效质量
ctrl.damp    = 1.6;  % 速度阻尼
ctrl.k_track = 2.5;  % 目标跟踪增益
ctrl.k_rep   = 1.2;  % 排斥势场的增益
ctrl.d_safe  = 1.8;  % 安全距离，间距小于该值时排斥力剧增
ctrl.d_infl  = 4.0;  % 势场影响距离，该距离外无势场作用
ctrl.maxAcc  = 3.5;  % 最大允许的加速度
ctrl.maxVel  = 10.0;  % 最大允许的速度
% FOV视场参数
fov.halfVAngleDeg = 58/2;    % 垂直半视场
fov.halfHAngleDeg = 87/2;    % 水平半视场
fov.maxRange     = 12;     
fov.minRange     = 0.6;
% 真实无人机位置/速度
q = zeros(3,N,T);
v = zeros(3,N,T);
q(:,:,1) = X_drones(1:3,:);
v(:,:,1) = zeros(3,N);
% 期望轨迹容器（在线更新）
q_des = nan(3,N,T);
for i=1:N
    j0 = assignment_history(i,1);
    q_des(:,i,1) = W_est_full(1:3, j0, i, 1);
end
% 盲区记录
blind_record = false(N,M,T);

%% ========================================================================
%  2. 协同估计 + 控制闭环主循环
%% ========================================================================
for t = 2:T
        % 新目标出现（加入系统）
        for j = 1:M
            if ~target_active(j) && t == t_birth(j)
                fprintf('[INFO] Target %d APPEARS at t=%d\n', j, t);
    
                target_active(j) = true;
                target_mode(j)   = "exposed";   % 初始暴露
                % 初始化估计（所有无人机）
                for i = 1:N
                    if ~useCompression
                        w_hat(:,j,i) = X_targets(:,j) + 0.5*randn(p,1);
                    else
                        w_hat(:,j,i) = Phi * (X_targets(:,j) + 0.5*randn(p,1));
                    end
                end
            end
        end

    %% 0) 准备“当前时刻”无人机状态（用真实位置 q）
    % 用于 infoContribution / 生成回归 / 可见性判断
    X_drones_cur = X_drones;              % 保留 p×N 结构
    X_drones_cur(1:3,:) = q(:,:,t-1);     % 用真实位置替换静止初值

    %% 1) 盲区判断（基于真实位置 q(:,i,t-1)与目标真实位置）
    visible = false(N,M);
    visible_fov = false(N,M); 
    for i = 1:N
        ui_pos = q(:,i,t-1);
        for j = 1:M
    
            if ~target_active(j)
                continue;
            end
    
            tgt_pos = X_targets_history(1:3,j,t);
    
            % ===== 真实FOV可见性（用于状态切换）=====
            forward = tgt_pos - ui_pos;
            if norm(forward) < 1e-6
                forward = [1;0;0];
            end
            forward = forward / norm(forward);
    
            visible_fov(i,j) = isVisibleFOVAndOcclusion( ...
                ui_pos, forward, tgt_pos, fov, obstacles);
    
            % ===== 信息可见性（用于估计/控制）=====
            if target_mode(j) == "exposed"
                visible(i,j) = true;   % 信息层：全局知道
            else
                visible(i,j) = visible_fov(i,j);
            end
            blind_record(i,j,t) = ~visible(i,j);
        end
    end
    
    %% ===== 暴露态 → 正常FOV（仅当“被分配的 UAV”真实看到）=====
    for j = 1:M
        if target_active(j) && target_mode(j) == "exposed"
    
            % 找到当前分配给目标 j 的 UAV
            idx_assigned = find(assignment == j);
    
            % 只要其中任意一架真实FOV可见
            if any(visible_fov(idx_assigned, j))
                target_mode(j) = "normal";
                fprintf('[INFO] Target %d switches to NORMAL at t=%d\n', j, t);
            end
        end
    end

    visCount = sum(visible,1); % 计算某目标可见无人机数量
    if any(visCount==0) % 当某目标分配到的所有无人机不可见时给出提示
        fprintf('t=%d: some target lost, visCount=%s\n', t, mat2str(visCount));
    end

    %% 2) 分配决策前：滑窗形成base信息矩阵R_base
    R_base = cell(1,M);  % 累计最近W步中所有分配到目标j的回归信息
    for j = 1:M
        Rj = zeros(p_est);
        if t >= W+1
            for tt = (t-W):(t-1)
                for ii = 1:N
                    if assignment_history(ii,tt) ~= j, continue; end
                    Uu = U_cache{ii,j,tt};
                    if isempty(Uu), continue; end
                    Rj = Rj + (Uu * Uu');
                end
            end
        end
        R_base{j} = (Rj + Rj')/2;  % 对称处理避免数值误差
    end

    %% 3) 潜在博弈：目标分配（infoContribution用X_drones_cur）
    n_bar = N / sum(target_active);  % 理想目标分配设定为均分
    for i = 1:N
        j_old = assignment(i);
        best_j = j_old;
        best_utility = -inf;
        best_reason = '';
        for j_try = find(target_active)
            temp_assign = assignment;
            temp_assign(i) = j_try; % 假设无人机i切换到目标j_try
            load_penalty_total = 0; % 初始化负载均衡惩罚
            for jj = 1:M
                idx = find(temp_assign == jj);
                n_j = length(idx);
                load_penalty_total = load_penalty_total + lambda * (n_j - n_bar)^2;
            end
            % 预测无人机i对目标j_try的信息贡献
            S_new = infoContribution(i, j_try, X_drones_cur, X_targets_history, p, ...
                                     useCompression, Phi, PPt, p_est, L, sigma_u, U_last);
            % 以协方差减少量作为信息增益
            trP_before = traceP_fromR(R_base{j_try}, sigma_v, reg_R);
            trP_after  = traceP_fromR(R_base{j_try} + S_new, sigma_v, reg_R);
            info_gain  = trP_before - trP_after;
            % 计算切换代价
            switch_cost = eta_switch * (j_try ~= j_old);
            % 得到效用值
            utility = eta_info * info_gain - load_penalty_total - switch_cost;
            % 记录：候选效用与分量
            util_all(i, j_try, t)        = utility;
            info_gain_all(i, j_try, t)   = info_gain;
            load_pen_all(i, j_try, t)    = load_penalty_total;
            switch_cost_all(i, j_try, t) = switch_cost;

            if utility > best_utility
                best_utility = utility;
                best_j = j_try;
                best_reason = sprintf('ΔtrP=%.4f, loadPen=%.2f, U=%.3f', info_gain, load_penalty_total, best_utility);
            end
        end
        if assignment(i) ~= best_j
            switch_info{i} = [switch_info{i}; {t, assignment(i), best_j, best_reason}];
        end
        assignment(i) = best_j;
        best_util(i,t)   = best_utility;
        best_choice(i,t) = best_j;
    end
    assignment_history(:,t) = assignment;
    % 记录：全局潜在函数（近似）
    % 用“每架无人机最终选择的效用分量”求和形成潜在函数轨迹
    potU = 0; potI = 0; potL = 0; potS = 0;
    for ii = 1:N
        jj = assignment(ii);
        if ~isnan(util_all(ii,jj,t))
            potU = potU + util_all(ii,jj,t);
            potI = potI + eta_info * info_gain_all(ii,jj,t);
            potL = potL + load_pen_all(ii,jj,t);
            potS = potS + switch_cost_all(ii,jj,t);
        end
    end
    potential(t)      = potU;
    potential_info(t) = potI;
    potential_load(t) = potL;
    potential_sw(t)   = potS;


    %% 4) 生成回归观测回归向量缓存（只在无人机i能看到其分配目标j时才生成回归数据）
    for i=1:N
        j = assignment(i);
        if visible(i,j)
            if p >= 6
                % 基于无人机与无人机期望位置相对方向构造回归向量主方向
                rel = W_est_full(1:3, j, i, t-1) - q(:,i,t-1);
                rel = rel / (norm(rel)+1e-6);
                base = [rel; 0.2*randn(3,1)];
                if p > 6
                    base = [base; 0.1*randn(p-6,1)];
                end
            else
                base = 0.8*randn(p,1);
            end
            U = zeros(p,L); % 构造L个回归样本
            for ell = 1:L
                U(:,ell) = base + 0.8*randn(p,1);
            end
            if ~useCompression
                U_cache{i,j,t} = U;
                U_last{i,j}    = U_cache{i,j,t};
            else
                U_cache{i,j,t} = Phi * U;
                U_last{i,j}    = U_cache{i,j,t};
            end
        else
            U_cache{i,j,t} = [];
        end
    end

    %% 5) ATC Adapt：局部NLMS/LMS
    for i=1:N
        j = assignment(i);
        w_old = w_hat(:,j,i);
        if visible(i,j)
            Uu = U_cache{i,j,t};
            Lc = size(Uu,2);
            d_obs = zeros(Lc,1);
            % 构造带噪的观测
            if ~useCompression
                x_true = X_targets_history(:, j, t);
                for ell = 1:Lc
                    u = Uu(:,ell);
                    d_obs(ell) = u' * x_true + sigma_v*randn();
                end
            else
                w_c_star = Phi * X_targets_history(:, j, t);
                for ell = 1:Lc
                    u_c = Uu(:,ell);
                    d_obs(ell) = u_c' * w_c_star + sigma_v*randn();
                end
            end
            % NLMS或LMS更新
            if useNLMS
                w_new = w_old;
                for ell = 1:Lc
                    u = Uu(:,ell);
                    e = d_obs(ell) - u' * w_new;
                    w_new = w_new + (mu/(delta + u'*u)) * u * e;
                end
            else
                e = d_obs - (Uu' * w_old);
                w_new = w_old + mu * (Uu * e);
            end
            psi(:,j,i) = w_new;
        else
            psi(:,j,i) = w_old; % 不可见则保持
        end
        innovation(i,j,t) = norm(psi(:,j,i) - w_old); % 计算是谁在喂信息谁在吃信息
    end

    %% 6) ATC Combine：同目标子群扩散融合
    for j = 1:M
        idxj = find(assignment == j);
        % 当前目标对应的拓扑图
        Adj_eff = false(N,N);
        Adj_eff(idxj,idxj) = Adj(idxj,idxj);
        Adj_eff_time(:,:,j,t) = Adj_eff;
        for ii = idxj'
            % 融合权重，并且对可见性进行增强，避免可见无人机权重低喂信息少
            w = A(ii, idxj);
            vis_flag = visible(idxj, j);
            alpha_vis = 1;
            w = w .* (1 + alpha_vis * vis_flag);
            sw = sum(w);
            if sw < 1e-12
                w = zeros(size(w));
                w(idxj == ii) = 1;
            else
                w = w / sw;
            end
            % 加权融合
            w_comb = zeros(p_est,1);
            for kk = 1:length(idxj)
                k = idxj(kk);
                w_comb = w_comb + w(kk) * psi(:,j,k);
            end
            w_hat(:,j,ii) = w_comb;
        end
    end
    W_est_c(:,:,:,t) = w_hat;

    %% 7) 生成原域full估计（用于RMSE&作为期望轨迹）
    for j=1:M
        for i=1:N
            if ~useCompression
                W_est_full(:,j,i,t) = W_est_c(:,j,i,t);
            else
                W_est_full(:,j,i,t) = omp(Phi, W_est_c(:,j,i,t), s_omp);
            end
        end
    end

    %% 8) RMSE
    for j=1:M
        idx = find(assignment==j);
        if isempty(idx)
            RMSE(t,j) = NaN;
            continue;
        end
        est_mean_full = mean(squeeze(W_est_full(:,j,idx,t)), 2);
        if p >= 3
            err_pos = est_mean_full(1:3) - X_targets_history(1:3, j, t);
            RMSE(t,j) = sqrt(mean(err_pos.^2));
        else
            err = est_mean_full - X_targets_history(:,j,t);
            RMSE(t,j) = sqrt(mean(err.^2));
        end
    end

    %% 9) 理论监测：联合连通+协同信息条件
    if t >= W
        for j=1:M
            % 获得W步内的联合通信图
            unionAdj = false(N,N);
            for tt = (t-W+1):t
                unionAdj = unionAdj | Adj_eff_time(:,:,j,tt);
            end

            idxj_now = find(assignment_history(:,t) == j);
            if length(idxj_now) <= 1
                joint_conn(t,j) = 1;
            else
                unionSub = unionAdj(idxj_now, idxj_now);
                joint_conn(t,j) = isConnectedUndirected(unionSub);
            end
            % 验证PE条件，确保协同信息矩阵最小特征值满足约束
            Rcoop = zeros(p_est);
            for tt = (t-W+1):t
                for i = 1:N
                    if assignment_history(i,tt) ~= j, continue; end
                    Uu = U_cache{i,j,tt};
                    if isempty(Uu), continue; end
                    Rcoop = Rcoop + (Uu * Uu');
                end
            end
            Rcoop = (Rcoop + Rcoop')/2 + 1e-12*eye(p_est);
            lambda_min_R(t,j) = min(eig(Rcoop));
            PE_satisfied(t,j) = (lambda_min_R(t,j) >= eps_PE);
        end
    end

    %% 10) 期望点q_des（在线更新：追踪“当前分配目标”的估计位置）
    for i = 1:N
        j_ass = assignment(i);
    
        % 新目标暴露阶段：直接追真实位
        if target_mode(j_ass) == "exposed"
            q_des(:,i,t) = X_targets_history(1:3, j_ass, t);
   
        % 正常阶段：追协同估
        else
            q_des(:,i,t) = W_est_full(1:3, j_ass, i, t);
        end
    end


    %% 11) 控制更新：由q_des推动无人机运动（并产生下一步可见性所需位置）
    for i=1:N
        qi = q(:,i,t-1);        % 当前无人机位置
        vi = v(:,i,t-1);        % 当前无人机速度
        qdi = q_des(:,i,t);     % 期望位置（由估计得到）
        % 目标跟踪吸引力
        F_track = -ctrl.k_track * (qi - qdi);
        % 人工势场排斥力初始化
        F_rep = zeros(3,1);
        % 无人机–无人机避障
        for j=1:N
            if j==i, continue; end
            qj = q(:,j,t-1);
            diff = qi - qj;
            dist = norm(diff) + 1e-9;
    
            if dist > ctrl.d_infl
                continue;   % 超出影响范围
            end
            if dist < ctrl.d_safe
                % 强排斥区
                F_rep = F_rep + ctrl.k_rep * ( (1/dist - 1/ctrl.d_safe) / (dist^3) ) * diff;
            else
                % 软排斥区（平滑过渡，防止抖动）
                soft = 0.15;
                F_rep = F_rep + soft * ctrl.k_rep * (1/(dist^3)) * diff;
            end
        end
        % 无人机–障碍物避障
        for k = 1:size(obstacles,1)
            cxy  = obstacles(k,1:2).';   % 圆柱中心 (x,y)
            r    = obstacles(k,3);       % 半径
            zmin = obstacles(k,4);       % 高度下界
            zmax = obstacles(k,5);       % 高度上界
            % 高度不重叠 → 无影响
            if qi(3) < zmin || qi(3) > zmax
                continue;
            end
            % 只在 XY 平面计算排斥
            diff_xy = qi(1:2) - cxy;
            dist_xy = norm(diff_xy) - r;  % 到圆柱表面的距离
            if dist_xy > ctrl.d_infl
                continue;
            end
            if dist_xy < ctrl.d_safe
                % 强排斥
                F_rep(1:2) = F_rep(1:2) + ...
                    ctrl.k_rep * ...
                    ( (1/dist_xy - 1/ctrl.d_safe) / (dist_xy^3) ) * diff_xy;
            else
                % 软排斥
                soft = 0.15;
                F_rep(1:2) = F_rep(1:2) + ...
                    soft * ctrl.k_rep * (1/(dist_xy^3)) * diff_xy;
            end
        end
        % 合力 → 加速度（含阻尼）
        F_total = F_track + F_rep;
        acc = (F_total - ctrl.damp * vi) / ctrl.mass;
        % 加速度饱和
        a_norm = norm(acc);
        if a_norm > ctrl.maxAcc
            acc = acc / a_norm * ctrl.maxAcc;
        end
        % 速度、位置更新
        vi_new = vi + dt * acc;
        % 速度饱和
        v_norm = norm(vi_new);
        if v_norm > ctrl.maxVel
            vi_new = vi_new / v_norm * ctrl.maxVel;
        end
        qi_new = qi + dt * vi_new;
        v(:,i,t) = vi_new;
        q(:,i,t) = qi_new;
    end
    % 记录无人机扩展状态历史(看看后面要不要用吧，暂时还没用)
    X_drones_history(:,:,t) = X_drones_cur;
end


%% ========================================================================
%  3. 重要结果可视化、3D 动画可视化(使用闭环生成的 q / q_des)
%% ========================================================================
% plotting defaults
set(groot, 'defaultFigureColor','w');
set(groot, 'defaultAxesFontName','Times New Roman');
set(groot, 'defaultTextFontName','Times New Roman');
set(groot, 'defaultAxesFontSize',11);
set(groot, 'defaultTextFontSize',11);
set(groot, 'defaultAxesLineWidth',1.0);
set(groot, 'defaultLineLineWidth',1.8);
set(groot, 'defaultAxesBox','on');
set(groot, 'defaultAxesGridLineStyle',':');
set(groot, 'defaultAxesXGrid','on', 'defaultAxesYGrid','on');
set(groot, 'defaultLegendBox','off');
set(groot, 'defaultLegendLocation','best');
set(groot, 'defaultFigureUnits','centimeters');
set(groot, 'defaultFigurePosition',[2 2 14 9]); % 常见单栏图比例
gifFile = 'UAV_Swarm_Tracking.gif';  % GIF 文件名
gifDelay = 0.5;                      % 每帧时间间隔（秒）


%% ================= 动态可视化 =================
% UAV的3D模型参数
params.L  = 0.6;
params.Rr = 0.18;
params.Wr = 0.04;
params.H_body = 0.25;
params.D_body = 0.20;
params.bodyColor  = [0.25 0.55 0.95];
params.edgeColor  = [0 0 0];
params.bladeColor = [0.1 0.1 0.1];
params.bodyAlpha  = 0.85;
params.bladeAlpha = 0.55;
figure('Name','UAV Swarm: Closed-Loop Tracking with Visibility–Estimation–Control and APF Obstacle Avoidance (3D)');
grid on; hold on; view(3);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Closed Loop: q for FOV, W_{est}^{full} generates q_{des}, controller updates q');
% 期望轨迹
for i = 1:N
    plot3(squeeze(q_des(1,i,:)), ...
          squeeze(q_des(2,i,:)), ...
          squeeze(q_des(3,i,:)), ...
          '--', 'LineWidth', 1.0);
end
axis equal;
zlim([0 30]);
set(gca,'ZLimMode','manual');
skip = 1;
targetColors = lines(M);
% 直接清空出生前的轨迹吧避免画直线
for j = 1:M
    X_targets_history(1:3, j, 1:t_birth(j)-1) = NaN;
end
% 图注标志
legendOn = true;
for t = 1:skip:T
    cla; grid on; hold on; view(3);
    axis equal;
    zlim([0 30]);
    set(gca,'ZLimMode','manual');
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title(sprintf('Time Step: %d / %d', t, T));
    % 真实目标轨迹
    for j = 1:M
        % 确定绘制的“起点”
        % 起点是该目标的 t_birth(j)
        idx_start = t_birth(j);
        % 截取从“出生”到“当前”的这一段数据
        % X_targets_history 的维度是 (3, M, T)，截取后变成 (3, 长度)
        traj_segment = squeeze(X_targets_history(1:3, j, idx_start:t));
        % 处理刚出生只有1个点的情况（防止 squeeze 导致转置问题）
        if size(traj_segment, 2) == 1 && size(traj_segment, 1) == 3
             % 已经是3x1向量，无需操作
        elseif size(traj_segment, 1) ~= 3
             % 防止 squeeze把3x1xN压成N*3
             traj_segment = reshape(traj_segment, 3, []);
        end
        % 绘制
        plot3(traj_segment(1,:), traj_segment(2,:), traj_segment(3,:), ...
              '.-', ...                     % <--- 加上点标记，确保出生瞬间可见
              'LineWidth', 1.0, ...
              'Color', targetColors(j,:), ...
              'HandleVisibility','off');
    end

    % 障碍物
    drawCylinders(obstacles);
    % UAV轨迹
    for i = 1:N
        plot3(squeeze(q(1,i,1:t)), ...
              squeeze(q(2,i,1:t)), ...
              squeeze(q(3,i,1:t)), ...
              '-', 'LineWidth', 1.6, ...
              'HandleVisibility','off');
    end
    % UAV视场可视化
    for i = 1:N
        j_ass = assignment_history(i,t);
        if isnan(j_ass), continue; end
        ui_pos  = q(:,i,t);
        tgt_pos = X_targets_history(1:3,j_ass,t);
        forward = tgt_pos - ui_pos;
        if norm(forward) < 1e-6
            forward = [1;0;0];
        end
        forward = forward / norm(forward);
        dist  = norm(tgt_pos - ui_pos);
        alpha = 0.05 + 0.18 * max(0, 1 - dist/fov.maxRange);
        if blind_record(i,j_ass,t)
            fovColor = [1 0 0];        % Occluded (blind)
        else
            fovColor = [0 0.8 0.2];    % Visible
        end
        drawFOVCone(ui_pos, forward, fov, ...
            'Color', fovColor, 'Alpha', alpha);
    end
    % UAV模型
    for i = 1:N
        drawQuadrotorModel(q(:,i,t), eye(3), params);
    end
    % 图注编辑
    hTgt = plot3(nan,nan,nan,'-','LineWidth',1.5, ...
        'Color',targetColors(1,:), ...
        'DisplayName','True Target Trajectory');
    hUAV = plot3(nan,nan,nan,'-','LineWidth',1.8, ...
        'Color',[0 0 0], ...
        'DisplayName','UAV Trajectory');
    hObs = patch(nan,nan,nan,[0.6 0.6 0.6], ...
        'FaceAlpha',0.25,'EdgeColor','none', ...
        'DisplayName','Cylindrical Obstacle');
    hFOV_vis = patch(nan,nan,nan,[0 1 0], ...
        'FaceAlpha',0.12,'EdgeColor','none', ...
        'DisplayName','FOV (Visible)');
    hFOV_blk = patch(nan,nan,nan,[1 0 0], ...
        'FaceAlpha',0.12,'EdgeColor','none', ...
        'DisplayName','FOV (Occluded)');
    if legendOn
        legend([hUAV,hTgt,hFOV_vis,hFOV_blk,hObs], ...
            'Location','northeastoutside');
    end
    drawnow;
    % 保存 GIF 帧
    frame = getframe(gcf);        % 抓取当前图窗
    img = frame2im(frame);        % 转为图像
    [imind, cm] = rgb2ind(img, 256);
    if t == 1
        imwrite(imind, cm, gifFile, 'gif', ...
            'Loopcount', inf, ...
            'DelayTime', gifDelay);
    else
        imwrite(imind, cm, gifFile, 'gif', ...
            'WriteMode', 'append', ...
            'DelayTime', gifDelay);
    end
end

%% ================= 分配热图 =================
figure('Name','Assignment Heatmap');
imagesc(assignment_history);
colormap(turbo(M)); colorbar;
caxis([1 M]);
xlabel('Time Step');
ylabel('UAV Index');
title('Assignment History: UAV-to-Target Mapping Over Time');
% 叠加切换强度
hold on;
sw = sum(assignment_history(:,2:end) ~= assignment_history(:,1:end-1), 1);
[~, idxTop] = maxk(sw, min(8, numel(sw)));
for k=1:numel(idxTop)
    xline(idxTop(k)+1, 'k:', 'LineWidth',1.0, 'HandleVisibility','off');
end

%% ================= 目标负载平衡情况 =================
loadCount = zeros(T,M);
for t = 1:T
    for j = 1:M
        loadCount(t,j) = sum(assignment_history(:,t) == j);
    end
end
figure('Name','Target Load Balance'); hold on; grid on;
for j = 1:M
    plot(1:T, loadCount(:,j), ...
        'LineWidth', 2, ...
        'DisplayName', sprintf('Target %d', j));
end
yline(N/M,'--','Ideal Uniform Load','LineWidth',1.2);
xlabel('Time Step');
ylabel('Number of Assigned UAVs');
title('Load Balancing Performance (Closer to N/M is Better)');
legend('Location','best');

%% ================= 盲区时可视无人机贡献以及盲区标记 =================
innov_mean = nan(T,M);
blind_rate = nan(T,M);
for t = 1:T
    for j = 1:M
        idx = find(assignment_history(:,t) == j);
        if isempty(idx), continue; end
        innov_mean(t,j) = mean(innovation(idx,j,t),'omitnan');
        blind_rate(t,j) = mean(blind_record(idx,j,t),'omitnan');
    end
end
figure('Name','Innovation and Blind-Spot Ratio');
tiledlayout(2,1,'TileSpacing','compact');
nexttile; hold on; grid on;
for j = 1:M
    plot(innov_mean(:,j),'LineWidth',2, ...
        'DisplayName',sprintf('Target %d',j));
end
xlabel('Time Step');
ylabel('Mean ||\psi - w||');
title('Mean Innovation within Assigned Subgroup');
legend('Location','best');
nexttile; hold on; grid on;
for j = 1:M
    plot(blind_rate(:,j),'LineWidth',2, ...
        'DisplayName',sprintf('Target %d',j));
end
xlabel('Time Step');
ylabel('Blind-Spot Ratio (0–1)');
title('Blind-Spot Ratio within Assigned Subgroup (Lower is Better)');
legend('Location','best');

%% ================= PE情况 =================
figure('Name','PE Condition: \lambda_{min}(R_{coop})'); hold on; grid on;
for j = 1:M
    plot(lambda_min_R(:,j),'LineWidth',2, ...
        'DisplayName',sprintf('Target %d',j));
end
yline(eps_PE,'--','\epsilon_{PE}','LineWidth',1.2);
xlabel('Time Step');
ylabel('\lambda_{min}(R_{coop})');
title('Minimum Eigenvalue of Cooperative Information Matrix (PE Monitoring)');
legend('Location','best');

%% ================= 联合连通条件 =================
figure('Name','Connectivity and PE Satisfaction');
tiledlayout(2,1,'TileSpacing','compact');
nexttile;
imagesc(PE_satisfied'); colormap(gray); colorbar;
xlabel('Time Step');
ylabel('Target Index');
title('PE Satisfied (1 = Yes)');
nexttile;
imagesc(joint_conn'); colormap(gray); colorbar;
xlabel('Time Step');
ylabel('Target Index');
title('Joint Connectivity (1 = Connected)');

%% ================= 真实与估计轨迹 =================
est_traj = nan(3,M,T);
for t = 1:T
    for j = 1:M
        idx = find(assignment_history(:,t) == j);
        if isempty(idx), continue; end
        est_mean_full = mean(squeeze(W_est_full(:,j,idx,t)),2);
        est_traj(:,j,t) = est_mean_full(1:3);
    end
end
figure('Name','True vs Estimated Target Trajectories (3D)');
hold on; grid on; view(3); axis equal;
for j = 1:M
    truep = squeeze(X_targets_history(1:3,j,:));
    estp  = squeeze(est_traj(:,j,:));
    plot3(truep(1,:), truep(2,:), truep(3,:), '-', ...
        'LineWidth',2, 'DisplayName',sprintf('True Target %d',j));
    plot3(estp(1,:), estp(2,:), estp(3,:), '--', ...
        'LineWidth',2, 'DisplayName',sprintf('Estimated Target %d',j));
end

xlabel('X'); ylabel('Y'); zlabel('Z');
title('True vs Cooperative Estimated Target Trajectories');
legend('Location','bestoutside');

%% ================= 通信子图 =================
t_show = round(linspace(1,T,6));
figure('Name','Target Subgroup Communication Topology (Sampled Frames)');
tiledlayout(2,3,'TileSpacing','compact');

for k = 1:numel(t_show)
    t = t_show(k);
    nexttile;
    for j = 1:M
        G = graph(Adj_eff_time(:,:,j,t));
        plot(G);
        title(sprintf('Target %d at t = %d', j, t));
    end
end

%% ================== RMSE可视化 ==================
figure; hold on;
colors = lines(M);
for j=1:M
    y = RMSE(:,j);
    semilogy(1:T, y, '-', 'Color', colors(j,:), 'DisplayName', sprintf('Target %d',j));
end
% 全目标平均（忽略NaN）
ymean = nanmean(RMSE,2);
semilogy(1:T, ymean, 'k-', 'LineWidth',2.6, 'DisplayName','Mean over targets');
% 稳态窗口
tw = round(0.8*T):T;
mss = nanmean(ymean(tw));
sss = nanstd(ymean(tw));
xpatch = [tw(1) tw(end) tw(end) tw(1)];
ypatch = [mss-sss mss-sss mss+sss mss+sss];
patch(xpatch, ypatch, [0.7 0.7 0.7], 'FaceAlpha',0.25, 'EdgeColor','none', ...
      'DisplayName','Steady-state mean±std');
xlabel('Time step');
ylabel('RMSE (m)');
title(sprintf('%s Diffusion-%s Tracking Error', ternary(useCompression,'CS','Raw'), ternary(useNLMS,'NLMS','LMS')));
legend;
disp(mean(RMSE));

%% ================= 博弈候选效用差值 / 决策优势度 (适配任意 M) =================
figure('Name','Game Utility Analysis');

if M == 2
    % === M=2: 经典模式 (保留你原来的习惯) ===
    % 红色代表偏向目标1，蓝色代表偏向目标2
    dU = squeeze(util_all(:,1,:) - util_all(:,2,:)); 
    imagesc(dU);
    colormap(turbo); 
    hcb = colorbar;
    title(hcb, 'U1 - U2');
    xlabel('Time Step'); ylabel('UAV Index');
    title('Utility Difference: U(i,1) - U(i,2) (Red->T1, Blue->T2)');
    
else
    % === M >= 3: 决策优势度模式 (Best - 2ndBest) ===
    % 计算每个无人机在每个时刻的：[第一名效用 - 第二名效用]
    % 颜色越亮(黄/红)代表决策越坚定；颜色越暗(蓝)代表在两个目标间犹豫（易切换）
    
    margin_mat = zeros(N, T);
    
    for t = 1:T
        for i = 1:N
            % 1. 取出当前时刻该无人机对所有目标的效用
            u_vec = squeeze(util_all(i, :, t));
            
            % 2. 排除 NaN (还未出生的目标效用是 NaN)
            u_vec = u_vec(~isnan(u_vec));
            
            % 3. 计算优势差值
            if length(u_vec) >= 2
                u_sorted = sort(u_vec, 'descend'); % 降序排列
                margin_mat(i, t) = u_sorted(1) - u_sorted(2);
            elseif length(u_vec) == 1
                % 只有一个目标可选时，优势设为一个较大的固定值（显示为最稳）
                margin_mat(i, t) = 5; 
            else
                % 无目标可选
                margin_mat(i, t) = 0;
            end
        end
    end
    
    imagesc(margin_mat);
    colormap(parula); % 推荐 parula 或 jet
    hcb = colorbar;
    title(hcb, 'Margin');
    
    % 设置色阶范围，防止由于刚出现目标时的巨大数值导致细节看不清
    % 根据你的 eta_info=10，差值通常在 0~5 之间
    caxis([0, 5]); 
    
    xlabel('Time Step'); ylabel('UAV Index');
    title(sprintf('Decision Margin (Best Utility - 2ndBest) | M=%d', M));
end

hold on;
% === 通用：叠加显示切换频繁的时间点 ===
% 统计每一时刻发生了多少次切换
switch_counts = sum(assignment_history(:,2:end) ~= assignment_history(:,1:end-1), 1);
% 找出切换次数最多的 8 个时刻
[~, idxTop] = maxk(switch_counts, min(8, numel(switch_counts)));
for k = 1:numel(idxTop)
    % 用白色虚线标出切换高峰时刻
    xline(idxTop(k)+1, 'w:', 'LineWidth', 1.5, 'HandleVisibility', 'off');
end
%% ================= 博弈效用分解热图（按最终选择目标取值） =================
U_info = nan(N,T);
U_load = nan(N,T);
U_sw   = nan(N,T);
U_tot  = nan(N,T);

for t=2:T
    for i=1:N
        j = assignment_history(i,t);
        if isnan(j), continue; end
        U_tot(i,t)  = util_all(i,j,t);
        U_info(i,t) = eta_info * info_gain_all(i,j,t);
        U_load(i,t) = -load_pen_all(i,j,t);         % 取负号：越大越“好看”（惩罚越小）
        U_sw(i,t)   = -switch_cost_all(i,j,t);      % 同理
    end
end

figure('Name','Game Utility Decomposition (chosen target)');
tiledlayout(2,2,'TileSpacing','compact');

nexttile; imagesc(U_tot);  colormap(turbo); colorbar;
title('Chosen Total Utility'); xlabel('Time'); ylabel('UAV');

nexttile; imagesc(U_info); colormap(turbo); colorbar;
title('Chosen Info Term  (+η·ΔtrP)'); xlabel('Time'); ylabel('UAV');

nexttile; imagesc(U_load); colormap(turbo); colorbar;
title('Chosen Load Term  (-LoadPenalty)'); xlabel('Time'); ylabel('UAV');

nexttile; imagesc(U_sw);   colormap(turbo); colorbar;
title('Chosen Switch Term (-SwitchCost)'); xlabel('Time'); ylabel('UAV');
%% ================= 潜在函数（近似）轨迹 =================
figure('Name','Potential Function (approx)'); hold on; grid on;
plot(1:T, potential, 'LineWidth',2.5, 'DisplayName','Potential (sum chosen utility)');
plot(1:T, potential_info, 'LineWidth',1.8, 'DisplayName','Info term sum');
plot(1:T, -potential_load, 'LineWidth',1.8, 'DisplayName','Load penalty sum');
plot(1:T, -potential_sw, 'LineWidth',1.8, 'DisplayName','Switch cost sum');
xlabel('Time Step');
ylabel('Value');
title('Game Evolution: Potential & Component Sums (visual diagnostic)');
legend('Location','best');
%% ================= 切换事件光栅图 =================
switchMat = false(N,T);
switchMat(:,2:T) = assignment_history(:,2:T) ~= assignment_history(:,1:T-1);

figure('Name','Switch Raster');
imagesc(switchMat);
colormap(gray); colorbar;
xlabel('Time Step'); ylabel('UAV Index');
title('Switch Events Raster (1 = UAV switched target)');
%% ================= 分配二部图快照（采样时刻） =================
t_show = unique(round(linspace(1,T,6)));
figure('Name','Bipartite Assignment Graph (sampled)');
tiledlayout(2,3,'TileSpacing','compact');

for kk = 1:numel(t_show)
    t = t_show(kk);
    nexttile; hold on; axis off;
    title(sprintf('Assignment Graph @ t=%d', t));

    % 左侧 UAV 节点
    uavY = linspace(0,1,N);
    uavX = zeros(1,N);
    % 右侧 Target 节点
    tgtY = linspace(0.2,0.8,M);
    tgtX = ones(1,M);

    % 画边：UAV -> assigned target
    for i=1:N
        j = assignment_history(i,t);
        plot([uavX(i) tgtX(j)], [uavY(i) tgtY(j)], '-', 'LineWidth',0.8);
    end
    % 画点
    scatter(uavX, uavY, 20, 'filled');
    scatter(tgtX, tgtY, 80, 'filled');

    % 标注
    for i=1:N
        text(uavX(i)-0.02, uavY(i), sprintf('U%d',i), 'HorizontalAlignment','right', 'FontSize',8);
    end
    for j=1:M
        text(tgtX(j)+0.02, tgtY(j), sprintf('T%d',j), 'HorizontalAlignment','left', 'FontSize',10);
    end
    xlim([-0.2 1.2]); ylim([-0.05 1.05]);
end