大问题大问题当所有无人机都处于盲区超过3个步长后就会跟不上目标从而原地徘徊..！！！之后再改进吧!
addcontrolmainfinal1.m      主程序，NLMS针对遮挡设计自适应变步长，根据“子群信息质量+自身观测”，子群信息可靠步长越小子群信息越差步长增大，并设上限避免噪声放大
addcontrolmainfinal.m 		主程序，NLMS定步长
pubagoalwhenrun.m          		主程序，NLMS定步长，在某时刻插入一个新目标

generateCylindersNearTraj.m	用于在目标途经路径附近生成障碍物
infoContribution.m			用于计算博弈决策分配目标的效用函数
traceP_fromR.m				用于计算参数协方差矩阵的迹即信息增益
isConnectedUndirected.m		用于判定无向图连通性以确保扩散收敛
isVisibleFOVAndOcclusion.m	用于判断无人机视场内与目标之间是否有障碍物遮挡
segmentIntersectsCylinder.m	用于判断直线与障碍物是否相交
drawCylinders.m 				用于可视化障碍物
drawQuadrotorMoldel.m 		用于可视化3D无人机模型
randInRange.m				用于在给定区间取均匀的随机数
ternary.m					用于求三目运算即a?b:c
omp.m						用于稀疏重构(考虑到无人机状态不具稀疏性，后续要在感知层用压缩估计系数重构)