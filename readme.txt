本github前期的理论介绍与简单验证对应论文已投稿于第三届空天前沿大会AFC；
后期的理论证明与仿真实验对应论文将投稿于IEEE Transactions on Automation Science and Engineering(T-ASE)期刊

前期的简单验证即信息形式卡尔曼滤波与NLMS滤波(均嵌入博弈效用函数)在估计效果的比较；
后期的理论证明即博弈效用的嵌入与基于协同信息自适应调整的NLMS滤波不会改变联合连通性与协同信息条件的满足、仿真实验即采用人工势场模拟多智能体追踪目标可抵抗智能体受遮挡的影响与新目标加入智能体仍能均衡分配智能体


main.m                        主程序，将在论文见刊后给出，包括简单验证与仿真实验两个版本
generateCylindersNearTraj.m	  用于在目标途经路径附近生成障碍物
infoContribution.m			      用于计算博弈决策分配目标的效用函数
traceP_fromR.m				        用于计算参数协方差矩阵的迹即信息增益
isConnectedUndirected.m		    用于判定无向图连通性以确保扩散收敛
isVisibleFOVAndOcclusion.m	  用于判断无人机视场内与目标之间是否有障碍物遮挡
segmentIntersectsCylinder.m	  用于判断直线与障碍物是否相交
drawCylinders.m 				      用于可视化障碍物
drawQuadrotorMoldel.m 		    用于可视化3D无人机模型
randInRange.m			          	用于在给定区间取均匀的随机数
ternary.m					            用于求三目运算即a?b:c
omp.m						              用于稀疏重构(考虑到无人机状态不具稀疏性，后续要在感知层用压缩估计系数重构)
