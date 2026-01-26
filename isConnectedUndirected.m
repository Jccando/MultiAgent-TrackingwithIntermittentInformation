%% 辅助函数：无向图连通性判定，输入为目标子群的邻接矩阵，是扩散收敛与否的条件
function tf = isConnectedUndirected(AdjSub)
    n = size(AdjSub,1);
    visited=false(n,1); q=1; visited(1)=true;
    while ~isempty(q)
        v=q(1); q(1)=[];
        for u=find(AdjSub(v,:))
            if ~visited(u)
                visited(u)=true; q(end+1)=u; %#ok<AGROW>
            end
        end
    end
    tf=all(visited);
end