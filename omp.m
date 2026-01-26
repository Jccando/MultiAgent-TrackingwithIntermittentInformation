%% 辅助函数：OMP 稀疏重构，输入为压缩测量矩阵、压缩域中的向量、允许的最大稀疏度
function x = omp(Phi,y,s)
    [~,p]=size(Phi); r=y; idx=[];
    x_ls = [];
    for k=1:s
        [~,i]=max(abs(Phi'*r));
        idx=unique([idx,i]);
        A=Phi(:,idx); x_ls=A\y; r=y-A*x_ls;
        if norm(r)<1e-6, break; end
    end
    x=zeros(p,1);
    if ~isempty(idx) && ~isempty(x_ls)
        x(idx)=x_ls;
    end
end