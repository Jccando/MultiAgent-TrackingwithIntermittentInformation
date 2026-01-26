%% 辅助函数：三目运算，即当cond为true时输出a否则输出b
function out = ternary(cond,a,b)
    if cond, out=a; else, out=b; end
end