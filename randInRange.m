%% 辅助函数：在给定区间rng2里取均匀随机数
function x = randInRange(rng2)
    x = rng2(1) + (rng2(2)-rng2(1))*rand();
end