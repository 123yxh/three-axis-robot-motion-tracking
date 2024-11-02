%% 定义函数
f = @(x) exp(x) - x^2 + 3*x - 2;

%% 初始化参数
a = 0;  % 下界
b = 1;  % 上界
tol = 1e-10;  % 误差范围
max_iter = 1000;  % 最大迭代次数

%% 循环迭代求解--二分法
for iter = 1:max_iter
    c = (a + b) / 2;  % 找到中点
    if abs(f(c)) < tol  % 检查是否满足误差要求，是则打印精确解
        fprintf('满足要求的解: x = %.10f\n', c);
        fprintf('需要的iter：%d', iter)

        % 返回的结果是 x = 0.2575302855；iter：34
        return
    elseif f(a) * f(c) < 0  % 如果满足位于[a,c]之间
        b = c;  %更新上界
    else
        a = c;  %位于[b,c]之间，反之更新下界
    end
end

%% 未能找到满足要求的解
fprintf('%d次迭代后，在给定的误差范围内找不到解决方案\n', max_iter);
