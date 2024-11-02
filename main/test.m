clear;clc;close all  

%% 初始化
% 门型点定义
P1 = [-250; -150; -900];  % 定义第一个点P1的坐标
P2 = [-250; -75; -900];   % 定义第二个点P2的坐标
P3 = [-250+75; 0; -900];  % 定义第三个点P3的坐标
P4 = [-75+250; 0; -900];  % 定义第四个点P4的坐标
P5 = [250; -75; -900];    % 定义第五个点P5的坐标
P6 = [250; -150; -900];   % 定义第六个点P6的坐标

P = [P1 P2 P3 P4 P5 P6];  % 将所有点连接在一起，构成路径点的矩阵

% 计算路径每一段的长度
% 第1段 - 直线
s1 = sqrt(sum((P2 - P1).^2));  % 计算P1到P2的直线距离
% 第2段 - 圆弧
s2 = 2 * pi * sqrt(sum((P3(1) - P2(1)).^2)) / 4;  % 计算P2到P3的圆弧长度
% 第3段 - 直线
s3 = sqrt(sum((P4 - P3).^2));  % 计算P3到P4的直线距离
% 第4段 - 圆弧
s4 = 2 * pi * sqrt(sum((P5(1) - P4(1)).^2)) / 4;  % 计算P4到P5的圆弧长度
% 第5段 - 直线
s5 = sqrt(sum((P6 - P5).^2));  % 计算P5到P6的直线距离

s12345 = s1 + s2 + s3 + s4 + s5;  % 总路径长度

% 设置最大加速度和总时间
amax = 2;  % 最大加速度
T = sqrt(8.135 * s12345 / amax);  % 计算总时间T
svajArr = [];  % 创建空数组，用于存储位移、速度、加速度和跃度
tArr = [];     % 创建空数组用于存储时间
pArr = [];     % 创建空数组用于存储路径上的位置信息

mydt = 0.01;  % 设置时间步长

%% 循环遍历求解
% 计算路径的各个运动状态
for t = 0:mydt:T
    tao = t / T;  % 归一化时间
    % 计算位移s、速度v、加速度a、跃度j
    s = amax / 8.135 * T^2 * (20 * tao^3 - 45 * tao^4 + 36 * tao^5 - 10 * tao^6);
    v = amax / 8.135 * T * (60 * tao^2 - 180 * tao^3 + 180 * tao^4 - 60 * tao^5);
    a = amax / 8.135 * (120 * tao - 540 * tao^2 + 720 * tao^3 - 300 * tao^4);
    j = amax / 8.135 / T * (120 - 1080 * tao + 2160 * tao^2 - 1200 * tao^3);
    svajArr = [svajArr; s, v, a, j];  % 将计算的s、v、a、j添加到数组
    tArr = [tArr; t];  % 添加当前时间

    % 根据位移s，确定当前在路径的哪个段落并计算对应位置
    if s <= s1
        scal = s / s1;  % 归一化比例
        p = P1 + (P2 - P1) * scal;  % 线性插值计算位置
        pArr = [pArr; p'];
    elseif s <= (s1 + s2)
        scal = (s - s1) / s2;  % 圆弧段比例
        r = sqrt(sum((P3(1) - P2(1)).^2));  % 半径
        p = P2 + r * [cos(pi - scal * pi / 2) + 1; sin(pi - scal * pi / 2); 0];  % 计算圆弧位置
        pArr = [pArr; p'];
    elseif s <= (s1 + s2 + s3)
        scal = (s - s1 - s2) / s3;  % 线性段比例
        p = P3 + (P4 - P3) * scal;  % 线性插值
        pArr = [pArr; p'];
    elseif s <= (s1 + s2 + s3 + s4)
        scal = (s - s1 - s2 - s3) / s4;  % 圆弧段比例
        r = sqrt(sum((P5(1) - P4(1)).^2));  % 半径
        p = P4 + r * [cos(pi / 2 - scal * pi / 2); sin(pi / 2 - scal * pi / 2) - 1; 0];  % 计算圆弧位置
        pArr = [pArr; p'];
    else
        scal = (s - s1 - s2 - s3 - s4) / s5;  % 线性段比例
        p = P5 + (P6 - P5) * scal;  % 线性插值
        pArr = [pArr; p'];
    end
end

% 绘制位移、速度、加速度和跃度的曲线
S = amax / 8.135 * T^2;
for i = 1:4
    subplot(2, 2, i)
    plot(tArr, svajArr(:, i))
end

% 绘制3D路径图
figure();
scatter3(P(1,:), P(2,:), P(3,:), "filled");  % 绘制散点图
hold on; grid on; axis equal;
plot3(pArr(:,1), pArr(:,2), pArr(:,3), 'r', 'linewidth', 2);  % 绘制3D路径图
xlabel('X');

% 将路径转换到z轴方向
p = pArr;
p = [p(:, 1), 0 * p(:, 2), p(:, 2) - 900 + 150];

% 定义机器人的几何参数
R = 225;
r = 75;
l1 = 400;
l2 = 880;
size1 = 1;

% 初始化3D图形的句柄，用于更新绘制内容
figure();
h1 = plot3(p(:,1), p(:,2), p(:,3), 'r');
hold on; grid on; axis equal;
h2 = plot3(0, 0, 0, 'r', 'LineWidth', size1);
h3 = plot3(0, 0, 0, 'r', 'LineWidth', size1);
h4 = plot3(0, 0, 0, 'r', 'LineWidth', size1);

h5 = plot3(0, 0, 0, 'b', 'LineWidth', size1);
h6 = plot3(0, 0, 0, 'b', 'LineWidth', size1);
h7 = plot3(0, 0, 0, 'b', 'LineWidth', size1);

h8 = plot3(0, 0, 0, 'g', 'LineWidth', size1);
h9 = plot3(0, 0, 0, 'g', 'LineWidth', size1);
h10 = plot3(0, 0, 0, 'g', 'LineWidth', size1);

h11 = plot3(0, 0, 0, 'k', 'LineWidth', size1);
h12 = plot3(0, 0, 0, 'k', 'LineWidth', size1);
h13 = plot3(0, 0, 0, 'k', 'LineWidth', size1);

% 循环更新图像
for i = 1:size(p, 1)
    [theta1(i), theta2(i), theta3(i), ~, A, C, B] = inverse_delta(p(i,:)', R, r, l1, l2);
    O = [0; 0; 0];

    pA1 = [p(i,:)', A(:,1)];
    pA2 = [p(i,:)', A(:,2)];
    pA3 = [p(i,:)', A(:,3)];

    OC1 = [O, C(:,1)];
    OC2 = [O, C(:,2)];
    OC3 = [O, C(:,3)];
        AB1 = [A(:,1), B(:,1)];
    AB2 = [A(:,2), B(:,2)];
    AB3 = [A(:,3), B(:,3)];

    CB1 = [C(:,1), B(:,1)];
    CB2 = [C(:,2), B(:,2)];
    CB3 = [C(:,3), B(:,3)];

    % 每隔20个点，更新绘图的内容
    if rem(i, 20) == 0
        % 更新绘制的连杆位置
        set(h2, 'xdata', pA1(1,:), 'ydata', pA1(2,:), 'zdata', pA1(3,:));
        set(h3, 'xdata', pA2(1,:), 'ydata', pA2(2,:), 'zdata', pA2(3,:));
        set(h4, 'xdata', pA3(1,:), 'ydata', pA3(2,:), 'zdata', pA3(3,:));
        
        % 更新OC连线的位置
        set(h5, 'xdata', OC1(1,:), 'ydata', OC1(2,:), 'zdata', OC1(3,:));
        set(h6, 'xdata', OC2(1,:), 'ydata', OC2(2,:), 'zdata', OC2(3,:));
        set(h7, 'xdata', OC3(1,:), 'ydata', OC3(2,:), 'zdata', OC3(3,:));
        
        % 更新AB连线的位置
        set(h8, 'xdata', AB1(1,:), 'ydata', AB1(2,:), 'zdata', AB1(3,:));
        set(h9, 'xdata', AB2(1,:), 'ydata', AB2(2,:), 'zdata', AB2(3,:));
        set(h10, 'xdata', AB3(1,:), 'ydata', AB3(2,:), 'zdata', AB3(3,:));

        % 更新CB连线的位置
        set(h11, 'xdata', CB1(1,:), 'ydata', CB1(2,:), 'zdata', CB1(3,:));
        set(h12, 'xdata', CB2(1,:), 'ydata', CB2(2,:), 'zdata', CB2(3,:));
        set(h13, 'xdata', CB3(1,:), 'ydata', CB3(2,:), 'zdata', CB3(3,:));

        % 更新路径
        set(h1, 'xdata', p(:,1), 'ydata', p(:,2), 'zdata', p(:,3));
        drawnow;  % 刷新绘图
    end
end

% 保存关节角度数据
% dlmwrite('theta1.txt', [time_count', theta1' - theta1(1)], ',')
% dlmwrite('theta2.txt', [time_count', theta2' - theta2(1)], ',')
% dlmwrite('theta3.txt', [time_count', theta3' - theta3(1)], ',')

tend = tArr(end);  % 获取最后一个时间点
tArr = tArr / tArr(end);  % 归一化时间

% 计算关节角速度
vtheta1 = [0, diff(theta1)] / mydt * tend;
vtheta2 = [0, diff(theta2)] / mydt * tend;
vtheta3 = [0, diff(theta3)] / mydt * tend;

% 计算关节角加速度
atheta1 = [0, diff(vtheta1)] / mydt * tend;
atheta2 = [0, diff(vtheta2)] / mydt * tend;
atheta3 = [0, diff(vtheta3)] / mydt * tend;

%% 结果绘图
% 绘制关节角度随时间变化的曲线
figure();
plot(tArr, theta1, 'r', 'linewidth', 1); hold on;
plot(tArr, theta2, 'g', 'linewidth', 1);
plot(tArr, theta3, 'b', 'linewidth', 1);
xlabel('时间(s)');
ylabel('关节角度(rad)');
legend('θ_1', 'θ_2', 'θ_3');

% 绘制关节角速度随时间变化的曲线
figure();
plot(tArr, vtheta1, 'r', 'linewidth', 1); hold on;
plot(tArr, vtheta2, 'g', 'linewidth', 1);
plot(tArr, vtheta3, 'b', 'linewidth', 1);
xlabel('时间(s)');
ylabel('关节角速度(rad/s)');
legend('θ_1', 'θ_2', 'θ_3');

% 绘制关节角加速度随时间变化的曲线
figure();
plot(tArr, atheta1, 'r', 'linewidth', 1); hold on;
plot(tArr, atheta2, 'g', 'linewidth', 1);
plot(tArr, atheta3, 'b', 'linewidth', 1);
xlabel('时间(s)');
ylabel('关节角加速度(rad/s^2)');
legend('θ_1', 'θ_2', 'θ_3');


