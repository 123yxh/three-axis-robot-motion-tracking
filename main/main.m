clear;clc;close all

%% 门型点
% 圆弧轨迹范围-半径设置
radius = 75;

% 定义1-6个点的坐标（三维）
P1 = [-250; -2*radius; -900];
P2 = [-250; -radius; -900];
P3 = [-250+radius; 0; -900];
P4 = [-radius+250; 0; -900];
P5 = [250; -radius; -900];
P6 = [250; -2*radius; -900];

% P = [P1 P2 P3 P4 P5 P6];
% hold on;grid on
% axis equal
% scatter3(P(1,:),P(2,:),P(3,:),"filled")

% 0.5*a*delta_t^2+a*delta_t*(t1-delta) = 75;
% 计算5段路径下的速度，时间，并设定起始的加速度时间，为15行的公式，可计算起始加速度a1，后续一直保持线速度大小不变

%% 初始参数设置---这部分内容和门型点的坐标一起确定了轨迹
delta_t = 2;    % 设置初始加速时间段
t1 = 2;         % 设置第一段运动时间
a1 = radius/(0.5*delta_t^2+delta_t*(t1-delta_t));    % 通过公式计算初始加速度a1
v1 = a1*delta_t;     % 计算初始速度v1

w2 = v1/radius;  % 通过速度计算角速度w2
t2 = pi/2/w2;  % 通过角速度计算第二段运动时间t2
t3 = (500-150)/v1;  % 第三段直线运动的时间t3
v3 = v1;  % 第三段速度v3与第一段相同
w4 = w2;  % 第四段角速度与第二段相同
t4 = t2;  % 第四段时间t4与第二段相同
t5 = t1;  % 第五段时间t5与第一段相同

t_z = t1+t2+t3+t4+t5;  % 总时间t_z为5段时间的总和
T = linspace(0,t_z,200);  % 生成时间序列T，共200个点

%% 循环仿真遍历结果
% 计算每个时间点的位移
for i = 1:length(T)
    t = T(i);
    if t <= delta_t  % 第一段加速运动
        p(i,:) = P1 + 0.5*a1*t^2*(P2-P1)/norm(P2-P1);  % 计算点P的位移
    elseif t > delta_t && t <= t1  % 第二段匀速运动
        p(i,:) = P1 + 0.5*a1*delta_t^2*(P2-P1)/norm(P2-P1)+v1*(t-delta_t)*(P2-P1)/norm(P2-P1);
    elseif t > t1 && t <= t2+t1  % 第三段曲线运动-如果想要调整圆弧轨迹大一点，可以把参数75设置大一点，比如说85
        p(i,:) = P2 + [radius*(1-cos(w2*(t-t1))) ;radius*sin(w2*(t-t1)); 0];
    elseif t > t2+t1 && t <= t3+t2+t1  % 第四段匀速直线运动
        p(i,:) = P3 + v3*(t-(t2+t1))*(P4-P3)/norm(P4-P3);
    elseif t > t3+t2+t1 && t <= t4+t3+t2+t1  % 第五段曲线运动
        p(i,:) = P4 + [radius*sin(w2*(t-(t3+t2+t1))) ;-radius*(1-cos(w2*(t-(t3+t2+t1)))); 0];
    elseif t > t4+t3+t2+t1 && t <= t5+t4+t3+t2+t1-delta_t  % 第六段匀速运动
        p(i,:) = P5 + v1*(t-(t4+t3+t2+t1))*(P6-P5)/norm(P6-P5);
    elseif t > t5+t4+t3+t2+t1-delta_t && t <= t5+t4+t3+t2+t1  % 第七段减速运动
        p(i,:) = P6 + 0.5*a1*(t_z-t)^2*(P5-P6)/norm(P5-P6);
    end
end

% plot3(p(:,1),p(:,2),p(:,3),'r') % 绘制3D路径图
hold on;grid on
axis equal

% 计算路径长度s
s = 0;
for i = 2:size(p,1)
    s = [s;s(end)+sqrt(sum((p(i,:)-p(i-1,:)).^2))];  % 累加计算每段路径的距离
end

% 将图像转换到z轴方向
p = [p(:,1),0*p(:,2),p(:,2)-900+150];
plot3(p(:,1),p(:,2),p(:,3),'r')  % 绘制转换后的3D路径图

% 定义机器人的几何参数
R = 225;
r = radius;
l1 = 400;
l2 = 880;
size1 = 1;

% 循环绘图程序  逆运动学计算和绘制动画
for i = 1:size(p,1)
    clf
    [theta1(i),theta2(i),theta3(i),~,A,C,B] = inverse_delta(p(i,:)',R,r,l1,l2);  % 计算逆运动学角度和点的位置
    O = [0;0;0]; % 初始化原点

    % 绘制连杆和节点
    pA1 = [p(i,:)',A(:,1)];
    pA2 = [p(i,:)',A(:,2)];
    pA3 = [p(i,:)',A(:,3)];
    
    OC1 = [O,C(:,1)];
    OC2 = [O,C(:,2)];
    OC3 = [O,C(:,3)];
    
    AB1 = [A(:,1),B(:,1)];
    AB2 = [A(:,2),B(:,2)];
    AB3 = [A(:,3),B(:,3)];
    
    CB1 = [C(:,1),B(:,1)];
    CB2 = [C(:,2),B(:,2)];
    CB3 = [C(:,3),B(:,3)];
    
    hold on
    grid on
    axis equal
    view(34,28)  % 设置视角

    plot3(pA1(1,:),pA1(2,:),pA1(3,:),'r','LineWidth',size1)
    plot3(pA2(1,:),pA2(2,:),pA2(3,:),'r','LineWidth',size1)
    plot3(pA3(1,:),pA3(2,:),pA3(3,:),'r','LineWidth',size1)
    
    plot3(OC1(1,:),OC1(2,:),OC1(3,:),'b','LineWidth',size1)
    plot3(OC2(1,:),OC2(2,:),OC2(3,:),'b','LineWidth',size1)
    plot3(OC3(1,:),OC3(2,:),OC3(3,:),'b','LineWidth',size1)
    
    plot3(AB1(1,:),AB1(2,:),AB1(3,:),'g','LineWidth',size1)
    plot3(AB2(1,:),AB2(2,:),AB2(3,:),'g','LineWidth',size1)
    plot3(AB3(1,:),AB3(2,:),AB3(3,:),'g','LineWidth',size1)
    
    plot3(CB1(1,:),CB1(2,:),CB1(3,:),'k','LineWidth',size1)
    plot3(CB2(1,:),CB2(2,:),CB2(3,:),'k','LineWidth',size1)
    plot3(CB3(1,:),CB3(2,:),CB3(3,:),'k','LineWidth',size1)

    plot3(p(:,1),p(:,2),p(:,3),'r') % 绘制路径
    pause(0.001)  % 暂停0.001秒，用于创建动画效果
end

% 保存theta角度变化
% dlmwrite('theta1.txt',[time_count',theta1'-theta1(1)],',')
% dlmwrite('theta2.txt',[time_count',theta2'-theta2(1)],',')
% dlmwrite('theta3.txt',[time_count',theta3'-theta3(1)],',')

% 绘制角度变化图
figure(2)
hold on
plot(T,theta1,'r')
plot(T,theta2,'g')
plot(T,theta3,'k')

% 计算并绘制角速度变化图
figure(3)
hold on
v_theta1 = (theta1(1,2:end)-theta1(1,1:end-1))/(T(2)-T(i));
v_theta2 = (theta2(1,2:end)-theta2(1,1:end-1))/(T(2)-T(i));
v_theta3 = (theta3(1,2:end)-theta3(1,1:end-1))/(T(2)-T(i));
plot(T(2:end),-v_theta1)
plot(T(2:end),-v_theta2)
plot(T(2:end),-v_theta3)

% 计算并绘制加速度变化图
figure(4)
hold on
a_theta1 = (v_theta1(1,2:end)-v_theta1(1,1:end-1))/(T(3)-T(i));
a_theta2 = (v_theta2(1,2:end)-v_theta2(1,1:end-1))/(T(3)-T(i));
a_theta3 = (v_theta3(1,2:end)-v_theta3(1,1:end-1))/(T(3)-T(i));
plot(T(3:end),a_theta1)
plot(T(3:end),a_theta2)
plot(T(3:end),a_theta3)

%% 主要的逻辑
% 1）定义"门型"的六个3D坐标点，并且通过运动学公式计算了路径中各段的速度、加速度和时间。
% 2）随后通过循环计算路径上各个点的位置，并通过 plot3 函数绘制出路径。
% 3）运动学相关的逆运动学计算，并绘制了随时间变化的机器人运动轨迹，同时绘制了相关的角度、速度和加速度的变化曲线。



