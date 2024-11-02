% 输入参数为角度theta，输出为旋转矩阵R
function R=RotZ(theta)

    num=3; % 设置变量num为3，表示绕z轴旋转

    s1=sin(theta);  % 计算角度theta的正弦值
    c1=cos(theta);  % 计算角度theta的余弦值
    switch (num)
        case 1
            R=[1 0 0;0 c1 -s1;0 s1 c1]; % 绕x轴的旋转矩阵
        case 2
            R=[c1 0 s1;0 1 0;-s1 0 c1];  % 绕y轴的旋转矩阵
        case 3
            R=[c1 -s1 0;s1 c1 0;0 0 1];  % 绕z轴的旋转矩阵
    end
end