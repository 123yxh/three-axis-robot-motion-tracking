% �������Ϊ�Ƕ�theta�����Ϊ��ת����R
function R=RotZ(theta)

    num=3; % ���ñ���numΪ3����ʾ��z����ת

    s1=sin(theta);  % ����Ƕ�theta������ֵ
    c1=cos(theta);  % ����Ƕ�theta������ֵ
    switch (num)
        case 1
            R=[1 0 0;0 c1 -s1;0 s1 c1]; % ��x�����ת����
        case 2
            R=[c1 0 s1;0 1 0;-s1 0 c1];  % ��y�����ת����
        case 3
            R=[c1 -s1 0;s1 c1 0;0 0 1];  % ��z�����ת����
    end
end