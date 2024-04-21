clear;
clc;
close all;
syms L C_M C_T w_1 w_2 w_3 w_4 %在原体轴系 b0 下的转动惯量分量
e_g_b = [0,0,1];% 体轴系的重力方向
rs=[
    -1,-1,0
    -1,1,0
    1,1,0
    1,-1,0
];% (4,3) 四个电机在体轴系的位置
omegas=[
    1
    -1
    1
    -1]*e_g_b; % (4,3) 四个电机的角速度朝向(右手定则,与重力同向为正)

for i=1:size(rs,1)
    r_i = rs(i,:);
    rs(i,:)=r_i/norm(r_i);
end
rs=L*rs;

ws=[w_1,w_2,w_3,w_4];

M_b=0; % 力矩(体轴系分量)
T_b=0; % 升力(体轴系分量)
mat_control=zeros(4,'sym');
for i=1:size(rs,1)
    w_i_2 = power(ws(i),2);
    r_i = rs(i,:);
    c_T_i = C_T*-e_g_b;

    c_MT_i = cross(r_i,c_T_i);
    c_MR_i = -C_M*omegas(i,:);
    c_M_i = c_MT_i+c_MR_i;

    M_b=M_b+c_M_i*w_i_2;
    T_b=T_b+c_T_i*w_i_2;
    for k=1:4
        if k==1
            mat_control(1,i)=c_T_i(3);
        else
            mat_control(k,i)=c_M_i(k-1);
        end

    end
end
T_b=simplify(transpose(T_b))
M_b=simplify(transpose(M_b))
mat_control=simplify(mat_control)
formal_T = latex(T_b)
formal_M = latex(M_b)
formal_control_mat = latex(mat_control)
formal_control_inv = latex(simplify(inv(mat_control)))