clc; clear all; close all;
P_theta=-21.1;
D_theta=-3.165;
P_v=-0.05775;
I_v=-0.035;
mp=1.9;%kg �������� 
Ip=0.11;%kgm^2 ���������ĵ�ת������
mb=0.1;%kg ������(����)
r=0.11;%m ��뾶
h=0.16;%m �������ľ��������ĵĸ߶�
g=9.8;
Ib=0.4*mb*r^2;%���ת������
A=Ip+mp*h^2*(1-1/(1+mb/mp+Ib/(mp*r^2)));
range=[-25:-15];
for P_theta=range
    Ga1=tf([D_theta,P_theta],1);
    Ga2=tf([P_v,I_v],1);
    Gm1=tf(Ib+(mp+mb)*r^2+mp*h*r,[-A, 0, mp*h*g]*(Ib+(mp+mb)*r^2));
    Gm2=tf([-(Ip+mp*(h+r)^2+Ib+mb*r^2),0,mp*h*g],[-A, 0, mp*h*g ,0 ,0]*(Ib+(mp+mb)*r^2));
    G=Ga1*Gm1/(1+Ga2*Gm2);
    nyquist(G,'');hold on;
end
title('��ͬP_theta');
legend([num2str(range'),repmat('',length(range),1)]);
