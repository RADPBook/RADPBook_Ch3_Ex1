%parameters.m


delta0=1;  % SS power angle
Vs=1;        % Bus voltage
KG=1;        % Governer constant 
TT=2;
R=0.05;
omega0=314.159;

Ef0=3;

D=5;
H=4;

xd=1.863;
xdp=0.257;

xT=0.127;
xL=0.4853;

xds=xT+xL/2+xd;
xds_new=xT+xL+xd;
xdsp=xT+xL/2+xdp;
xdsp_new=xT+xL+xdp;

Td0=6.9; % sec
Tdp=xdsp/xds*Td0;


a1=1/Tdp;
a2=2*(xd-xdp)/Tdp*(Vs)^2/xds/xdsp;
a2_new=2*(xd-xdp)/Tdp*(Vs)^2/xds_new/xdsp_new;
a3=delta0;
b1=D/2/H;
b2=omega0/H*Vs/xds*Ef0;
b2_new=omega0/H*Vs/xds_new*Ef0;
b3=omega0/2/H;
c1=1/TT;
c2=KG/TT/R/omega0;


N1=12;3;%5;
N2=7;3;%4;
N3=7;%9;
N4=4;%15;

delta0_new=asin(xds_new/xds*sin(delta0));

P0=Vs/xds*Ef0*sin(delta0);
P0_new=Vs/xds_new*Ef0*sin(delta0_new);

r=1;
