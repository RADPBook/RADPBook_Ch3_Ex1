function dx=powersys(t,x)

parameters % load parameters

w=x(1);
x1=x(2);
x2=x(3);
z=x(4);

Iphi2phi2=x(4+1:4+N2^2);
Iphi2z=x(4+N2^2+1:4+N2^2+N2);
IQ=x(4+N2^2+N2+1);
Iphi3=x(4+N2^2+N2+1+1:4+N2^2+N2+1+N3);
Iphi4delta=x(4+N2^2+N2+1+N3+1:4+N2^2+N2+1+N3+N4);
Iu=x(4+N2^2+N2+1+N3+N4+1);

%controller
u=-x1-10*x2+0.001*sin(t);


dw=-a1*w+a2*sin(x1/2+a3)*sin(x1/2);
dx1=x2;
dx2=-b1*x2-b2*cos(x1/2+a3)*sin(x1/2)+b3*(z-w*sin(x1+a3));
%dx2=-b1*x2-b2*cos(a3)*x1/2+b3*(z-w*sin(x1+a3));
dz=-c1*z-c2*x2+u;


dIphi2phi2=kron(Phi2(x1,x2),Phi2(x1,x2));
dIphi2z=Phi2(x1,x2)*(z-w*sin(x1+a3));
dIQ=Q(x1,x2);

dIphi3=Phi3(x1,x2,z);
dIphi4delta=Phi4(x1,x2,z)*(-w*sin(x1+a3));
dIu=u;


dx=[dw;
    dx1;
    dx2;
    dz;
    dIphi2phi2;
    dIphi2z;
    dIQ;
    dIphi3;
    dIphi4delta;
    dIu];
end
