function dx=powersys0new(t,x)
global w2 w3 w4
parameters % load parameters

 w=x(1)-0.2025;
 x1=x(2);
 x2=x(3);
 z=x(4);


dx=Phi2(x1,x2);

u1=1/2*(rho0(x1^2+x2^2)^2+2*rho0(x1^2+x2^2)+2)*w2'*Phi2(x1,x2);

zeta=z-u1;

u=-w3'*Phi3(x1,x2,z)+2*w2'*Phi2(x1,x2)-0.1*zeta ...
  -zeta*(w4'*Phi4(x1,x2,z))^2*rho((x1^2+x2^2+zeta^2)/2)^2 ...
  -zeta/4*rho((x1^2+x2^2+zeta^2)/2)^2 ...
  -0.1*zeta/2*(rho(zeta^2/2)/rho(x1^2+x2^2))^2;


dw=-a1*w+a2*sin(x1/2+a3)*sin(x1/2);
dx1=x2;
dx2=-b1*x2-b2*cos(x1/2+a3)*sin(x1/2) ...
    +b3*(z-w*sin(x1+a3));
dz=-c1*z-c2*x2+u;

dx=[dw;dx1;dx2;dz];
end