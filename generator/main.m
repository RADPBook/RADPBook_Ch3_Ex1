%main
clear all
global w2 w3 w4;% N3 N4 w2 w3 w4

parameters %load parameters

Dphi1=[];
Iphi2phi2=[];
IQ=[];
Iphi2z=[];
Iphi3=[];
Iphi4delta=[];
Iu=[];

Dzeta0=[];

x0=[1,.5,1,0.1,zeros(1,N2*N2+N2+1+N3+N4+1)]; 
X=x0;

T=0.01;
tt=[];
y=[];

for i=1:199
    [t,X]=ode45(@powersys,[0,T]+i*T,X(end,:));
    
    Dphi1=[Dphi1; Phi1(X(end,2),X(end,3))'-Phi1(X(1,2),X(1,3))'];
    
    Iphi2phi2=[Iphi2phi2; X(end,4+1:4+N2*N2)-X(1,4+1:4+N2*N2)];
    
    Iphi2z=[Iphi2z;X(end,4+N2*N2+1:4+N2*N2+N2)-X(1,4+N2*N2+1:4+N2*N2+N2)];
    
    IQ=[IQ;X(end,4+N2*N2+N2+1)-X(1,4+N2*N2+N2+1)];
    
    
    Dzeta0=[Dzeta0;X(end,4)-X(1,4) ... 
    1/2*(rho0( X(end,2:3)*X(end,2:3)' )^2+2*rho0(X(end,2:3)*X(end,2:3)')+2)*Phi2(X(end,2),X(end,3))'-1/2*(rho0(X(1,2:3)*X(1,2:3)')^2+2*rho0(X(1,2:3)*X(1,2:3)')+2)*Phi2(X(1,2),X(1,3))'];
     
    
    Iphi3=[Iphi3; X(end, 4+N2*N2+N2+1+1:4+N2*N2+N2+1+N3)-X(1, 4+N2*N2+N2+1+1:4+N2*N2+N2+1+N3)];
    Iphi4delta=[Iphi4delta; X(end,4+N2*N2+N2+1+N3+1:4+N2*N2+N2+1+N3+N4)-X(1,4+N2*N2+N2+1+N3+1:4+N2*N2+N2+1+N3+N4)];
    
    Iu=[Iu;X(end,4+N2*N2+N2+1+N3+N4+1)-X(1,4+N2*N2+N2+1+N3+N4+1)];
    
    tt=[tt;t];
    y=[y;X];
end


figure(1)
plot(tt,y(:,1:4))
legend('1','2','3','4')
disp('Phase one learning started')
w2=[-1 zeros(1,N2-1)]'; %N2=2
%w2=[1 1/2 0 0 0]'; N2=5
w2old=0*w2;
%w2=1;
%r=.01;
it1=0;
while norm(w2-w2old)>.01
   

w2old=w2;
w1save=[];
w2save=[];
%for j=1:10
   % w2old=w2;
    it1=it1+1
    Theta=[Dphi1 2*r*Iphi2z-2*r*Iphi2phi2*kron(w2,eye(N2))];
    Xi=-IQ-Iphi2phi2*kron(w2,w2);
    w12=inv(Theta'*Theta)*Theta'*Xi;
    w1=w12(1:N1)
    w2=[w12(end-(N2-1):end)]
    w1save=[w1save w1];
    w2save=[w2save w2];
        if it1==1
        w11=w1;
    end
end

