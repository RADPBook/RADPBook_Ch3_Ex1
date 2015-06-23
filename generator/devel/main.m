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

%x0=0.01*[1,1,1,-1,zeros(1,N2*N2+N2+1)]
%x0=0.01*[1,1,1,1,zeros(1,N2*N2+N2+1+N3+N4+1)];
%x0=[.2,.5,-.5,.1,zeros(1,N2*N2+N2+1+N3+N4+1)]; %works
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

%controllererror(w1,w2)

disp('Phase two learning started')
Dzeta=Dzeta0*[1;-w2];
Thetaz=[Iphi3 Iphi4delta];
Xiz=Dzeta-Iu;
wfg=inv(Thetaz'*Thetaz)*Thetaz'*Xiz;
w3=wfg(1:N3)
w4=wfg(N3+1:end)


y=y(:,1:4);
plot(tt,y)
legend('1','2','3','4')


%ode23(@powersys0,[i*T+T,30],y(end,:))

[t2,y2]=ode23s(@powersys0,[i*T+T,10],y(end,:));
tt=[tt;t2];
y=[y;y2];

figure(2)
subplot(411)
plot(tt,y(:,1),'linewidth',2)
legend('w')
subplot(412)
plot(tt,y(:,2),'linewidth',2)
legend('x_1')
subplot(413)
plot(tt,y(:,3),'linewidth',2)
legend('x_2')
subplot(414)
plot(tt,y(:,4),'linewidth',2)
legend('z')

anno


%figure(2)
%ode23s(@powersys0,[0,10],X(end,1:4))
%%
figure(3)
plot(tt,y(:,1),'linewidth',2)
legend('w')
anno2
ylabel('Dynamic uncertainty')
xlabel('Time (sec)')
%%
figure(4)
plot(tt,y(:,2),'linewidth',2)
legend('\delta-\delta_0')
anno2
axis([0 10 -0.3 .6])
ylabel('Deviation of the power angle (rad)')
xlabel('Time (sec)')
%%

figure(5)
plot(tt,y(:,3),'linewidth',2)
legend('\omega')
anno2
axis([0 10 -5 5])
ylabel('Relative frequency (rad/s)')
xlabel('Time (sec)')

%%

figure(6)
plot(tt,y(:,4),'linewidth',2)
legend('P_m-P_0')
ylabel('Deviation of the mechanical power (per unit)')
xlabel('Time (sec)')
anno2
