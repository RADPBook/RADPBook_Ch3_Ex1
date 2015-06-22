%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% LP_example_nonlinear
% Objective: Simulation for our new paper on nonlinear codesign.
%
% Written by Yu Jiang, June 17th, 2013
% Copyright MERL, Cambridge MA 02139
% Yebin Wang, revised, Apr. 2014
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%function []=LP_example()
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all
close all
warning off
clc


%-------------------------- Begin Data-------------------------------------
global k dL dB mL mB kB R K w
itermax = 10;      % Maximum iteration numbers
iter    = 0;       % Counter of iterations

H_save  = [];
J1_save = [];
J2_save = [];

mLmin   = 1;
mL      = 1.5; % Initial value for mL
mLmax   = 3;

mBmin   = 15;
mB      = 20;
mBmax   = 25;


dLmin   = 10;
dL      = 15; % Initial value for dL
dLmax   = 20;


kBmin   = 10; %5;
kB      = 15; %7.5;
kBmax   = 20; %10;

dBmin   = 0.1;
dB      = 0.5;
dBmax   =   1;



R      =   1;           % weighting matrix R

p      =   [mL; mB; dL; kB; dB]; % Design parameters

x0     =   [-1 0 0 0]';  % Initial condition

k      =   [-2 0 0 0];  % Initial stabilziing control policy

tf     =   3;           % Simulation time duration
%%

%---------------------------- End Data-------------------------------------


%--------------------------------------------------------------------------
% Solve for the optimal contorl under fixed parameters, this is also the
% same step in conventional policy iteration.


A = [0    1             0       0;
    0 -dL/mL-dL/mB   kB/mB   dB/mB;
    0 0                0       1;
    0 dL/mB         -kB/mB  -dB/mB];

B = [0;
    1/mL+1/mB;
    0
    -1/mB];

ll = length(dPhi([0,0,0,0]));


w   =   [0 0 0 0 2*R/B(2)*(-k(1)) zeros(1,ll-5)]' % reset w for iterations


%%
%---------Solve for the initial cost---------------------------------------
[tcost,ycost] = ode45(@LP_model_calc,[0,100], [x0;zeros(4*4+4+1,1)]);
J =  ycost(end,end);
clear tcost ycost;
J1_save = [J];
J2_save = [J];
%--------------------------------------------------------------------------

tic()
disp('Working on the standard PI...')
for i=1:10
disp([num2str(i), '-th iteration'])
    X = [];
    Y = [];
    for x1 = -1.5:.5:1.5
        for x2 = -1.5:.5:1.5
            for x3 = -1.5:.5:1.5
                for x4 = -2:0.5:2
                    dP = dPhi([x1 x2 x3 x4]);
                    
                    %u = k * [x1 x2 x3 x4]';
                    
                    u = - 1/2 * inv(R) * B' * dPhi([x1 x2 x3 x4])' * w;
                  
                    X = [X;(dP*(A*[x1;x2;x3;x4]+B*u))'];
                    
                    Y = [Y; 1000*x1*x1+500*abs(x1)+u'*R*u];
                    
                end
            end
        end
    end
    w = X\(-Y); % update the weights
    
    %------ extract the linear portion of the  feedback gains--------------
    k = - 1/2 * inv(R) * [B(2)*w(5)+B(4)*w(7) 2*B(2)*w(2)+B(4)*w(9) ...
                     B(2)*w(8)+B(4)*w(10) 2*B(4)*w(4)+B(2)*w(9)]
                
    [tcost,ycost] = ode45(@LP_model_calc,[0,100], [x0;zeros(4*4+4+1,1)]);
    J =  ycost(end,end);
    clear tcost ycost;
    J2_save = [J2_save, J]; 
end

stand_PI_cpu = toc()
%%

disp('Simulating the standard PI...')
%--------Simulate the system performance without co-design-----------------
   K=k; [t0,y0]=ode45(@LP_model,[0,tf],x0);
   
   w0 = w;
for i = 1:length(y0)
 uwc0(i) = -1/2 * inv(R) * B'* dPhi(y0(i,1:4))' * w;
end

% figure(3)
% xx = -1:0.1:1;
% yy = -2:0.1:2;
% zz0 = zeros(length(xx),length(yy));
% 
% for i = 1:length(xx)
%  for j = 1:length(yy)   
%   zz0(i,j) = Phi([xx(i),yy(j),0,0])*w; 
%  end
% end
% 
% [xx,yy]=meshgrid(xx,yy);
% surf(xx,yy,zz0')
% xlabel('x')
%--------------------------------------------------------------------------

w   =   [0 0 0 0 2*R/B(2)*(-k(1)) zeros(1,ll-5)]'; % reset w for iterations
k      =   [-1 0 0 0]; 


disp('Working on the M-PI...')
tic()
while (iter<itermax)
    %J0   = Jcompac;
    iter = iter+1;        % Counter the number of iterations
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Solve the H2 problem when p is fixed
    % Compute the A, B matrices based on the updated parameters
    A = [     0    1             0       0;
        0 -dL/mL-dL/mB   kB/mB   dB/mB;
        0 0                0       1;
        0 dL/mB         -kB/mB  -dB/mB];
    B = [     0;
        1/mL+1/mB;
        0
        -1/mB];
    
    %-------------------- Gradient Policy Improvement----------------------
  
 
    X = [];
    Y = [];
    for x1 = -1.5:.5:1.5
        for x2 = -1.5:.5:1.5
            for x3 = -1.5:.5:1.5
                for x4 = -2:0.5:2
                    dP = dPhi([x1 x2 x3 x4]);
                    
                    u = - 1/2 * inv(R) * B' * dP' * w;
                    
                    %u = k * [x1 x2 x3 x4]';
                    
                    X = [X;
                        (dP*(A*[x1;x2;x3;x4]+B*u))'];
                    
                    Y = [Y; 1000*x1*x1+500*abs(x1) + u' * R * u];
                    
                end
            end
        end
    end
    
    w = -X\Y; % inv(X'*X)*X'*(-Y) % update the weights
    
    % extract the linear portion of the  feedback gains
    k = - 1/2 * inv(R) * [B(2)*w(5)+B(4)*w(7) 2*B(2)*w(2)+B(4)*w(9) ...
        B(2)*w(8)+B(4)*w(10) 2*B(4)*w(4)+B(2)*w(9)]
    
    
    %----System equivalence-based policy improvement-----------------------
    %
    % the feedback matrix, described as follows, should remain unchanged.
    % [  0            1                  0                      0         ]
    % [  k1/mL   k2/mL-dL/mL          k3/mL                    k4/mL      ]
    % [  0            0                  0                      1         ]
    % [ -k1/mB   dL/mB-k2/mB         -k3/mB-kB/mB             -k4/mB-dB/mB]
    %
    %
    % In the second optimization problem, the variable we consider is
    % PK=[k1
    %     k2
    %     k3
    %     k4
    %     mL
    %     mB
    %     dL
    %     kB
    %     dB]
    
    
    %preparing the equality constraints for QP solvers
    %       k(1) k(2) k(3) k(4) mL       mB       dL   kB  dB
    Aeq = [ 1  0  0  0  -k(1)/mL          0      0   0   0 %k1/mL
        0  1  0  0  -(k(2)-dL)/mL     0     -1   0   0 %k2/mL-dL/mL
        0  0  1  0  -k(3)/mL          0      0   0   0 %k3/mL
        0  0  0  1  -k(4)/mL          0      0   0   0 %k4/mL
        1  0  0  0      0         -k(1)/mB   0   0   0 %-k1/mB
        0 -1  0  0      0    -(dL-k(2))/mB   1   0   0 %dL/mB-k2/mB
        0  0 -1  0      0     (k(3)+kB)/mB   0  -1   0 %-k3/mB-kB/mB
        0  0  0 -1      0     (k(4)+dB)/mB   0   0  -1 ];%-k4/mB-dB/mB
    Beq = zeros(8,1);
    
    [Aeq, Beq] = mat_rdc(Aeq, Beq); % extract the independent rows from Aeq
        
    %preparing inequality constraints for QP solvers
    %        k1    k2  k3  k4    mL     mB    dL    kB    dB
    LB  = [-Inf -Inf -Inf -Inf  mLmin mBmin dLmin kBmin dBmin]';
    UB  = [ Inf  Inf  Inf  Inf  mLmax mBmax dLmax kBmax dBmax]';
    PK=[k(:);p]
    
    
    [tc,yc] = ode45(@LP_model_calc, [0, 100], [x0;zeros(4*4+4+1,1)]);
    H       =  R * reshape(yc(end, 4+1:4+16)',4,4);
    f       =  yc(end, 4+16+1:4+16+4)';
    
    
    options.LargeScale = 'off';  % setting opitons for the Q_Programming
    options.Display = 'off';     % get rid of those super annoying text
    
    PK = quadprog(blkdiag(H,zeros(5)),[f;zeros(5,1)],[],[], ...
        Aeq, Beq, LB, UB, PK, options);
    
    k =  PK(1:4)';
    
    [tcost,ycost] = ode45(@LP_model_calc,[0,100], [x0;zeros(4*4+4+1,1)]);
    J1 =  ycost(end,end);
    clear tcost ycost;
    
    %J2 = yc(end, 4+1:4+16) * Qk(:);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Update the co-design parameters
    
    mL = PK(5);
    mB = PK(6);
    dL = PK(7);
    kB = PK(8);
    dB = PK(9);
    
    J1_save = [J1_save J1];
    p      =   [mL; mB; dL; kB; dB]; % Design parameters
    
end %while

m_PI_cpu = toc()

%% -----------Print out a simulation summary-------------------------------
disp(['The initial cost is ', num2str(J1_save(1))])
disp(['The final   cost is ', num2str(J1_save(end))])
disp(['The LQR     cost is ', num2str(J2_save(end))])
disp(['The cost is reduced by ', ...
      num2str((J2_save(end)-J1_save(end))/J2_save(end)*100), ...
     '%, compared with standard LQR'])
%%-------------------------------------------------------------------------
%%
figure(1)
plot((1:iter),log10(J1_save(1:iter)),'r-o',(1:iter), log10(J2_save(1:iter)), 'b-^','Linewidth',2)
legend('Modified Policy Iteration', 'Convential Policy Iteration')
title('Convential vs. Modified policy iteration', 'Fontsize', 14)
% legend('J_{compac} (Plant redesign under system equivalence)',' J_a (Standard H2/H_{\infty} mixed control design)')
xlabel('Number of iterations', 'Fontsize', 14)
ylabel('Log_{10}(J)', 'Fontsize', 14)
% %%%%%%%%%%%%%%%%x%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Draw the co-designed system performance


figure(2)
subplot(121)
K=k;
[t,y]=ode45(@LP_model,[0,tf],x0);
plot(t0,y0(:,1)+1,'b-.',t,y(:,1)+1,'r-','Linewidth',2);%,t0,y0(:,2),t0,y0(:,3),t0,y0(:,4))
%plot(t,y(:,1)+1,'r-','Linewidth',2);%,t0,y0(:,2),t0,y0(:,3),t0,y0(:,4))
legend('Without co-design', 'With co-design')%,'x_2','x_3','x_4')
%title('Traking performance with respect to a step signal', 'Fontsize', 12)
xlabel('Time (sec)', 'Fontsize', 12)
ylabel('Load position xL', 'Fontsize', 12)
axis([0 tf 0 1.4])
subplot(122)
for i = 1:length(y)
 uwc(i) = -1/2 * inv(R) * B'* dPhi(y(i,1:4))' * w;
end
plot(t0,uwc0,'b-.',t,uwc,'r-','Linewidth',2)%+1,'b-.',t,y(:,1)+1,'r-','Linewidth',2)
legend('Without co-design', 'With co-design')%,'x_2','x_3','x_4')
xlabel('Time (sec)', 'Fontsize', 12)
ylabel('Control input u', 'Fontsize', 12)



%%
figure(3)
xx = -1:0.1:1;
yy = -1:0.1:1;
zz = zeros(length(xx),length(yy)); zz0 = zz;

for i = 1:length(xx)
 for j = 1:length(yy)   
  zz(i,j) = Phi([xx(i),0,yy(j),0])*w; 
  zz0(i,j) = Phi([xx(i),0,yy(j),0])*w0; 
 end
end

[xx,yy]=meshgrid(xx,yy);
surf(xx,yy,zz0')

xlabel('x1:=xL-yd','Fontsize',14)
ylabel('x3:=xB','Fontsize',14)
zlabel('V(x)','Fontsize',14)

hold on

surf(xx,yy,zz')

%axis([-1 1 -1 1 0 700])

hold off
title('Comparison of the value functions','Fontsize',14)


