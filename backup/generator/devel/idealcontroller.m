disp('Plotting the controller...')
% n=input('input scale (integer)');
% clear Z X Y Zi
% [X,Y]=meshgrid(-5:10/n:5,-20:20/n:20);
% [ii,jj]=size(X);
% Z=X;



%% Calculate the ideal controller

syms x1 x2 z
%the first virtual control input
r=1;

u1=1/2*(rho0(x1^2+x2^2)^2+2*rho0(x1^2+x2^2)+2)*w2'*Phi2(x1,x2);


% u1=-1/4*(rho0(phi^2)^2+2/r*rho0(phi^2)+2/r^2)*(-5*phi)
zeta=z-u1
f1=-[diff(u1,x1) diff(u1,x2)]*[x2; -b1*x2-b2*cos(x1/2+a3)*sin(x1/2)+b3*z]-c1*z-c2*x2;

%*(-psi-3/2*phi^2-1/2*phi^3)
g1=-diff(u1,x2)*(b3)
%u=-f1+5*phi-0.1*zeta ...
%   -zeta*(g1)^2*rho((phi^2+zeta^2)/2)^2 ...
%   -zeta/4*rho((phi^2+zeta^2)/2)^2 ...
%   -0.1*zeta/2*(rho(zeta^2/2)/rho((phi^2/2)))^2;

%% 
% for i=1:ii
    % for j=1:jj
        % phi=X(i,j);
        % psi=Y(i,j);
        % Z(i,j)=ADPcontroller(phi,psi);
        % Zi(i,j)=eval(u);
    % end
% end
%
% figure(3)
% surf(X,Y,Z)
% xlabel('\phi')
% ylabel('\psi')
% zlabel('u')
% title('RADP Controller')

% figure(4)
% surf(X,Y,Z)
% xlabel('\phi')
% ylabel('\psi')
% zlabel('u')
% title('Ideal Controller')

% figure(5)
% surf(X,Y,Z-Zi)
% xlabel('\phi')
% ylabel('\psi')
% zlabel('u-u^*')
% title('Error')

