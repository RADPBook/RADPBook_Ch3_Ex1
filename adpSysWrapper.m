function dX = adpSysWrapper(t,X)
 
% System dynamics part
x = X(1:4);
u = sin(t);

dx = susp_sys(x,u);       % dx as the first 1-4 states of the wrapper

% Augmented part

psi = Psi(x);

PsiPsi = kron(psi,psi);   % Export \psi\otimes\psi as the 4+[1,24^2] states
Psiu   = psi*u;           % Export \psi*u as the 4+[1,24^2]+ [1,24] states
Q = x'*x;                 % Export q(x) as the last state

dX = [dx; 
	  PsiPsi; 
	  Psiu;
	  Q];
end