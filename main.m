% Main Script

PsiPsi = [];
PsiU = [];
Phi = [];
CostQ = [];

PhiLength = numel(Phi_fun(zeros(1,4)));
PsiLength = numel(Psi_fun(zeros(1,4)));

r = 1;  % Weight on u

N = 100;
IterMax = 10;  % Number of iterations

X = [1,.3,1,-.2,zeros(1,PsiLength^2+PsiLength+1)];

T = 0.01; % Length for each time interval

% Data Collection
for i = 1:N	
	[t,X] = ode45(@adpSysWrapper, ...
		          [i-1,i]*T, ...
				  [X(end,1:4) zeros(1, PsiLength^2 + PsiLength + 1)]);
	Phi    = [Phi; 
		      Phi_fun(X(end,1:4))-Phi_fun(X(1,1:4))];        %#ok<AGROW>
	PsiPsi = [PsiPsi;
		      X(end,4 + (1:PsiLength^2))];                   %#ok<AGROW>
	PsiU   = [PsiU; 
		      X(end,4 + PsiLength^2 + (1:PsiLength))];       %#ok<AGROW>
	CostQ = [CostQ; 
		      X(end,end)];			                         %#ok<AGROW>
end

% Online Off-Policy Learning. Solve the matrix Ax = B 

w = zeros(PsiLength,1);
w(1) = 1;

for i = 1:1%IterMax
	A = [Phi -2*r*PsiU+2*r*PsiPsi*kron(w,ones(PsiLength))];
    rank(A)
	B = -(CostQ + PsiPsi*kron(w,w));
	pw = A\B;
  	p = pw(1:PhiLength)'
	w = pw(PhiLength+1:end);
end



