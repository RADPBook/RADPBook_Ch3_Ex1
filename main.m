% Main Script

PsiPsi = [];
PsiU = [];
Phi = [];
CostQ = [];
x_save = []; % Keep track of the states
t_save = []; % Keep track of the time

currentTime = 0; % Indicating the current time throught the simulation

PhiLength = numel(Phi_fun(zeros(1,4)));
PsiLength = numel(Psi_fun(zeros(1,4)));

r = 1;  % Weight on u

N = 100;
IterMax = 10;  % Number of iterations

X = [1,.3,1,-.2,zeros(1,PsiLength^2+PsiLength+1)];

T = 0.01; % Length for each time interval

%% Online Data Collection
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
    t_save = [t_save; 
              t(:)];                                         %#ok<AGROW>
    x_save = [x_save; 
              X(:,1:4)];                                     %#ok<AGROW>
    currentTime = t_save(end);
end

% Off-Policy Learning. Solve the matrix A*pw = B 

w = zeros(PsiLength,1);

for i = 1:IterMax
	A = [Phi -2*r*PsiU+2*r*PsiPsi*kron(w,eye(PsiLength))];
    B = -(CostQ + PsiPsi*kron(w,w));
	pw = A\B;
  	p = pw(1:PhiLength);
	w = pw(PhiLength+1:end);
end


%% Terminate exploration noise but keep appying the inital gains


%% Post learning phase

currentStates = x_save(end,:);
[t,y] = ode45(@(t,x) simpleSysWrapper(t,x,w), ...
              currentTime+[0,5], ...
              currentStates);
t_save = [t_save
          t(:)];
x_save = [x_save
          y(:,1:4)];

[t0,y0] = ode45(@(t,x) simpleSysWrapper(t,x,w*0), ...
                                        currentTime+[0,5], ...
                                        currentStates);
%%
plot(t_save,x_save(:,1),t0, y0(:,1))
%legend('learned', 'unlearned')

%%
%plot(tt,yy(:,2),tt0,yy0(:,2))
%legend('learned', 'unlearned')


