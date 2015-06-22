
PhiPhi = [];
PhiU = [];

X = zeros(1,425);
[t,X] = ode45(@adpSysWrapper, [0,1], X(end,:));
PhiPhi = [PhiPhi;
	      ]
PhiU = [PhiU; 
