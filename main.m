% Main Script

PhiPhi = [];
PhiU = [];
CostQ = [];

X = zeros(1,4+24^2+24+1);
[t,X] = ode45(@adpSysWrapper, [0,1], X(end,:));
PhiPhi = [PhiPhi;X(end,4 + (1:24^2))];
PhiU = [PhiU; X(end,4 + 24^2 + (1:24))];
CostQ = [CostQ; X(end,end)];
