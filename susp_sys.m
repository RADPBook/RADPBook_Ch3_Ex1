function dx = susp_sys(~,x)
% Dynamics of the syspension system
x1 = x(1);
x2 = x(2);
x3 = x(3);
x4 = x(4);


% Coefficients
mb = 300;    % kg
mw = 60;     % kg
bs = 1000;   % N/m/s
ks = 16000 ; % N/m
kt = 190000; % N/m


% combine the output
dx = [x1;x2;x3;x4];
end