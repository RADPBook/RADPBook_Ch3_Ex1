function dX=LP_model_calc(t,X,A,B,kBndmB,Q,R,k,w)
%global k dL dB mL mB kB R w
x       = X(1:4);
Hv      = X(4+1:4+16);
% A       = [ 0 1             0       0;
%             0 -s.dL/s.mL-s.dL/s.mB   s.kB/s.mB   s.dB/s.mB;
%             0 0             0       1;
%             0 s.dL/s.mB        -s.kB/s.mB  -s.dB/s.mB];
% B       = [ 0;
%             1/s.mL+1/s.mB;
%             0
%             -1/s.mB];
        
ul       =  k * (x(1:4)); % linear portion of the controller

unl      =  -1/2*inv(R) * B' *dPhi(x)' * [zeros(10,1); w(11:end)] ;

dx      =  A * x + [0;kBndmB*x(3)^3;0;-kBndmB*x(3)^3]+ B * (ul + unl);

dH      =  x * x';

dHv     =  dH(:);

df      =  x * R * unl;

dJ      =  x'*Q*x + (ul+unl) * R *(ul+unl);

dX      = [dx ; dHv ; df ; dJ];

end