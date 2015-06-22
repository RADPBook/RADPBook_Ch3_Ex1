%function dx=LP_model(t,x)
function dx=LP_model(t,x,A,B,kBndmB,R,k,w)
% global K dL dB mL mB kB w R
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

dx       =  A * x + [0;kBndmB*x(3)^3;0;-kBndmB*x(3)^3]+ B * (ul + unl);
end