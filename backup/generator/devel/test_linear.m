A=[0                     1'
   -b2*cos(a3)/2        -b1 ];
B=[0;b3];


r=1;

disp('1st')
lyap(A',eye(2)+[-1 0]*[-1 0]')

disp('optimal')
[K,S,E]=lqr(A,B,eye(2),r)