function y=Phi3(a,b,c)
y=[%
   a
   b
   c
   a*b
   a*c
   b*c
   a*b*c
   % sin(a)
   %a*sin(a/2)^2
  % a*sin(a/2)^2
  % b^2   
   %sin(a/2)^2
%    a^2*sin(a/2)^2
%    a^3*sin(a/2)^2
%    b^2*sin(a/2)^2
%    b^4*sin(a/2)^2
%    a^2*c;
%    a^3*c;
%    b^2*c;
%    a*sin(a/2)^2
%    a*b^2*c
%    a*b*sin(a/2)^2
%    a*b^2*sin(a/2)^2
%    a*b^3*sin(a/2)^2
%    a^3*b*sin(a/2)^2
%   a*b*c
     %a*sin(a);
%    a^2*b^2*sin(a/2)^2
%    a^3*b^2*sin(a/2)^2
];
end