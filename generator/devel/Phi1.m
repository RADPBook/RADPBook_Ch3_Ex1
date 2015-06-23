function y=Phi1(x1,x2)
parameters
 y=[
     x1.^2
     x2.^2
     x1.*x2
     x1.*x1.*x1
     x2.*x2.*x2
     x1.*x1.*x2
     x1.*x2.*x2
     x1.*x1.*x1.*x1
     x2.*x2.*x2.*x2
     x1.*x1.*x1.*x2
     x2.*x2.*x2.*x1
     x1.*x1.*x2.*x2
%      cos(x1/2)*x2;
%      cos(x1)*x2;
%      sin(x1/2)*cos(a3 + x1/2)
	];
end