% function y=offline(w2o)
 % for i=1:10
   % w2o=offlinek(w2o)
 % end
% end
function y=offlinek(w1,w2)
 [X,Y]=meshgrid(-1:.1:1,-5:1:5);
 for i=1:21
  for j=1:11
   e(j,i)=offlinekernel(X(j,i),Y(j,i),w1,w2);
   end
 end
 figure(2)
 surf(X,Y,e) 
 size(X)
 size(Y)
 size(e)
 end
 
 
function e=offlinekernel(x1,x2,w1,w2)
parameters
f=[x2;-b1*x2-b2*cos(x1/2+a3)*sin(x1/2)];
g=[0;b3];
Vf=[w1'*Phi21(x1,x2)*f(1)-2*r/g(2)*w2'*Phi2(x1,x2)*f(2)]
size(w2)
Vg2=(w2'*Phi2(x1,x2))^2;
e=Vf+Q(x1,x2)-Vg2;
end