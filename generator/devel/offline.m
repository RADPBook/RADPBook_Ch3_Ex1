function y=offline(w2o)
 for i=1:10
   w2o=offlinek(w2o)
 end
end





function y=offlinek(w2o)


Vfsave=[];
Bsave=[];
for x1=-1:0.1:1
 for x2=-5:1:5
  [Vf B]=offlinekernel(x1,x2,w2o);
  Vfsave=[Vfsave;Vf'];
  Bsave=[Bsave;B];
  end
end
size(Vfsave)
size(Bsave)
w2o=inv(Vfsave'*Vfsave)*Vfsave'*Bsave;
y=w2o;

end



function [Vf B]=offlinekernel(x1,x2,w2)
parameters
f=[x2;-b1*x2-b2*cos(x1/2+a3)*sin(x1/2)];
g=[0;b3];
Vf=-2/g(2)*[Phi2(x1,x2)*f(1);Phi2(x1,x2)*f(2)];
%size(w2)
Vg2=(w2(N2+1:end)'*Phi2(x1,x2))^2;
B=Vg2-Q(x1,x2);
end