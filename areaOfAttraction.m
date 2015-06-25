%%

% The set for approximation is set as 
%\Omega = {x1,x2,x3,x4 | x1\in(-0.5, 0.5),
%                        x2\in(-5,5), 
%                        x3\in(-0.2,0.2),
%                        x4\in(-10,10)}

% We need to find D, by solving
%
% max D
% st. V(x)\le D => x\in\Omega
%
% One necessary condition is that if x in boundary of \Omega
% => V(x) >= D

xs = [];
% on boundary of x1

for x1 = [-0.5 0.5]/100;
    for x2 = (-5:dx2:5)/100
        for x3 = (-0.2:dx3:0.2)/100
            for x4 = (-10:dx4:10)/100
                xs = [xs;[x1 x2 x3 x4]]; 
                %xs = [xs; p'*Phi_fun([x1 x2 x3 x4])'];
            end
        end
    end        
end