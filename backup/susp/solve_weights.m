%%
% (\partial phi_1/ \partial x)
% ...
% (\partial phi_q/ \partial x)
%
P = lyap_jiang(A-B*[1 0 0 0],Q+[1 0 0 0]'*R*[1 0 0 0]);
w = [P(1,1) P(2,2) P(3,3) P(4,4) P(1,2)*2 P(1,3)*2 P(1,4)*2 P(2,3)*2 P(2,4)*2 P(3,4)*2]';


for i=1:10
X = [];
Y = [];
    for x1 = -1:.4:1
        for x2 = -2:.8:2
            for x3 = -1:.4:1
                for x4 = -1:0.4:1
                    dP = dPhi([x1 x2 x3 x4]);
                    
                    u = - 1/2 * inv(R) * B' * dP' * w;
                    
                    %u = [-1 0 0 0]*[x1 x2 x3 x4]';
                    
                    X = [X;
                        (dP*(A*[x1;x2;x3;x4]+B*u))'];
                    
                    Y = [Y; 1000*x1*x1 + u' * R * u];
                    
                end
            end
        end
    end
    i
    w = inv(X'*X)*X'*(-Y) % update the weights
    K = - 1/2 * inv(R) * [B(2)*w(5)+B(4)*w(7) 2*B(2)*w(2)+B(4)*w(9) B(2)*w(8)+B(4)*w(10) 2*B(4)*w(4)+B(2)*w(9)]
    % extract the linear portion of the  feedback gains
end