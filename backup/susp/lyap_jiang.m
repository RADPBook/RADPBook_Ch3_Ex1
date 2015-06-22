% lyap_jiang(A,Q)is expected to be the same function as lyap
% I wrote this function simple to avoid the lack of license.
% Basically,
%           A'P+PA+Q=0
%
% Using Kroneckor product, we have
%
%          [kron(I,A')+kron(A',I)]*vec(P) = -vec(Q)
%
% Hence, P can be solved from
%
%          vec(P) = - inv( [kron(I,A') + kron(A',I)] ) * vec(Q)
%


function P = lyap_jiang(A,Q)

[m,n] = size(A);

P     = - inv([kron(eye(n),A') + kron(A',eye(n))] ) * Q(:);

P     = reshape(P,m,n);

end