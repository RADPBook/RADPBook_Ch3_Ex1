function [An,Bn]=mat_rdc(A,B)
n=length(B);
i=0;
An=[];
Bn=[];
i=0;

while i < n
    i = i+1;
    if rank([An; A(i,:)])>rank(An)
        An = [An; A(i,:)];
        Bn = [Bn; B(i,:)];
    end
    
end
end