function [xn,F] = ImpactMap_five_link_walker(x)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

q = x(1:5);
dq = x(6:10);
N = length(q);
R = [1 1 1 -1 -1; 0 0 0 0 1; 0 0 0 1 0; 0 0 1 0 0; 0 1 0 0 0];


Je = Je_matrix(q);
De = De_matrix(q);


A = [De -Je'; Je zeros(2)];
b = [De*[dq;zeros(2,1)]; zeros(2,1)];
y = A\b;
        
F = y((N+1):end);

dqn = R*y(1:N);
qn = R*q;

xn = [qn;dqn];
end

