%% five link model
clear;clc;close all;
syms qsf qsk qsh qnsh qnsk dqsf dqsk dqsh dqnsh dqnsk 'real'

g=9.81; 

%taken from visual -----------------------
lf = 0.4064; 
lT = 0.4999; 
lt = 1.373-lT-lf; %3Mpaper TABLE I height
pMT = 0.28507; 
%pMT = 0.28507;  %result in com-y 0.9518
%pMT = lT-0.28507; %result in com-y 0.9094
pMf =0.1494; pMt = 0.171885;
%-----------------------------------------
MT = 13; Mf = 3.4261; Mt = 0.85216;
IT = 0.1940; If = 0.0389; It = 0.019286025; 

MTerr = 13;Mferr =Mf; Mterr = Mt;
ITerr = 0.5940; Iferr = 0.3389; Iterr = 0.219286025; 

qs = [qsf,qsk,qsh,qnsh,qnsk]';
dqs = [dqsf,dqsk,dqsh,dqnsh,dqnsk]';  
x = [qs;dqs];
theta = [qsf qsf+qsk qsf+qsk+qsh pi+qsf+qsk+qsh-qnsh pi+qsf+qsk+qsh-qnsh-qnsk]';

N = 5;
%% nominal
pknee1 = Rot(theta(1)+pi/2)*[lt;0];
phip = pknee1 + Rot(theta(2)+pi/2)*[lf;0];
pknee2 = phip + Rot(theta(4)+pi/2)*[lf;0];
pfoot2 = pknee2 + Rot(theta(5)+pi/2)*[lt;0];

pcm1 = Rot(theta(1)+pi/2)*[lt-pMt;0];
pcm2 = pknee1 + Rot(theta(2)+pi/2)*[lf-pMf;0];
pcm3 = phip + Rot(theta(3)+pi/2)*[pMT;0];
pcm4 = phip + Rot(theta(4)+pi/2)*[pMf;0];
pcm5 = pknee2 + Rot(theta(5)+pi/2)*[pMt;0];
pcm = (Mt*pcm1+Mf*pcm2+MT*pcm3+Mf*pcm4+Mt*pcm5)/(Mf*2+Mt*2+MT);
% double(subs(pcm,qs,[0 0 0 0 0]'))

%nominal
p = [pcm1;pcm2;pcm3;pcm4;pcm5];
Dm = Mt*jacobian(pcm1,qs)'*jacobian(pcm1,qs) + Mf*jacobian(pcm2,qs)'*jacobian(pcm2,qs) +MT*jacobian(pcm3,qs)'*jacobian(pcm3,qs) + Mf*jacobian(pcm4,qs)'*jacobian(pcm4,qs)+ Mt*jacobian(pcm5,qs)'*jacobian(pcm5,qs);
DJ = It*jacobian(theta(1),qs)'*jacobian(theta(1),qs) + If*jacobian(theta(2),qs)'*jacobian(theta(2),qs) + IT*jacobian(theta(3),qs)'*jacobian(theta(3),qs) + If*jacobian(theta(4),qs)'*jacobian(theta(4),qs) + It*jacobian(theta(5),qs)'*jacobian(theta(5),qs) ;
Drobot = simplify(Dm+DJ);
P = [0 Mt 0 Mf 0 MT 0 Mf 0 Mt]*g*p;


% 
% %motor inertia
% ng = 91.4286;
% Ng = diag([1 ng ng ng ng]);
% Ia = 0.000051;
% Nua = 1;
% Ifull = Ia*eye(N); Ifull(Nua,Nua)=0;
% D = Drobot+Ng*Ifull*Ng;
D = Drobot;

for j = 1:N
    for k = 1:N
        for i = 1:N
            Ctemp(i) = (diff(Drobot(k,j),qs(i))+diff(Drobot(k,i),qs(j))-diff(Drobot(i,j),qs(k)))*dqs(i);
        end
        C(k,j) = 1/2*sum(Ctemp);
    end
end
C = simplify(C);
G = simplify(jacobian(P,qs)');
%% with error


p = [pcm1;pcm2;pcm3;pcm4;pcm5];
Dm = Mterr*jacobian(pcm1,qs)'*jacobian(pcm1,qs) + Mferr*jacobian(pcm2,qs)'*jacobian(pcm2,qs) +MTerr*jacobian(pcm3,qs)'*jacobian(pcm3,qs) + Mferr*jacobian(pcm4,qs)'*jacobian(pcm4,qs)+ Mterr*jacobian(pcm5,qs)'*jacobian(pcm5,qs);
DJ = Iterr*jacobian(theta(1),qs)'*jacobian(theta(1),qs) + Iferr*jacobian(theta(2),qs)'*jacobian(theta(2),qs) + ITerr*jacobian(theta(3),qs)'*jacobian(theta(3),qs) + Iferr*jacobian(theta(4),qs)'*jacobian(theta(4),qs) + Iterr*jacobian(theta(5),qs)'*jacobian(theta(5),qs) ;
Droboterr = simplify(Dm+DJ);
P = [0 Mterr 0 Mferr 0 MTerr 0 Mferr 0 Mterr]*g*p;


% 
% %motor inertia
% ng = 91.4286;
% Ng = diag([1 ng ng ng ng]);
% Ia = 0.000051;
% Nua = 1;
% Ifull = Ia*eye(N); Ifull(Nua,Nua)=0;
% D = Drobot+Ng*Ifull*Ng;
Derr = Droboterr;

for j = 1:N
    for k = 1:N
        for i = 1:N
            Ctemp(i) = (diff(Droboterr(k,j),qs(i))+diff(Droboterr(k,i),qs(j))-diff(Droboterr(i,j),qs(k)))*dqs(i);
        end
        C(k,j) = 1/2*sum(Ctemp);
    end
end
Cerr = simplify(C);
Gerr = simplify(jacobian(P,qs)');

%%


% consider residual 
B = [zeros(1,N-1);eye(N-1)];

fx = [dqs; -inv(D)*(C*dqs+G)];
gx = [zeros(N,N-1); inv(D)*B];
fxerr = [dqs; -inv(Derr)*(Cerr*dqs+Gerr)];
gxerr = [zeros(N,N-1); inv(Derr)*B];


syms ldes r m1 a 'real'
syms c [1 5] 'real'
syms thetamp [1 2] 'real'
%r: radius of the stone
%m: order changes shape
%ldes: distance from stance foot to desired foot strike location
O = [ldes; 0];
pswing = pfoot2; 
Ox = ldes;
Fx = pswing(1);
theta = c*qs;
tau = (theta-thetamp(2))/(thetamp(1)-thetamp(2));
k = (1-a*r)/(a*r);
Rtau = -k./(k+exp(-m1*(tau-1)))+1+r;
s1 = Rtau-(Ox-Fx);
Js1q = jacobian(s1,qs);
Lfs1 = jacobian(s1,x)*fx;
s1e = 80*s1 + Lfs1;
Lfs1e = jacobian(s1e,x)*fx;
Lfs1eerr = jacobian(s1e,x)*fxerr;
Lgs1e = jacobian(s1e,x)*gx;
Lgs1eerr = jacobian(s1e,x)*gxerr;

err_Lfs1e = Lfs1e -Lfs1eerr;
err_Lgs1e = Lgs1e -Lgs1eerr;
%%
matlabFunction(err_Lfs1e ,'File','safety_funcs_tau/CBF_err_Lfs1e.m','Vars',{x,c,thetamp,ldes,r,m1 ,a})
matlabFunction(err_Lgs1e,'File','safety_funcs_tau/CBF_err_Lgs1e.m','Vars',{x,c,thetamp,ldes,r,m1 ,a})