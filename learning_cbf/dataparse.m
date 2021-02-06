%% setup
clear;clc;close all;

addpath(fullfile(pwd, 'dynamics/'));
taubasedCBF= 1;
if taubasedCBF
    addpath(fullfile(pwd, 'safety_funcs_tau/'));
else
    addpath(fullfile(pwd, 'safety_funcs/'));
end
addpath(fullfile(pwd, 'gaits/'));
addpath(fullfile(pwd, 'controllers/'));
addpath(fullfile(pwd, 'experiments/'));
addpath(fullfile(pwd, 'basic/'));
addpath(fullfile(pwd, 'sim/'));
odeoptions = odeset('MaxStep', 1e-2,'RelTol',1e-5,'AbsTol',1e-5,'Event',@switchsurface);
gait_num = '0.67';
gaitpath = strcat('gaits/',gait_num,'/');
list = dir(fullfile(pwd,gaitpath));
gaitname = list(4).name;
PARAM = YAML.read(strcat(gaitpath,gaitname));

% PARAM = YAML.read(fullfile(pwd,'gaits/params_2016-09-07T21-35-04-00.yaml'));
thetamp = PARAM.domain.p;
alphas = PARAM.domain.a;
%%

clear ind err errdata errLf errLg
filename = '../data/SimPullIn1.csv';
% Load raw data file
A = readmatrix([ filename]);

ind = 1;
t = (A(:,ind)-A(1,ind));        ind=ind+1; % Sample time
tau =  A(:,ind);                ind=ind+1;
switchBack =  A(:,ind);         ind=ind+1; 
tau_d =  A(:,ind);              ind=ind+1; % State variable used by controller
tau_a =  A(:,ind);              ind=ind+1; % Measured state variable
stance = A(:,ind);              ind=ind+1; % Stance leg (1 for right)
a_pos = A(:,ind:ind+4);         ind=ind+5; % Meas. pos. [Torso, LK, LH, RH, RK]
a_vel = A(:,ind:ind+4);         ind=ind+5; % Meas. vel.
d_pos = A(:,ind:ind+3);         ind=ind+4; % Desired pos.
d_vel = A(:,ind:ind+3);         ind=ind+4; % Desired vel.
torque = A(:,ind:ind+3);        ind=ind+4; % Torque [Nm]
udes = A(:,ind:ind+3);          ind=ind+4; % Torque [Nm]
foot_pos = A(:,ind:ind+1);      ind=ind+2; % Barrier function
hip_pos = A(:,ind:ind+1);       ind=ind+2; % Barrier function 
h = A(:,ind:ind+2);             ind=ind+3; % Barrier function
he = A(:,ind:ind+2);            ind=ind+3; % Barrier function
Lfh = A(:,ind:ind+2);           ind=ind+3; % Barrier function
Lf2h = A(:,ind:ind+2);          ind=ind+3; % Barrier function
LgLfh{1} = A(:,ind:ind+3);      ind=ind+4; % Barrier function
LgLfh{2} = A(:,ind:ind+3);      ind=ind+4; % Barrier function
LgLfh{3} = A(:,ind:ind+3);      ind=ind+4; % Barrier function
actS1 = A(:,ind:ind+3);         ind=ind+4; % Barrier function
actS2 = A(:,ind:ind+3);         ind=ind+4; % Barrier function
driftS1 = A(:,ind);             ind=ind+1;
driftS2 = A(:,ind);             ind=ind+1;



a = .3;
r = .05/(1+a);
m1 = 22;
alpha = 150;
ldes=.4;

lf = 0.4064;
lT = 0.4999;
lt = 1.373-lT-lf;
c = [-lt-lf -lf 0 0 0];
a_pos_switched = [];
a_vel_switched = [];
torque_switched = [];

for i = 1:length(t)/10
    q = a_pos(i,:)';
    dq = a_vel(i,:)';
    u = torque(i,:)';
    if stance(i)==1 %change from L/R to stance/non-stance
        q = [q(1) q(5) q(4) q(3) q(2)]';
        dq = [dq(1) dq(5) dq(4) dq(3) dq(2)]';
        u = [u(4) u(3) u(2) u(1)]';
    end
    %qsf = qTor-qsk-qsh
    q(1) = q(1)-q(2)-q(3);
    dq(1) = dq(1)-dq(2)-dq(3);
    a_pos_switched = [a_pos_switched, q];
    a_vel_switched = [a_vel_switched, dq];
    torque_switched = [torque_switched, u];
    theta = c*q;
    x = [q;dq];
    xplot(i,:)=x;
    tau(i) = (theta-thetamp(2))/(thetamp(1)-thetamp(2));
    errLfs1(i) = CBF_err_Lfs1e(x,c,thetamp,ldes,r,m1,a);
    errLgs1(i,:) = CBF_err_Lgs1e(x,c,thetamp,ldes,r,m1,a);
    errs1(i) = CBF_err_Lfs1e(x,c,thetamp,ldes,r,m1,a)+CBF_err_Lgs1e(x,c,thetamp,ldes,r,m1,a)*u;
    errdatas1(i) = driftS1(i)+actS1(i,:)*u;
    
     errLfs2(i) = CBF_err_Lfs2e(x,c,thetamp,ldes,r,m1,a);
     errLgs2(i,:) = CBF_err_Lgs2e(x,c,thetamp,ldes,r,m1,a);
     errs2(i) = CBF_err_Lfs2e(x,c,thetamp,ldes,r,m1,a)+CBF_err_Lgs2e(x,c,thetamp,ldes,r,m1,a)*u;
     errdatas2(i) = driftS2(i)+actS2(i,:)*u;
    
end
ind = find(stance==0);

a_pos = a_pos_switched; 
a_vel = a_vel_switched;
torque = torque_switched;


%%

clear ind
filename = '../data/SimPullIn2.csv';
% Load raw data file
A = readmatrix([ filename]);

ind = 1;
twot = (A(:,ind)-A(1,ind));        ind=ind+1; % Sample time
twotau =  A(:,ind);                ind=ind+1;
switchBack =  A(:,ind);         ind=ind+1; 
tau_d =  A(:,ind);              ind=ind+1; % State variable used by controller
tau_a =  A(:,ind);              ind=ind+1; % Measured state variable
stance = A(:,ind);              ind=ind+1; % Stance leg (1 for right)
a_pos = A(:,ind:ind+4);         ind=ind+5; % Meas. pos. [Torso, LK, LH, RH, RK]
a_vel = A(:,ind:ind+4);         ind=ind+5; % Meas. vel.
d_pos = A(:,ind:ind+3);         ind=ind+4; % Desired pos.
d_vel = A(:,ind:ind+3);         ind=ind+4; % Desired vel.
torque = A(:,ind:ind+3);        ind=ind+4; % Torque [Nm]
udes = A(:,ind:ind+3);          ind=ind+4; % Torque [Nm]
foot_pos = A(:,ind:ind+1);      ind=ind+2; % Barrier function
hip_pos = A(:,ind:ind+1);       ind=ind+2; % Barrier function 
h = A(:,ind:ind+2);             ind=ind+3; % Barrier function
he = A(:,ind:ind+2);            ind=ind+3; % Barrier function
Lfh = A(:,ind:ind+2);           ind=ind+3; % Barrier function
Lf2h = A(:,ind:ind+2);          ind=ind+3; % Barrier function
LgLfh{1} = A(:,ind:ind+3);      ind=ind+4; % Barrier function
LgLfh{2} = A(:,ind:ind+3);      ind=ind+4; % Barrier function
LgLfh{3} = A(:,ind:ind+3);      ind=ind+4; % Barrier function
actS1 = A(:,ind:ind+3);         ind=ind+4; % Barrier function
actS2 = A(:,ind:ind+3);         ind=ind+4; % Barrier function
twodriftS1 = A(:,ind);             ind=ind+1;
twodriftS2 = A(:,ind);             ind=ind+1;



a = .3;
r = .02/(1+a);
m1 = 22;
alpha = 150;
ldes=.4;

lf = 0.4064;
lT = 0.4999;
lt = 1.373-lT-lf;
c = [-lt-lf -lf 0 0 0];
a_pos_switched = [];
a_vel_switched = [];
torque_switched = [];

for i = 1:length(twot)/10
    length(twot)
    i
    q = a_pos(i,:)';
    dq = a_vel(i,:)';
    u = torque(i,:)';
    if stance(i)==1 %change from L/R to stance/non-stance
        q = [q(1) q(5) q(4) q(3) q(2)]';
        dq = [dq(1) dq(5) dq(4) dq(3) dq(2)]';
        u = [u(4) u(3) u(2) u(1)]';
    end
    %qsf = qTor-qsk-qsh
    q(1) = q(1)-q(2)-q(3);
    dq(1) = dq(1)-dq(2)-dq(3);
    a_pos_switched = [a_pos_switched, q];
    a_vel_switched = [a_vel_switched, dq];
    torque_switched = [torque_switched, u];
    theta = c*q;
    x = [q;dq];
    xplot(i,:)=x;
    tau(i) = (theta-thetamp(2))/(thetamp(1)-thetamp(2));
    twoerrs1(i) = CBF_err_Lfs1e(x,c,thetamp,ldes,r,m1,a)+CBF_err_Lgs1e(x,c,thetamp,ldes,r,m1,a)*u;
    Lgherr(i,:) = CBF_err_Lgs1e(x,c,thetamp,ldes,r,m1,a);

    twoerrdatas1(i) = driftS1(i)+actS1(i,:)*u;
    
    twoerrs2(i) = CBF_err_Lfs2e(x,c,thetamp,ldes,r,m1,a)+CBF_err_Lgs2e(x,c,thetamp,ldes,r,m1,a)*u;
    twoerrdatas2(i) = driftS2(i)+actS2(i,:)*u;
    
end
ind = find(stance==0);

a_pos = a_pos_switched; 
a_vel = a_vel_switched;
torque = torque_switched;

%% tau yaml
figure
subplot(2,1,1)
plot(tau,-errLfs1,'b.',tau,driftS1,'r.')
legend('true', 'model')
subplot(2,1,2)
plot(tau,-errLgs1,'b.',tau,actS1,'r.')
xlabel(['\tau  from ' gait_num '.yaml'])
sgtitle('S1 act drift comparisons: true (blue), model (red)')

figure
subplot(2,1,1)
plot(tau,-errLfs2,'b.',tau,driftS2,'r.')
legend('true', 'model')
subplot(2,1,2)
plot(tau,-errLgs2,'b.',tau,actS2,'r.')
xlabel(['\tau  from ' gait_num '.yaml'])
sgtitle('S2 act drift comparisons: model (blue), true (red)')

%% tau a
figure
subplot(2,1,1)
plot(tau_a,-errLf,'.',tau_a,driftS1,'.')
title('Lfh_e error')

subplot(2,1,2)
plot(tau_a,-errLg,'b.',tau_a,actS1,'r.')
title('Lgh_e error')

xlabel('\tau_a ')

%% This onE!!!!!!!!!!!!!!!!!!R ONGsdofinso dfn;OGN
figure
subplot(2,1,1)
hold on 
last = 10000;
plot(t(1:last),driftS1(1:last) + errs1(1:last)','r.')
%plot(t(1:length(twoerrs1)),-errs1(1:length(twoerrs1)),'b.',t(1:length(twoerrs1)),driftS1(1:length(twoerrs1)) + errs1(1:length(twoerrs1)),'r.')
%plot(twot(1:length(twoerrs1)),-twoerrs1(1:length(twoerrs1)),'k.',twot(1:length(twoerrs1)),twodriftS1(1:length(twoerrs1)),'g.')

% hold on
title('delta 1 vs true residual')
legend('true ep1', 'delta1 ep1', 'true ep2', 'delta1 ep2')

% % plot(t,stance*100,'.')
% % hold on
% % plot(t,foot_pos*100,'.')
% subplot(2,1,2)
% hold on 
% %plot(t(1:length(twoerrs1)),-errs2(1:length(twoerrs1)),'b.',t(1:length(twoerrs1)),driftS2(1:length(twoerrs1)),'r.')
% plot(twot(1:length(twoerrs1)),-twoerrs2(1:length(twoerrs1)),'k.',twot(1:length(twoerrs1)),twodriftS2(1:length(twoerrs1)),'g.')
% xlabel('t')
% title('delta 2  vs true residual')
% legend('true ep1', 'delta2 ep1', 'true ep2', 'delta2 ep2')

%% switching
figure
plot(t,stance,'.')
hold on
plot(t,foot_pos(:,2),'.')
hold on
plot(t,tau_a,'.')
legend('stance','foot_z','tau_a')
%%
figure
plot(t,-err,'.',t,errdata,'.')
legend('true error','from data')
xlabel('sample')
ylabel('residual')
title('total residual error')

%%
% figure
% plot(actS1+errLg)
% title('plus)')
% actS1 = readmatrix([ 'network_parameters/act_predictions1.csv']);

atilde_s1 = [];
atilde_s2 = [];

norms_proj = [];
norms_std=[];
norms_orth = [];
orth = []; 
for i = 1:length(actS1)
    i
    u = torque(:,i); 
    atilde_s1 = [ atilde_s1, norm((errLgs1(i,:))*u/(u'*u)*u - actS1(i,:)')];
    atilde_s2 = [ atilde_s2, norm((errLgs2(i,:))*u/(u'*u)*u - actS2(i,:)')];

    %orth = [orth,  (actS1(i,:)+errLgs1(i,:))' - (actS1(i,:)+errLgs1(i,:))*u/(u'*u)*u];
    %norms_proj = [norms_proj, norm( (actS1(i,:)+errLgs1(i,:))*u/(u'*u)*u)];
    %norms_std =[norms_std, norm((actS1(i,:)+errLgs1(i,:))')];
    %norms_orth =[norms_orth, norm((actS1(i,:)+errLgs1(i,:))' - (actS1(i,:)+errLgs1(i,:))*u/(u'*u)*u)]; 
end

%%
err = (actS1+errLgs1);
figure 
subplot(2,2,1)
plot(atilde(1,:)', 'r', 'linewidth', 2)
hold on 
plot(err(:,1), 'b', 'linewidth', 2)
plot(orth(1,:)', 'g', 'linewidth', 2)

subplot(2,2,2)
plot(atilde(2,:)', 'r', 'linewidth', 2)
hold on 
plot(err(:,2), 'b', 'linewidth', 2)
plot(orth(2,:)', 'g', 'linewidth', 2)
legend('proj_u(atilde)', 'atilde', 'atilde -proj_u(atilde)' )

subplot(2,2,3)
plot(atilde(3,:)', 'r', 'linewidth', 2)
hold on 
plot(err(:,3), 'b', 'linewidth', 2)
plot(orth(3,:)', 'g', 'linewidth', 2)

subplot(2,2,4)
plot(atilde(4,:)', 'r', 'linewidth', 2)
hold on 
plot(err(:,4), 'b', 'linewidth', 2)
sgtitle('elements')
plot(orth(4,:)', 'g', 'linewidth', 2)


figure
hold on 
plot(norms_proj, 'r', 'linewidth', 2)
plot(norms_std, 'b', 'linewidth', 2)
plot(norms_orth, 'g', 'linewidth', 2)
plot(stance*10, 'k', 'linewidth', 2)
legend('proj_u(atilde)', 'atilde', 'atilde -proj_u(atilde)', 'stance' )

title('norms: atilde = act model - act true')


%%
% figure
% plot(actS1+errLg)
% title('plus)')
% actS1 = readmatrix([ 'network_parameters/act_predictions1.csv']);

atilde = [];
norms_proj = [];
norms_std=[];
norms_orth = [];
orth = []; 
for i = 1:length(actS2)
    i
    u = torque(:,i); 
    atilde = [ atilde, (actS2(i,:)+errLgs2(i,:))*u/(u'*u)*u];
    orth = [orth,  (actS2(i,:)+errLgs2(i,:))' - (actS2(i,:)+errLgs2(i,:))*u/(u'*u)*u];
    norms_proj = [norms_proj, norm( (actS2(i,:)+errLgs2(i,:))*u/(u'*u)*u)];
    norms_std =[norms_std, norm((actS2(i,:)+errLgs2(i,:))')];
    norms_orth =[norms_orth, norm((actS2(i,:)+errLgs2(i,:))' - (actS2(i,:)+errLgs2(i,:))*u/(u'*u)*u)]; 
end

%%
err = (actS2+errLgs2);
figure 
subplot(2,2,1)
plot(tau_a, atilde(1,:)', 'r.', 'linewidth', 2)
hold on 
plot(tau_a,err(:,1), 'b.', 'linewidth', 2)
plot(tau_a,orth(1,:)', 'g.', 'linewidth', 2)

subplot(2,2,2)
plot(tau_a,atilde(2,:)', 'r.', 'linewidth', 2)
hold on 
plot(tau_a,err(:,2), 'b.', 'linewidth', 2)
plot(tau_a,orth(2,:)', 'g.', 'linewidth', 2)
legend('proj_u(atilde)', 'atilde', 'atilde -proj_u(atilde)' )

subplot(2,2,3)
plot(tau_a,atilde(3,:)', 'r.', 'linewidth', 2)
hold on 
plot(tau_a,err(:,3), 'b.', 'linewidth', 2)
plot(tau_a,orth(3,:)', 'g.', 'linewidth', 2)

subplot(2,2,4)
plot(tau_a,atilde(4,:)', 'r.', 'linewidth', 2)
hold on 
plot(tau_a,err(:,4), 'b.', 'linewidth', 2)
sgtitle('S2: elements of u ')
plot(tau_a,orth(4,:)', 'g.', 'linewidth', 2)


figure
hold on 
plot(tau_a,norms_proj, 'r.', 'linewidth', 2)
plot(tau_a,norms_std, 'b.', 'linewidth', 2)
plot(tau_a,norms_orth, 'g.', 'linewidth', 2)
plot(tau_a,stance*10, 'k.', 'linewidth', 2)
legend('proj_u(atilde)', 'atilde', 'atilde -proj_u(atilde)', 'stance' )

title('S2 norms: atilde = act model - act true')

%%

hr1_model = [];
hr1_true = []; 

hr2_model = [];
hr2_true = [];

for i = 1:length(actS1)
    u = torque(:,i); 
    hr1_model = [hr1_model , driftS1(i,:) + actS1(i,:)*u]; 
    hr1_true = [hr1_true, -(errLfs1(i) + errLgs1(i,:)*u)];
    
    hr2_model = [hr2_model , driftS2(i,:) + actS2(i,:)*u]; 
    hr2_true = [hr2_true, -(errLfs2(i) + errLgs2(i,:)*u)];

end
%%
set(0, 'DefaultLineLinewidth', 2)
figure 
hold on 
plot(hr1_model, 'r')
plot(hr1_true, 'b--')
title('Comparing Residuals: S1')
legend('Learning Model Residual', 'True Residual')
xlabel('sample')

figure 
hold on 
plot(hr2_model, 'r')
plot(hr2_true, 'b--')
title('Comparing Residuals: S2')
legend('Learning Model Residual', 'True Residual')
xlabel('sample')




%%
figure 
hold on 
plot(a_pos(:, 0.5*10^4:1.5*10^4)', 'linewidth', 2)
title('position states: [0.5, 1.5]')

figure
hold on 
plot(a_vel(:, 0.5*10^4:1.5*10^4)', 'linewidth', 2)
title('velocity states: [0.5, 1.5]')


figure
hold on 
plot(torque(:, 0.5*10^4:1.5*10^4)', 'linewidth', 2)
title('inputs: [0.5, 1.5]')
