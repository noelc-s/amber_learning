function [] = compute_safety(filename, plot_flag, is_sim)
% Load extracted data from './filtered_data/filtered_data_{filename}.mat'
% 
% 1. Load nominal gair and controller parameters
% 2. Compile true hdots (from spline numerical differentiation) and model
%    hdots (from safety function Lfh, Lgh functions)
%
% Save data to './learning_data/learning_data_0_{filename}.csv' and 
% './learning_data/learning_data_1_{filename}.csv' for the left and right 
% stance feet respectively
% 
% Plot all trues and models and then plot them with their residuals up to
% maxPlot

maxplot = 5; % Plot steps from 1 to maxplot or all steps in data
        

%% Load Data and Define Parameters

% Load Filtered Data
load(['./filtered_data/filtered_data_', filename(1:end-4), '.mat'])

% Add dynamics and controller parameters
addpath('./dynamics')
addpath('./safety_funcs_tau')
addpath('../yaml')

addpath(fullfile(pwd, 'gaits/'));
gait_num = '0.34';
gaitpath = strcat('gaits/',gait_num,'/');
list = dir(fullfile(pwd,gaitpath));
gaitname = list(4).name;
PARAM = YAML.read(strcat(gaitpath,gaitname));

% Control Parameters from Min 
thetamp = PARAM.domain.p;
factor = 0.3285;
ldes = 0.4;%; 0.54*factor;
lf = 0.4064; 
lT = 0.4999; 
lt = 1.373-lT-lf;
c = [-lt-lf -lf 0 0 0];
m = 4;
m1 = 12;%22; %% 18; 18 worked!!!! % TODO: Variable parameter for exponential decay, Noel has 20
m2 = 12;%5; 
a = 0.1;   % TODO: Variable Parameter, Min said 1 in message
r = 0.02/(1+a); %% 0.025 worked!!! % TODO: The stone radius

% Barrier Function, Extended Class K gains
alpha_e1 = 20;%150; % s1, alpha_e
alpha_1  = 20;%150; % s_1, alpha, not used

alpha_e2 = 20;%150; % s_2, alpha_e 
alpha_2  = 20;%150; % s_2, alpha, not used

if is_sim
    current2torque = 1;%1/91;% The sim logs out joint troques, convet to motor shaft torques
else
    current2torque = 1/23.13;%/91; % RL logs joint currents, convert to motor shaft torqursd
end


%% Compile true and model based h_dots
% Instantiate right and left stance foot outputs 
data_0 = [];
data_1 = [];
if plot_flag %%%% TODO: This is hacky, fix this to make it neater and in one place. Plots the constraints and the residuals
    figure 
    hold on 
end
for stepNum = 1:length(filtered_data)-1%% TODO CHANGE BACK< THIS IS JUST FOR THE SIM_SLIGHTLY_DIFF
    
    clear q q_dot_spline q_dot_diff u
    
    % Evalute splines 
%     if is_sim
%         q = filtered_data{stepNum}.q; 
%         q_dot = filtered_data{stepNum}.q_dot; 
%         u = filtered_data{stepNum}.u;
%         x = [q,q_dot]';
%         u = u'; 
%     else
        for i = 1:5 
            q(:,i) = fnval(filtered_data{stepNum}.q_spline{i}, filtered_data{stepNum}.tau);
            q_dot_spline(:,i) = fnval(filtered_data{stepNum}.q_dot_spline{i}, filtered_data{stepNum}.tau);
        end
        x = [q, q_dot_spline]';

        for i = 1:4 
            u(:,i) = current2torque*fnval(filtered_data{stepNum}.u_spline{i}, filtered_data{stepNum}.tau);
        end
        u = u';
%     end

    
    % Safety Functions
    %   longer step
    s1 = CBF_s1(x,c,thetamp,ldes,r,m1, a);
    Lfs1 = CBF_Lfs1(x,c,thetamp,ldes,r,m1, a);
    Lf2s1 = CBF_Lf2s1(x,c,thetamp,ldes,r,m1,a);
    s1e = Lfs1 + alpha_e1*s1; 

    
    %   for shorter step
    s2 = CBF_s2(x,c,thetamp,ldes,r,m2, a);
    Lfs2 = CBF_Lfs2(x,c,thetamp,ldes,r,m2, a);
    Lf2s2 = CBF_Lf2s2(x,c,thetamp,ldes,r,m2,a);
    s2e = Lfs2 + alpha_e2*s2;
    
    % Compile LgLfs's
    LgLfs1 = [];
    LgLfs2 = [];
    LgLfs1uvec = [];
    LgLfs2uvec = [];
    LgLfs1_norm_sq=[];
    LgLfs2_norm_sq=[];
    for i = 1:length(x)
        LgLfs1 = CBF_LgLfs1(x(:,i),c,thetamp,ldes,r,m1,a);
        LgLfs2 = CBF_LgLfs2(x(:,i),c,thetamp,ldes,r,m2,a);
        LgLfs1_norm_sq(i) = LgLfs1*LgLfs1'; 
        LgLfs2_norm_sq(i) = LgLfs2*LgLfs2';
        LgLfs1uvec(i) = LgLfs1*u(:,i);  
        LgLfs2uvec(i) = LgLfs2*u(:,i); 
    end
    
    
    % Calculate true se and se_dot
    s1e_dot_true{stepNum} = gradient(s1e(:))./gradient(filtered_data{stepNum}.tau).*filtered_data{stepNum}.dtaudt;
    s2e_dot_true{stepNum} = gradient(s2e(:))./gradient(filtered_data{stepNum}.tau).*filtered_data{stepNum}.dtaudt;
    
    
    % Remove any nans and infs which result from repeated values of tau
    %   (ex: [tau_1, tau_2, tau_1])
    nan_indeces = find(isnan(s1e_dot_true{stepNum}) | isinf(s1e_dot_true{stepNum}));
    nan_indeces = [nan_indeces, find(isnan(s2e_dot_true{stepNum}) | isinf(s2e_dot_true{stepNum}))];
    s1e_dot_true{stepNum}(nan_indeces) = []; 
    s2e_dot_true{stepNum}(nan_indeces) = []; 
    x(:,nan_indeces) = []; 
    u(:, nan_indeces)= []; 
    filtered_data{stepNum}.tau(nan_indeces) = [];
    Lf2s1(nan_indeces) = [];
    Lf2s2(nan_indeces) = [];
    LgLfs1uvec(nan_indeces) = [];
    LgLfs2uvec(nan_indeces) = [];
    Lfs1(nan_indeces) = [];
    Lfs2(nan_indeces) = [];
    LgLfs1_norm_sq(nan_indeces) = []; 
    LgLfs2_norm_sq(nan_indeces) = [];
    filtered_data{stepNum}.tau(nan_indeces) = [];
    
    % Calculate dhdx
    ds1edx = []; 
    ds2edx = [];
    for i = 1:length(x(1,:)) 
        ds1edx = [ds1edx; CBF_ds1edx(x(:,i),c,thetamp,ldes,r,m1,a)];
        ds2edx = [ds2edx; CBF_ds2edx(x(:,i),c,thetamp,ldes,r,m2,a)]; 
    end
    
    % Compute model based s_dot
    s1e_dot_model{stepNum} =  (Lf2s1 + LgLfs1uvec + alpha_e1*Lfs1);
    s2e_dot_model{stepNum} =  (Lf2s2 + LgLfs2uvec + alpha_e2*Lfs2);
    

    % Compute error residuals
    s1_dot_r{stepNum} = (s1e_dot_true{stepNum} - s1e_dot_model{stepNum}');%./LgLfs1_norm_sq';
    s2_dot_r{stepNum} = (s2e_dot_true{stepNum} - s2e_dot_model{stepNum}');%./LgLfs2_norm_sq';


    % Store data for learning
    one_stack = ones(length(x(1,:)),1);
    
    if is_sim 
        data_0 = [ data_0; [stepNum*one_stack, filtered_data{stepNum}.stance(1)*one_stack, x', u', ds1edx, ds2edx, s1_dot_r{stepNum}, s2_dot_r{stepNum}]];
    else
        if filtered_data{stepNum}.stance(1) == 0 
            data_0 = [ data_0; [stepNum*one_stack, filtered_data{stepNum}.stance(1)*one_stack, x', u', ds1edx, ds2edx, s1_dot_r{stepNum}, s2_dot_r{stepNum}]];
        else
            data_1 = [ data_1; [stepNum*one_stack, filtered_data{stepNum}.stance(1)*one_stack, x', u', ds1edx, ds2edx, s1_dot_r{stepNum}, s2_dot_r{stepNum}]];
        end
    end 
    
    if plot_flag
%         plot(Lf2s1 + LgLfs1uvec + alpha_e1*Lfs1)
%         %plot(Lf2s1 + LgLfs1uvec + alpha_e1*Lfs1 + alpha_1*s1e, 'b')
%         %plot(alpha_1*s1e, 'r')
%         %plot(Lf2s1 + LgLfs1uvec + alpha_e1*Lfs1, 'k')
        plot(filtered_data{stepNum}.tau, s1_dot_r{stepNum}, 'b')
        plot(filtered_data{stepNum}.tau, s2_dot_r{stepNum}, 'r')
        %subplot(2,1,1)
        %hold on 
        %plot(filtered_data{stepNum}.tau, s1_dot_r{stepNum}, 'b')
        %subplot(2,1,2)
        %hold on 
        %plot(filtered_data{stepNum}.tau, s2_dot_r{stepNum}, 'b')
    end
end
if plot_flag
    %legend('s_1 residual (true - model)', 's_2 residual (true - model)')
    sgtitle(['Residuals for gait: ', filename])
end 

%% Plot 

if plot_flag == true
    
    % Plot the s1e_dot_true and s1e_dot_model for each stance
    figure 
    hold on 
    for stepNum = 1:(maxplot*(maxplot<(length(filtered_data))) + length(filtered_data)*(maxplot>=length(filtered_data)))
        if filtered_data{stepNum}.stance(1) == 0 
            subplot(2,1,1)
            hold on
        else
            subplot(2,1,2)
            hold on
        end
        plot(filtered_data{stepNum}.tau, s1e_dot_true{stepNum}, 'r', 'linewidth', 2)
        plot(filtered_data{stepNum}.tau, s1e_dot_model{stepNum}, 'b', 'linewidth', 2)
    end
    sgtitle('$\dot{s}_1$: far side of stone', 'interpreter', 'latex')
    subplot(2,1,1)
    title('left')
    legend('true, from splines', 'model') 
    subplot(2,1,2)
    title('right')
    xlabel('tau')
    
    % Plot the s2e_dot_true and s2e_dot_model for each stance
    figure 
    hold on 
    for stepNum = 1:(maxplot*(maxplot<(length(filtered_data))) + length(filtered_data)*(maxplot>=length(filtered_data)))
        if filtered_data{stepNum}.stance(1) == 0 
            subplot(2,1,1)
            hold on
        else
            subplot(2,1,2)
            hold on
        end
        plot(filtered_data{stepNum}.tau, s2e_dot_true{stepNum}, 'r', 'linewidth', 2)
        plot(filtered_data{stepNum}.tau, s2e_dot_model{stepNum}, 'b', 'linewidth', 2)
    end
    sgtitle('$\dot{s}_2$: near side of stone', 'interpreter', 'latex')
    subplot(2,1,1)
    legend('true, from splines', 'model') 
    title('left')
    subplot(2,1,2)
    xlabel('tau')
    title('right')

    % Plot individual comparisons
    for stepNum = 1:(maxplot*(maxplot<(length(filtered_data))) + length(filtered_data)*(maxplot>=length(filtered_data)))
            figure 
            subplot(2,1,1)
            hold on 
            plot(filtered_data{stepNum}.tau, s1e_dot_true{stepNum}, 'r', 'linewidth', 2)
            plot(filtered_data{stepNum}.tau, s1e_dot_model{stepNum}, 'b', 'linewidth', 2)
            title('$\dot{s}$: far side of stone', 'interpreter', 'latex')
            legend('true s_dot, from splines', 'model, s_dot') 
            subplot(2,1,2)
            plot(filtered_data{stepNum}.tau, s1_dot_r{stepNum})
            legend('residual')
            xlabel('tau')


    end

end

%%
% Save learning data
cd('./learning_data');
if is_sim 
    csvwrite(['learning_data_sim_', filename(1:end-4), '.csv'], data_0);
else
    csvwrite(['learning_data_L_', filename(1:end-4), '.csv'], data_0);
    csvwrite(['learning_data_R_', filename(1:end-4), '.csv'], data_1);
end


%save(filename, 'data_d');
cd ..;


end

