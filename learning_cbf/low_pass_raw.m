%%
files = dir('../data');
n = 40; 
plot_flag = true;
%moving_average(filename, n, plot_flag)

w =0.05; 
is_sim = true;
for i = 3:length(files)
    i
    [i-2, 'of', string(length(files)-2)]
    tic
    filename = files(i).name;
    disc_low_pass(filename, w, false, is_sim)
    toc

end




%%

function [] = disc_low_pass(filename, w, plot_flag, is_sim)
%%
    addpath(fullfile(pwd, 'gaits/'));
    gait_num = '0.54';
    gaitpath = strcat('gaits/',gait_num,'/');
    list = dir(fullfile(pwd,gaitpath));
    gaitname = list(4).name;
    PARAM = YAML.read(strcat(gaitpath,gaitname));

    % Control Parameters from Min 
    thetamp = PARAM.domain.p;
    factor = 0.3285;
    ldes = 0.54*factor;
    lf = 0.4064; 
    lT = 0.4999; 
    lt = 1.373-lT-lf;
    c = [-lt-lf -lf 0 0 0];
    m = 4;
    r = .01; % TODO: The stone length is only 6 cm? 
    m1 = 20; % TODO: Variable parameter for exponential decay, Noel has 20
    a = 0.02;   % TODO: Variable Parameter, Min said 1 in message

    % Barrier Function, Extended Class K gains
    alpha_e1 = 80; % s1, alpha_e
    alpha_1  = 80; % s_1, alpha, not used

    alpha_e2 = 80; % s_2, alpha_e 
    alpha_2  = 80; % s_2, alpha, not used

    addpath('./safety_funcs_tau')    

    A = readmatrix(['../data/', filename]);  % (7:11) are position (12:16) are velocities
    if plot_flag 
         figure
         subplot(2,1,1)
             hold on 
             plot(A(:,7:11), 'k--')
         subplot(2,1,2)
             hold on 
             plot(A(:,12:16), 'k--')
    end
    range = {7:11, 12:16};
    beta = exp(-w); 
    if is_sim
        beta = 0; 
    end
    
    for key = 1:2
        for i = 1:length(A(:,1)) - 1
            A(i+1,range{key}) = beta*A(i, range{key}) + (1-beta)*A(i+1,range{key});
        end
    end
    
    A = [A, zeros(length(A),20)];
    for i = 1:length(A)-1

        
        if A(i,6)==0
            a_rearrange = [1 2 3 4 5];
            d_rearrange = [1 2 3 4];
        else 
            a_rearrange = [1 5 4 3 2]; 
            d_rearrange = [4 3 2 1];
        end
        A(i,[7:11, 12:16]) = A(i, [6 + a_rearrange, 11+a_rearrange]);
        x = A(i, [7:11, 12:16])'; 
        
        A(i, end-19:end) = [CBF_ds1edx(x,c,thetamp,ldes,r,m1,a),CBF_ds2edx(x,c,thetamp,ldes,r,m1,a) ];
    end
    if plot_flag
        subplot(2,1,1)
             plot(A(:,7:11))
         subplot(2,1,2)
             plot(A(:,12:16))
    end
%     
%             ds1edx = [ds1edx; CBF_ds1edx(x(:,i),c,thetamp,ldes,r,m1,a)];
%         ds2edx = [ds2edx; CBF_ds2edx(x(:,i),c,thetamp,ldes,r,m1,a)]; 
%     
%     for i = 1:length(A(:,1)) - 1    
%         , s1e_dot_model{stepNum}, s2e_dot_model{stepNum}
%         % Safety Functions
%     %   longer step
%         s1 = CBF_s1(x,c,thetamp,ldes,r,m1, a);
%         Lfs1 = CBF_Lfs1(x,c,thetamp,ldes,r,m1, a);
%         Lf2s1 = CBF_Lf2s1(x,c,thetamp,ldes,r,m1,a);
%         s1e = Lfs1 + alpha_e1*s1; 
% 
%         %   for shorter step
%         s2 = CBF_s2(x,c,thetamp,ldes,r,m1, a);
%         Lfs2 = CBF_Lfs2(x,c,thetamp,ldes,r,m1, a);
%         Lf2s2 = CBF_Lf2s2(x,c,thetamp,ldes,r,m1,a);
%         s2e = Lfs2 + alpha_e2*s2;
%             % Compute model based s_dot
%         s1e_dot_model(i) =  (Lf2s1 + LgLfs1uvec + alpha_e1*Lfs1);
%         s2e_dot_model(i) =  (Lf2s2 + LgLfs2uvec + alpha_e2*Lfs2);
%     end

    
    
    % Save learning data
    %%%%%%%%%%%%%%%%%% TODO %%%%%%%%%%%%%%%%%%%%%%%
    cd('./hardware_filter');
    csvwrite(['filtered_', filename(1:end-4), '.csv'], A);
    cd ..;

end



function [] = moving_average(filename, n, plot_flag)
  
    A = readmatrix(['../data/', filename]);  % (7:11) are position (12:16) are velocities

    if plot_flag 
         figure
         subplot(2,1,1)
             hold on 
             plot(A(:,7:11), 'k--')
         subplot(2,1,2)
             hold on 
             plot(A(:,12:16), 'k--')
    end
    % number of steps for moving average
    range = {7:11, 12:16};
    for key = 1:2
        samples = [];
        for i = 1:length(A(:,1)) - n
            samples = [samples; A(i,range{key})];
            A(i,range{key}) = sum(samples, 1)/length(samples(:,1));
            if length(samples(:,1))==n 
                samples(1,:) = [];
            end
        end
    end
    if plot_flag
        subplot(2,1,1)
             plot(A(:,7:11))
         subplot(2,1,2)
             plot(A(:,12:16))
    end

    % Save learning data
    %%%%%%%%%%%%%%%%%% TODO %%%%%%%%%%%%%%%%%%%%%%%
    cd('./hardware_filter');
    csvwrite(['filtered_', filename(1:end-4), '.csv'], A);
    cd ..;
end