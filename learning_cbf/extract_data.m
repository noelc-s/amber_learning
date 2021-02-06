function [] = extract_data(filename, plot_flag)%, same_step_length)
% Load aggregated data from the .mat file
% './aggregated_data/aggregated_data_{filename}.mat'
%
% 1. Removes repeated values of tau 
% 2. Separate out the different steps 
% 3. Rearrange within each step to Min's convenction [qsf,qsk,qsh,qnsh,qnsk]
%    for positions, velocities and torques
%
% Save data to './extracted_data/extracted_data_{filename}.mat'
%   The data is a cell array called extracted_data of structs where each 
%   struct contains the data for a step: 
%       - t, tau, switchBack, tau_a, tau_d, stance, a_pos, a_vel, d_pos, 
%         d_vel, torque 
% 
% Plot 4 plots: all qs, qdots, ses, and torques for every kept step



load( ['./aggregated_data/aggregated_data_', filename(1:end-4), '.mat'])

%% Determine relevant data
%   Remove: 
%       -repeated samples, where tau is the same

% Change these for the different files and write down the values!
start_tau = 0.5; 
end_tau = 1.0;

relevant_samples = []; 
tau_0 = aggregated_data.tau_a(1); 

% Remove Nonrepeating taus
for j = 1:length(aggregated_data.tau_a)
    tau_compare = aggregated_data.tau_a(j);
    if tau_compare ~= tau_0 && tau_compare > start_tau   && tau_compare < end_tau                         
        tau_0 = tau_compare; 
        relevant_samples = [relevant_samples, j];
    end
end
% Keep only relevant samples
keys = fieldnames(aggregated_data); 
for key = 1:numel(keys)
    if isnumeric(aggregated_data.(keys{key})) 
        aggregated_data.(keys{key}) = aggregated_data.(keys{key})(relevant_samples, :);
    end
end


%% Separate out and log the steps
%   Separate steps according to stance foot switches 
%   

stepNum = 1; 
stance = aggregated_data.stance(1); 
stepLength = 1; 
out = [];
step_start_sample = 1;

for i = 1:length(aggregated_data.t)-1
    if stance == aggregated_data.stance(i+1)
        % stance hasn't changed, count sample as part of previous step
        stepLength(stepNum) = stepLength(stepNum)+1; 
    else
        % stance has changed, log finished step
        stepStart = step_start_sample;
        stepEnd = step_start_sample+stepLength(stepNum)-1;
        step_start_sample = step_start_sample + stepLength(stepNum);

        % Adjust for percent start
        for key = 1:numel(keys)
            if isnumeric(aggregated_data.(keys{key}))
                step.(keys{key}) = aggregated_data.(keys{key})(stepStart:stepEnd,:);
            end
        end
        % start new step 
        extracted_data{stepNum} = step;
        stepNum = stepNum+1; 
        stance = aggregated_data.stance(i+1);
        stepLength(stepNum)=1;
    end
end
% Catch final step (since stance doesn't change at end)
stepStart = step_start_sample;
stepEnd = step_start_sample+stepLength(stepNum)-1;

for key = 1:numel(keys)
    if isnumeric(aggregated_data.(keys{key})) 
        step.(keys{key}) = aggregated_data.(keys{key})(stepStart:stepEnd,:); 
    end
end
if stepLength ~= 0 
    extracted_data{stepNum} = step;
end


%% Rearrange data in accordance with stance foot so that the order is: 
%   [qsf,qsk,qsh,qnsh,qnsk], Min's convention
%   Switch states and torques 

for i = 1:length(extracted_data)
    % Rearrange for stance foot, 
    if extracted_data{i}.stance(1)==0
        % Left foot
        a_rearrange = [1 2 3 4 5];
        d_rearrange = [1 2 3 4];
    else 
        % Right foot 
        a_rearrange = [1 5 4 3 2]; 
        d_rearrange = [4 3 2 1];
    end
    extracted_data{i}.a_pos = extracted_data{i}.a_pos(:,a_rearrange); 
    extracted_data{i}.a_vel = extracted_data{i}.a_vel(:,a_rearrange); 
    extracted_data{i}.d_pos = extracted_data{i}.d_pos(:,d_rearrange); 
    extracted_data{i}.d_vel = extracted_data{i}.d_vel(:,d_rearrange); 
    extracted_data{i}.torque= extracted_data{i}.torque(:,d_rearrange); 
    
    % Switch from q_torso to q_sf
    extracted_data{i}.a_pos(:,1) = extracted_data{i}.a_pos(:,1) - extracted_data{i}.a_pos(:,2) - extracted_data{i}.a_pos(:,3);
    extracted_data{i}.a_vel(:,1) = extracted_data{i}.a_vel(:,1) - extracted_data{i}.a_vel(:,2) - extracted_data{i}.a_vel(:,3);
end




%% Save Data 
save_filename = ['extracted_data_', filename(1:end-4), '.mat'];
cd('./extracted_data');
save(save_filename, 'extracted_data');                                     
cd ..;


%% Plot 

if plot_flag == true
    % Plot the qs of all the steps
    figure
    hold on 
    color = ['k', 'b', 'g', 'r', 'y'];
    for i = 1:length(extracted_data)
        for j = 1:5
             plot(extracted_data{i}.tau_a ,extracted_data{i}.a_pos(:,j), color(j)); 
        end
    end
    title(['Position Data: $q$, total number of steps = ', string(length(extracted_data))], 'Interpreter', 'latex')
    xlabel('\tau_a')
    ylabel('$q$', 'Interpreter', 'latex')
    legend('$q_\textrm{stance foot}$', '$q_\textrm{stance knee}$', '$q_\textrm{stance hip}$', '$q_\textrm{nonstance hip}$', '$q_\textrm{nonstance knee}$', 'Interpreter', 'latex', 'location', 'best')

    % Plot the q_dots of all the steps
    figure
    hold on 
    color = ['k', 'b', 'g', 'r', 'y'];
    for i = 1:length(extracted_data)
        for j = 1:5
             plot(extracted_data{i}.tau_a,extracted_data{i}.a_vel(:,j), color(j)); 
        end
    end
    title('Velocity Data: $\dot{q}$', 'Interpreter', 'latex')
    xlabel('\tau_a')
    ylabel('$\dot{q}$', 'Interpreter', 'latex')
    legend('$\dot{q}_\textrm{stance foot}$', '$\dot{q}_\textrm{stance knee}$', '$\dot{q}_\textrm{stance hip}$', '$\dot{q}_\textrm{nonstance hip}$', '$\dot{q}_\textrm{nonstance knee}$', 'Interpreter', 'latex', 'location', 'best')

    % Plot the inputs of all the steps
    figure
    hold on 
    for i = 1:length(extracted_data)
        for j = 1:4
            plot(extracted_data{i}.tau_a, extracted_data{i}.torque(:,j), color(j)); 
        end
    end
    title('Torques: $u$', 'Interpreter', 'latex')
    xlabel('\tau_a')
    ylabel('$q$', 'Interpreter', 'latex')
    legend('$u_\textrm{stance knee}$', '$u_\textrm{stance hip}$', '$u_\textrm{nonstance hip}$', '$u_\textrm{nonstance knee}$', 'interpreter', 'latex')
    
    
    % Plot the inputs of all the steps
    figure
    hold on 
    for i = 1:length(extracted_data)
        plot(extracted_data{i}.tau_a, extracted_data{i}.he(:,1), 'b'); 
        plot(extracted_data{i}.tau_a, extracted_data{i}.he(:,2), 'r'); 
    end
    title('Safety: $h_e$', 'Interpreter', 'latex')
    xlabel('\tau_a')
    ylabel('$he$', 'Interpreter', 'latex')
    legend('he_1', 'he_2')
end


end

