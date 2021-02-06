function [] = aggregate_data(filename)
% Loads raw data file from the .csv file '../data/{filename}.csv'
% 
% Separates out data by source and add to a struct
%
% Saves struct to './aggregated_data/aggregated_data_{filename}.mat'
%   The struct contains: 
%   - t, tau, switchBack, tau_a, tau_d, stance, a_pos, a_vel, d_pos, 
%     d_vel, torque 

%% Load raw data file
A = readmatrix(['../data/', filename]);

%% Pull out relevant data

ind = 1;
aggregated_data.t = (A(:,ind)-A(1,ind));        ind=ind+1; % Sample time
aggregated_data.tau =  A(:,ind);                ind=ind+1;
aggregated_data.switchBack =  A(:,ind);         ind=ind+1; 
aggregated_data.tau_d =  A(:,ind);              ind=ind+1; % State variable used by controller
aggregated_data.tau_a =  A(:,ind);              ind=ind+1; % Measured state variable
aggregated_data.stance = A(:,ind);              ind=ind+1; % Stance leg (1 for right)
aggregated_data.a_pos = A(:,ind:ind+4);         ind=ind+5; % Meas. pos. [Torso, LK, LH, RH, RK]
aggregated_data.a_vel = A(:,ind:ind+4);         ind=ind+5; % Meas. vel.
aggregated_data.d_pos = A(:,ind:ind+3);         ind=ind+4; % Desired pos.
aggregated_data.d_vel = A(:,ind:ind+3);         ind=ind+4; % Desired vel.
aggregated_data.torque = A(:,ind:ind+3);        ind=ind+4; % Torque [Nm]
      %udes = A(:,ind:ind+3);          
      ind=ind+4; % Torque [Nm]
      %foot_pos = A(:,ind:ind+1);      
      ind=ind+2; % Barrier function
      %hip_pos = A(:,ind:ind+1);       
      ind=ind+2; % Barrier function
      %h = A(:,ind:ind+2);             
      ind=ind+3; % Barrier function
aggregated_data.he = A(:,ind:ind+2);            ind=ind+3; % Barrier functions
aggregated_data.ldes = A(:,end);

% Additional unused data
%     aggregated_data.Lfh = A(:,ind:ind+2);           ind=ind+3; % Barrier function
%	  Lf2h = A(:,ind:ind+2);          ind=ind+3; % Barrier function
%     LgLfh{1} = A(:,ind:ind+3);      ind=ind+4; % Barrier function
%     LgLfh{2} = A(:,ind:ind+3);      ind=ind+4; % Barrier function
%     LgLfh{3} = A(:,ind:ind+3);      ind=ind+4; % Barrier function
%     actS1 = A(:,ind:ind+3);         ind=ind+4; % Barrier function
%     actS2 = A(:,ind:ind+3);         ind=ind+4; % Barrier function
%     driftS1 = A(:,ind);             ind=ind+1;
%     driftS2 = A(:,ind);             ind=ind+1;
%     foot_pos = A(:,ind:ind+1);      ind=ind+2; % Barrier function
%     hip_pos = A(:,ind:ind+1);       ind=ind+2; % Barrier function 


%% Write data to a .mat file for future use

aggregate_filename = ['aggregated_data_', filename(1:end-4)];
cd('./aggregated_data');
save(aggregate_filename, 'aggregated_data');                                     
cd ..;

end

