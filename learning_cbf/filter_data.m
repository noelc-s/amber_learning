function [] = filter_data( filename, plot_flag, is_sim)
% Load extracted data from './extracted_data/extracted_data_{filename}.mat'
% 
% 1. Get splines for tau, q, q_dot, input
%       
% Save data to './filtered_data/filtered_data_{filename}.mat'
%   The data is a cell array called filtered_data of structs where each 
%   struct contains the data for a step: 
%       - tau, t
%       - Spline Functions: for q, q_dot, u, dtaudt
% 
% Plot 4 plots: all qs, qdots, torques, tau and their splines for maxPlot
%   number of steps. 

    max_plot = 5; 


%% Load Data

    filepath = ['./extracted_data/extracted_data_', filename(1:end-4), '.mat'];
    load(filepath);
                

%% Define Splines to smooth data

    % Set tolerances for curves
    if is_sim
        q_tol =1e-8;  
        q_dot_tols = 1e-8*[1,1,1,1,1];
        u_tols = 1e-8*[1,1,1,1]; 
        tau_tol = 1e-8; 
        dtaudt_tol = 1e-8; 
        se_tol = 1e-4;
    else
        q_tol = 4;%1e-8;  
        q_dot_tols = 1+[5,7,7,7,7]; %1e-8*[1,1,1,1,1];%1/2*[5*1e-3, 1e-3, 0.5*1e-3, 0.5*1e-3, 1e-3];
        u_tols = 10*[1,1,1,1]; %1e-8*[1,1,1,1];%1/2*[1e2, 2*1e3,2*1e3,1e3]; 
        tau_tol = 3; %1e-6; 
        dtaudt_tol = 3; %1e-8; %1e-6; 
        se_tol = 9;  %3*1e-2;
    end

    for stepNum = 1:length(extracted_data)

        filtered_data{stepNum}.ldes = extracted_data{stepNum}.ldes(1);
        filtered_data{stepNum}.stance = extracted_data{stepNum}.stance(1);

        filtered_data{stepNum}.tau = extracted_data{stepNum}.tau_a;


        % Set the t_0 for each step to zero and convert to seconds
        filtered_data{stepNum}.t = (extracted_data{stepNum}.t - extracted_data{stepNum}.t(1))*1e-3;

        % Get tau from splines
        tau_func = polyfit(filtered_data{stepNum}.t, filtered_data{stepNum}.tau , tau_tol );%spaps(filtered_data{stepNum}.t, filtered_data{stepNum}.tau , tau_tol ) ; 
        filtered_data{stepNum}.tau = polyval(tau_func, filtered_data{stepNum}.t); %fnval(tau_func, filtered_data{stepNum}.t); 
        dtaudt_func = polyfit(filtered_data{stepNum}.tau, gradient(filtered_data{stepNum}.tau)./gradient(filtered_data{stepNum}.t), dtaudt_tol);%spaps(filtered_data{stepNum}.tau, gradient(filtered_data{stepNum}.tau)./gradient(filtered_data{stepNum}.t), dtaudt_tol);
        filtered_data{stepNum}.dtaudt = polyval(dtaudt_func, filtered_data{stepNum}.tau);

         % Get splines for q
         for i = 1:5 
             filtered_data{stepNum}.q_spline{i} = polyfit(filtered_data{stepNum}.tau, extracted_data{stepNum}.a_pos(:,i), q_tol);%spaps(filtered_data{stepNum}.tau ,extracted_data{stepNum}.a_pos(:,i),q_tol);
         end


         % Get splines for q_dot
         for i = 1:5
             q_dot_tol = q_dot_tols(i);
             filtered_data{stepNum}.q_dot_spline{i} = polyfit(filtered_data{stepNum}.tau, extracted_data{stepNum}.a_vel(:,i),q_dot_tol);%spaps(filtered_data{stepNum}.tau, extracted_data{stepNum}.a_vel(:,i),q_dot_tol);
         end

        % Get splines for u 
        for i = 1:4
            u_tol = u_tols(i);
            filtered_data{stepNum}.u_spline{i} = polyfit(filtered_data{stepNum}.tau, extracted_data{stepNum}.torque(:,i),u_tol);%spaps(filtered_data{stepNum}.tau, extracted_data{stepNum}.torque(:,i),u_tol);
        end


        % Get splines for se
        filtered_data{stepNum}.s1e_spline = polyfit(filtered_data{stepNum}.tau ,extracted_data{stepNum}.he(:,1),se_tol);%spaps(filtered_data{stepNum}.tau ,extracted_data{stepNum}.he(:,1),se_tol);
        filtered_data{stepNum}.s2e_spline = polyfit(filtered_data{stepNum}.tau ,extracted_data{stepNum}.he(:,2),se_tol);%spaps(filtered_data{stepNum}.tau ,extracted_data{stepNum}.he(:,2),se_tol);

    end

%% Plotting

    if plot_flag

%         % Plot q
%         qNames = {'torso', 'left knee', 'left hip', 'right hip', 'right knee'};
%          color = ['k', 'b', 'g', 'r', 'y'];
%          for stepNum = 1:max_plot
%             figure 
%              hold on 
%              for i = 1:5
%                  plot(filtered_data{stepNum}.tau, polyval(filtered_data{stepNum}.q_spline{i}, filtered_data{stepNum}.tau),  color(i));%fnval(filtered_data{stepNum}.q_spline{i}, filtered_data{stepNum}.tau),  color(i))
%              end
%              for i = 1:5
%                   plot(filtered_data{stepNum}.tau, extracted_data{stepNum}.a_pos(:,i), 'k--')
%              end
%              title('Position: splines')
%              ylabel('q')
%              xlabel('\tau')
%              legend(qNames)
%          end

%         % Plot qdot
          qNames = {'torso', 'left knee', 'left hip', 'right hip', 'right knee'};
           color = ['k', 'b', 'g', 'r', 'y'];
           for stepNum = 1:max_plot
              figure 
               hold on 
               for i = 1:5
                   plot(filtered_data{stepNum}.tau, polyval(filtered_data{stepNum}.q_dot_spline{i}, filtered_data{stepNum}.tau),  color(i))%fnval(filtered_data{stepNum}.q_dot_spline{i}, filtered_data{stepNum}.tau),  color(i))
               end
              for i = 1:5
                    plot(filtered_data{stepNum}.tau, extracted_data{stepNum}.a_vel(:,i), 'k--')
               end
               title('Velocity: splines')
               ylabel('q')
               xlabel('\tau')
               legend(qNames)
           end
 
 
          % Plot q_dot splines and differentiation plots
  
          for stepNum = 1:max_plot
              figure 
              hold on 
              for i = 1
                  plot(filtered_data{stepNum}.tau, extracted_data{stepNum}.he(:,1), 'k--')
                  plot(filtered_data{stepNum}.tau, polyval(filtered_data{stepNum}.s1e_spline, filtered_data{stepNum}.tau), 'b')%fnval(filtered_data{stepNum}.s1e_spline, filtered_data{stepNum}.tau), 'b')
                  plot(filtered_data{stepNum}.tau, extracted_data{stepNum}.he(:,2), 'k--')
                  plot(filtered_data{stepNum}.tau, polyval(filtered_data{stepNum}.s2e_spline, filtered_data{stepNum}.tau), 'r')%fnval(filtered_data{stepNum}.s2e_spline, filtered_data{stepNum}.tau), 'r')
              end
              title('$se$', 'interpreter', 'latex')
              legend('Raw Data', 'Spline on Raw Data')
          end
         % Plot torques 
         colors = ['g', 'b', 'm', 'r'];
 
         for stepNum = 1:max_plot
             figure
             hold on
             for i = 1:4
                 plot(filtered_data{stepNum}.tau, extracted_data{stepNum}.torque(:,i),[colors(i), '--'] );
                 legendObj(i) = plot(filtered_data{stepNum}.tau, polyval(filtered_data{stepNum}.u_spline{i}, filtered_data{stepNum}.tau),colors(i));%fnval(filtered_data{stepNum}.u_spline{i}, filtered_data{stepNum}.tau),colors(i));
             end
             title('Torque inputs')
             legend(legendObj(:),'$u_\textrm{stance knee}$', '$u_\textrm{stance hip}$', '$u_\textrm{nonstance hip}$', '$u_\textrm{nonstance knee}$', 'interpreter', 'latex')
         end
 
         for stepNum = 1:max_plot
             figure
             plot(filtered_data{stepNum}.t, extracted_data{stepNum}.tau_a, '--')
             hold on 
             plot(filtered_data{stepNum}.t, filtered_data{stepNum}.tau)
             legend('tau_a, tau')
             title('Tau spline')
         end
     end
%% Save Data
 
    cd('./filtered_data');
    save(['filtered_data_', filename(1:end-4), '.mat'], 'filtered_data');
    cd ..;

end

