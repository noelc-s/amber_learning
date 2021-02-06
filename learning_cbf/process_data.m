function [] = process_data( filename, plot_flag, is_sim, is_issf)
% Process a data file

% Aggregate 
aggregate_data(filename); 

% Extract data
extract_data(filename, plot_flag);

% % Filter data
filter_data(filename, plot_flag, is_sim);

% Compute safety
if is_issf
    "issf safety"
    compute_safety_issf(filename, plot_flag, is_sim);
else
    compute_safety(filename, plot_flag, is_sim);
end


end

