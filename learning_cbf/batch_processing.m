function [] = batch_processing(process, is_sim)
%%
files = dir('../data');

plot_flag = false; 
data_sim_stacked = [];
data_L_stacked = []; 
data_R_stacked = []; 


for i = 3:length(files)
    [i-2, 'of', string(length(files)-2)]
    
    filename = files(i).name 
    if process
        tic
        process_data(filename, false, is_sim) 
        toc
    end
    if is_sim 
        sim_out = load(['./learning_data/learning_data_sim_', filename]);
        data_sim_stacked =[data_sim_stacked; sim_out];
    else
        L_out = load(['./learning_data/learning_data_L_', filename]);
        %R_out = load(['./learning_data/learning_data_R_', filename]);
        data_L_stacked = [data_L_stacked; L_out];
        %data_R_stacked = [data_R_stacked; R_out];
    end
end

cd('./learning_data');

if is_sim
    csvwrite(['batch_sim.csv'], data_sim_stacked);
else
    csvwrite(['batch_L.csv'], data_L_stacked);
    csvwrite(['batch_R.csv'], data_R_stacked);
end
%save(filename, 'data_d');
cd ..;


end