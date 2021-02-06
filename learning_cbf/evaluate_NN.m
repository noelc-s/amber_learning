function [act, drift] = evaluate_NN(x)
%%
addpath('./network_parameters/data_4_20_states_10_epochs/')


learning_stack = [x', CBF_ds1edx(x,c,thetamp,ldes,r,m1,a)] ;

act_w1 = load('./network_parameters/data_4_20_states_10_epochs/batch_Ls1/act_w_1.csv');
act_b1 = load('./network_parameters/data_4_20_states_10_epochs/batch_Ls1/act_b_1.csv');
act_w1 = load('./network_parameters/data_4_20_states_10_epochs/batch_Ls1/act_w_2.csv');
act_b2 = load('./network_parameters/data_4_20_states_10_epochs/batch_Ls1/act_b_2.csv');





end
