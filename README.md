# Learning Method Code
This code is based on code written by Andrew Taylor for his work titled "Learning for Safety-Critical Control with Control Barrier Functions". Edits for use on the AMBER-3M platform with applications to the Stepping Stone problem were made by Ryan Cosner. 

# Code Structure: 
- **/data**: Data to be parsed in the form of .csv files output by the simulator or hardware-controller code. 

- **/learning_cbf**: main code for data parsing and learning
    - **/aggregated_data**: relevant data in a .mat file taken from output csv. 
    -**/extracted_data**: data separated into steps with noisy parts and repeated samples removed, stored as .mat files
    -**/filtered_data**: data smoothed with splines, stored as .mat files
    -**/gaits**: gait information used to calculate safety function information with respect to the nominal gait
    -**/Hardware**: hardware model of AMBER-3M
    -**/learning_code**: 
        -**/amber_learning.ipynb**: jupyter notebook containing learning code
    -**/learning_data**: processed data to be used for learning
    -**/network_parameters**: 
        -**/batch_Ls1, /batch_Ls2, /sim_data/ /no_act**: output from most recent learning model depending on the type of model and the data used to train it
        -**/python_model**: saved copies of the learning model
        -**/generateHeader.m**: generates weigths.h
        -**/weights.h**: C++ header to be used with simulation and hardware code
    -**/Raisim**: simulation model of AMBER-3M 
    -**/safety_funcs_tau**: contains safety functions generated using nominal model
    -**/aggregate_data.m**: aggregates data and saves to ./aggregated_data/
    -**/batch_processing.m**: runs process_data.m on all the files in the ../data folder and saves them to learning_data
    -**/compute_safety.m**: takes in data from ./filtered_data/ and computes the true safety function derivative using the numerical gradient and the model safety function derivative. Computes the delta error residual and saves that and all other relevant data (ie: states and inputs) to learning_data
    -**/extract_data.m**: extracts data and saves to ./extracted_data/
    -**/filter_data.m**: filters data and saves to ./filtered_data/
    -**/process_data.m**: runs aggregate_data.m, extract_data.m, filter_data.m, and compute_safety.m on a single data file from ../data/
-**/yaml**: Gait information for nominal periodic walking