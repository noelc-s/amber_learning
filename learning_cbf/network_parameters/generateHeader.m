%% Two Layer 
d_drift_in = 10;
d_act_in = 10;
d_drift_hidden = 50;
d_act_hidden = 50;
d_drift_out = 1;
d_act_out = 4;

delete('weights.h')
fidW = fopen('weights.h', 'wt' );

fprintf(fidW,'const uint32_t d_drift_in = %i;\n',d_drift_in);
fprintf(fidW,'const uint32_t d_act_in = %i;\n',d_act_in);
fprintf(fidW,'const uint32_t d_drift_hidden = %i;\n',d_drift_hidden);
fprintf(fidW,'const uint32_t d_act_hidden = %i;\n',d_act_hidden);
fprintf(fidW,'const uint32_t d_drift_out = %i;\n',d_drift_out);
fprintf(fidW,'const uint32_t d_act_out = %i;\n',d_act_out);


folders =  {'no_act/batch_Ls1' 'no_act/batch_Ls2'};% TODO {'batch_Ls1' 'batch_Ls2'};%, 'batch_Rs1', 'batch_Rs2'};
for folderNum = 1:2

    fprintf(fidW,'\n');

    drift_w_1 = csvread(strcat(folders{folderNum},'/drift_w_1.csv'));
    fprintf(fidW,strcat('const double w1sim', folders{folderNum}(15:end) , '_drift[d_drift_hidden*d_drift_in] = {'));
    for j = 1:d_drift_in
        for i = 1:d_drift_hidden
            if j == d_drift_in && i == d_drift_hidden
                fprintf(fidW,'%f};',drift_w_1(i,j));
            else
                
                fprintf(fidW,'%f,',drift_w_1(i,j));
            end
        end
    end
    fprintf(fidW,'\n');

    drift_w_2 = csvread(strcat(folders{folderNum},'/drift_w_2.csv'));
    fprintf(fidW,strcat('const double w2sim', folders{folderNum}(15:end) , '_drift[d_drift_hidden*d_drift_hidden] = {'));
    %drift_w_2 = drift_w_2';

    for j = 1:d_drift_hidden
        for i = 1:d_drift_hidden
            if j == d_drift_hidden && i == d_drift_hidden 
                fprintf(fidW,'%f};',drift_w_2(i,j));
            else
                fprintf(fidW,'%f,',drift_w_2(i,j));
            end
        end
    end
    
    fprintf(fidW,'\n');

    
    drift_w_3 = csvread(strcat(folders{folderNum},'/drift_w_3.csv'));
    fprintf(fidW,strcat('const double w3sim', folders{folderNum}(15:end) , '_drift[d_drift_hidden*d_drift_out] = {'));
    %drift_w_2 = drift_w_2';

    for j = 1:d_drift_hidden
        for i = 1:d_drift_out
            if j == d_drift_hidden && i == d_drift_out
                fprintf(fidW,'%f};',drift_w_3(i,j));
            else
                fprintf(fidW,'%f,',drift_w_3(i,j));
            end
        end
    end

    fprintf(fidW,'\n');

    drift_b_1 = csvread(strcat(folders{folderNum},'/drift_b_1.csv'));
    fprintf(fidW,strcat('const double b1sim', folders{folderNum}(15:end) , '_drift[d_drift_hidden] = {'));

    for i = 1:d_drift_hidden
        if i == d_drift_hidden
            fprintf(fidW,'%f};',drift_b_1(i));
        else
            fprintf(fidW,'%f,',drift_b_1(i));
        end
    end

    fprintf(fidW,'\n');

    drift_b_2 = csvread(strcat(folders{folderNum},'/drift_b_2.csv'));
    fprintf(fidW,strcat('const double b2sim', folders{folderNum}(15:end) , '_drift[d_drift_hidden] = {'));

    for i = 1: d_drift_hidden
        if i ==  d_drift_hidden
            fprintf(fidW,'%f};',drift_b_2(i));
        else
            fprintf(fidW,'%f,',drift_b_2(i));
        end
    end

    fprintf(fidW,'\n');

    drift_b_3 = csvread(strcat(folders{folderNum},'/drift_b_3.csv'));
    fprintf(fidW,strcat('const double b3sim', folders{folderNum}(15:end) , '_drift[d_drift_out] = {'));

    for i = 1:d_drift_out
        if i == d_drift_out
            fprintf(fidW,'%f};',drift_b_3(i));
        else
            fprintf(fidW,'%f,',drift_b_3(i));
        end
    end
    
    
    % act now

    fprintf(fidW,'\n');

    act_w_1 = csvread(strcat(folders{folderNum},'/act_w_1.csv'));
    fprintf(fidW,strcat('const double w1sim', folders{folderNum}(15:end) , '_act[d_drift_hidden*d_act_in] = {'));

    for j = 1:d_act_in
        for i = 1:d_act_hidden
            if j == d_act_in && i == d_act_hidden
                fprintf(fidW,'%f};',act_w_1(i,j));
            else
                fprintf(fidW,'%f,',act_w_1(i,j));
            end
        end
    end

    fprintf(fidW,'\n');

    act_w_2 = csvread(strcat(folders{folderNum},'/act_w_2.csv'));
    fprintf(fidW,strcat('const double w2sim', folders{folderNum}(15:end) , '_act[d_act_hidden*d_act_hidden] = {'));
    %act_w_2 = act_w_2'; % why did they flop these? 

    for j = 1:d_act_hidden
        for i = 1: d_drift_hidden
            if j == d_act_hidden && i == d_act_hidden
                fprintf(fidW,'%f};',act_w_2(i,j));
            else
                fprintf(fidW,'%f,',act_w_2(i,j));
            end
        end
    end

    fprintf(fidW,'\n');

    act_w_3 = csvread(strcat(folders{folderNum},'/act_w_3.csv'));
    fprintf(fidW,strcat('const double w3sim', folders{folderNum}(15:end) , '_act[d_act_hidden*d_act_out] = {'));
    %act_w_2 = act_w_2'; % why did they flop these? 

    for j = 1:d_act_hidden
        for i = 1:d_act_out
            if j == d_act_hidden && i == d_act_out
                fprintf(fidW,'%f};',act_w_2(i,j));
            else
                fprintf(fidW,'%f,',act_w_2(i,j));
            end
        end
    end
    
    fprintf(fidW,'\n');

    act_b_1 = csvread(strcat(folders{folderNum},'/act_b_1.csv'));
    fprintf(fidW,strcat('const double b1sim', folders{folderNum}(15:end) , '_act[d_act_hidden] = {'));

    for i = 1:d_act_hidden
        if i == d_act_hidden
            fprintf(fidW,'%f};',act_b_1(i));
        else
            fprintf(fidW,'%f,',act_b_1(i));
        end
    end

    fprintf(fidW,'\n');

    act_b_2 = csvread(strcat(folders{folderNum},'/act_b_2.csv'));
    fprintf(fidW,strcat('const double b2sim', folders{folderNum}(15:end) , '_act[d_act_hidden] = {'));

    for i = 1: d_drift_hidden
        if i ==  d_drift_hidden
            fprintf(fidW,'%f};',act_b_2(i));
        else
            fprintf(fidW,'%f,',act_b_2(i));
        end
    end
    
    fprintf(fidW,'\n');

    act_b_3 = csvread(strcat(folders{folderNum},'/act_b_3.csv'));
    fprintf(fidW,strcat('const double b3sim', folders{folderNum}(15:end) , '_act[d_act_out] = {'));

    for i = 1:d_act_out
        if i == d_act_out
            fprintf(fidW,'%f};',act_b_3(i));
        else
            fprintf(fidW,'%f,',act_b_3(i));
        end
    end
end


%% One Layer 
d_drift_in = 20;
d_act_in = 20;
d_drift_hidden = 100;
d_act_hidden = 100;
d_drift_out = 1;
d_act_out = 4;

delete('weights.h')
fidW = fopen('weights.h', 'wt' );

fprintf(fidW,'const uint32_t d_drift_in = %i;\n',d_drift_in);
fprintf(fidW,'const uint32_t d_act_in = %i;\n',d_act_in);
fprintf(fidW,'const uint32_t d_drift_hidden = %i;\n',d_drift_hidden);
fprintf(fidW,'const uint32_t d_act_hidden = %i;\n',d_act_hidden);
fprintf(fidW,'const uint32_t d_drift_out = %i;\n',d_drift_out);
fprintf(fidW,'const uint32_t d_act_out = %i;\n',d_act_out);


folders = {'batch_sims1' 'batch_sims2'};%, TODO 'batch_Rs1', 'batch_Rs2'};
for folderNum = 1:2

    fprintf(fidW,'\n');

    drift_w_1 = csvread(strcat(folders{folderNum},'/drift_w_1.csv'));
    fprintf(fidW,strcat('const double w1', folders{folderNum}(7:end) , '_drift[d_drift_hidden*d_drift_in] = {'));
    for j = 1:d_drift_in
        for i = 1:d_drift_hidden
            if j == d_drift_in && i == d_drift_hidden
                fprintf(fidW,'%f};',drift_w_1(i,j));
            else
                fprintf(fidW,'%f,',drift_w_1(i,j));
            end
        end
    end

    fprintf(fidW,'\n');

    drift_w_2 = csvread(strcat(folders{folderNum},'/drift_w_2.csv'));
    fprintf(fidW,strcat('const double w2', folders{folderNum}(7:end) , '_drift[d_drift_hidden*d_drift_out] = {'));
    %drift_w_2 = drift_w_2';

    for j = 1:d_drift_hidden
        for i = 1:d_drift_out
            if j == d_drift_hidden && i == d_drift_out
                fprintf(fidW,'%f};',drift_w_2(i,j));
            else
                fprintf(fidW,'%f,',drift_w_2(i,j));
            end
        end
    end

    fprintf(fidW,'\n');

    drift_b_1 = csvread(strcat(folders{folderNum},'/drift_b_1.csv'));
    fprintf(fidW,strcat('const double b1', folders{folderNum}(7:end) , '_drift[d_drift_hidden] = {'));

    for i = 1:d_drift_hidden
        if i == d_drift_hidden
            fprintf(fidW,'%f};',drift_b_1(i));
        else
            fprintf(fidW,'%f,',drift_b_1(i));
        end
    end

    fprintf(fidW,'\n');

    drift_b_2 = csvread(strcat(folders{folderNum},'/drift_b_2.csv'));
    fprintf(fidW,strcat('const double b2', folders{folderNum}(7:end) , '_drift[d_drift_out] = {'));

    for i = 1:d_drift_out
        if i == d_drift_out
            fprintf(fidW,'%f};',drift_b_2(i));
        else
            fprintf(fidW,'%f,',drift_b_2(i));
        end
    end

    % act now

    fprintf(fidW,'\n');

    act_w_1 = csvread(strcat(folders{folderNum},'/act_w_1.csv'));
    fprintf(fidW,strcat('const double w1', folders{folderNum}(7:end) , '_act[d_drift_hidden*d_act_in] = {'));

    for j = 1:d_act_in
        for i = 1:d_act_hidden
            if j == d_act_in && i == d_act_hidden
                fprintf(fidW,'%f};',act_w_1(i,j));
            else
                fprintf(fidW,'%f,',act_w_1(i,j));
            end
        end
    end

    fprintf(fidW,'\n');

    act_w_2 = csvread(strcat(folders{folderNum},'/act_w_2.csv'));
    fprintf(fidW,strcat('const double w2', folders{folderNum}(7:end) , '_act[d_act_hidden*d_act_out] = {'));
    %act_w_2 = act_w_2'; % why did they flop these? 

    for j = 1:d_act_hidden
        for i = 1:d_act_out
            if j == d_act_hidden && i == d_act_out
                fprintf(fidW,'%f};',act_w_2(i,j));
            else
                fprintf(fidW,'%f,',act_w_2(i,j));
            end
        end
    end

    fprintf(fidW,'\n');

    act_b_1 = csvread(strcat(folders{folderNum},'/act_b_1.csv'));
    fprintf(fidW,strcat('const double b1', folders{folderNum}(7:end) , '_act[d_act_hidden] = {'));

    for i = 1:d_act_hidden
        if i == d_act_hidden
            fprintf(fidW,'%f};',act_b_1(i));
        else
            fprintf(fidW,'%f,',act_b_1(i));
        end
    end

    fprintf(fidW,'\n');

    act_b_2 = csvread(strcat(folders{folderNum},'/act_b_2.csv'));
    fprintf(fidW,strcat('const double b2', folders{folderNum}(7:end) , '_act[d_act_out] = {'));

    for i = 1:d_act_out
        if i == d_act_out
            fprintf(fidW,'%f};',act_b_2(i));
        else
            fprintf(fidW,'%f,',act_b_2(i));
        end
    end
end