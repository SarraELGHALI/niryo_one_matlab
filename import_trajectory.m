function [thero_trajectory,real_trajectory,diff_trajectory]=import_trajectory()
    [fullFileName]=open_folder();
    Trajectories=xlsread(fullFileName);
    thero_trajectory=Trajectories(1:7,:);
    real_trajectory=Trajectories(8:14,:);
    diff_trajectory=Trajectories(15:21,:);