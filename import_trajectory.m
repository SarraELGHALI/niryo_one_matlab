function [thero_trajectory,real_trajectory,diff_trajectory]=import_trajectory()
    [fullFileName]=open_folder();
    Trajectories=xlsread(fullFileName);
    thero_trajectory=Trajectories(1:7,:);
    real_trajectory=Trajectories(8:14,:);
    diff_trajectory=Trajectories(15:21,:);
    
    
function [fullFileName]=open_folder()
    startingFolder = userpath % Or "pwd" or wherever you want.
    defaultFileName = fullfile(startingFolder, '*.xls');
    [baseFileName, folder] = uigetfile(defaultFileName, 'Specify a file');
    if baseFileName == 0
        % User clicked the Cancel button.
        return;
    end
    fullFileName = fullfile(folder, baseFileName);