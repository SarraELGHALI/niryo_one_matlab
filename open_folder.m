function [fullFileName]=open_folder()
    startingFolder = userpath % Or "pwd" or wherever you want.
    defaultFileName = fullfile(startingFolder, '*.xls');
    [baseFileName, folder] = uigetfile(defaultFileName, 'Specify a file');
    if baseFileName == 0
        % User clicked the Cancel button.
        return;
    end
    fullFileName = fullfile(folder, baseFileName);