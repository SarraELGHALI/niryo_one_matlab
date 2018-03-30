 function [fullFileName]=save_folder()
startingFolder = userpath % Or "pwd" or wherever you want.
defaultFileName = fullfile(startingFolder, '*.txt');
[baseFileName, folder] = uiputfile(defaultFileName, 'Specify a file');
if baseFileName == 0
	% User clicked the Cancel button.
	return;
end
fullFileName = fullfile(folder, baseFileName);