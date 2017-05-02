clear; clc

% Initialize variables.
filename = '/tmp/pose_3d.txt';
delimiter = ',';

% Read columns of data as text:
% For more information, see the TEXTSCAN documentation.
formatSpec = '%s%*s%*s%*s%s%s%s%[^\n\r]';

% Open the text file.
fileID = fopen(filename,'r');

% Read columns of data according to the format.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'TextType', 'string',  'ReturnOnError', false);

% Close the text file.
fclose(fileID);

% Convert the contents of columns containing numeric text to numbers.
% Replace non-numeric text with NaN.
raw = repmat({''},length(dataArray{1}),length(dataArray)-1);
for col=1:length(dataArray)-1
    raw(1:length(dataArray{col}),col) = mat2cell(dataArray{col}, ones(length(dataArray{col}), 1));
end
numericData = NaN(size(dataArray{1},1),size(dataArray,2));

for col=[1,2,3,4]
    % Converts text in the input cell array to numbers. Replaced non-numeric
    % text with NaN.
    rawData = dataArray{col};
    for row=1:size(rawData, 1)
        % Create a regular expression to detect and remove non-numeric prefixes and
        % suffixes.
        regexstr = '(?<prefix>.*?)(?<numbers>([-]*(\d+[\,]*)+[\.]{0,1}\d*[eEdD]{0,1}[-+]*\d*[i]{0,1})|([-]*(\d+[\,]*)*[\.]{1,1}\d+[eEdD]{0,1}[-+]*\d*[i]{0,1}))(?<suffix>.*)';
        try
            result = regexp(rawData(row), regexstr, 'names');
            numbers = result.numbers;
            
            % Detected commas in non-thousand locations.
            invalidThousandsSeparator = false;
            if numbers.contains(',')
                thousandsRegExp = '^\d+?(\,\d{3})*\.{0,1}\d*$';
                if isempty(regexp(numbers, thousandsRegExp, 'once'))
                    numbers = NaN;
                    invalidThousandsSeparator = true;
                end
            end
            % Convert numeric text to numbers.
            if ~invalidThousandsSeparator
                numbers = textscan(char(strrep(numbers, ',', '')), '%f');
                numericData(row, col) = numbers{1};
                raw{row, col} = numbers{1};
            end
        catch
            raw{row, col} = rawData{row};
        end
    end
end


% Replace non-numeric cells with NaN
R = cellfun(@(x) ~isnumeric(x) && ~islogical(x),raw); % Find non-numeric cells
raw(R) = {NaN}; % Replace non-numeric cells

% Allocate imported array to column variable names
time = cell2mat(raw(:, 1));
x = cell2mat(raw(:, 2));
y = cell2mat(raw(:, 3));
z = cell2mat(raw(:, 4));

% Clear temporary variables
clearvars filename delimiter formatSpec fileID dataArray ans raw col numericData rawData row regexstr result numbers invalidThousandsSeparator thousandsRegExp R;


%% 3D plots
figure
scatter3(x,y,z);
title('3D movement of ball')
xlabel('X-axis') % x-axis label
ylabel('Y-axis') % y-axis label
zlabel('Z-axis') % z-axis label

figure

subplot(3,1,1);
scatter(x,y)
title('XY movement of ball')
xlabel('X-axis') % x-axis label
ylabel('Y-axis') % y-axis label

subplot(3,1,2);
scatter(z,y);
title('ZY movement of ball')
xlabel('Z-axis') % z-axis label
ylabel('Y-axis') % y-axis label

subplot(3,1,3);
scatter(z,x);
title('ZX movement of ball')
xlabel('X-axis') % z-axis label
ylabel('Z-axis') % y-axis label
