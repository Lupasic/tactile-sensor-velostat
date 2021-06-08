function futek_data = import_futek_data(filename, dataLines)
%IMPORTFILE Import data from a text file
%  EXPERIMENTTESTSENSOR3F = IMPORTFILE(FILENAME) reads data from text
%  file FILENAME for the default selection.  Returns the data as a cell
%  array.
%
%  EXPERIMENTTESTSENSOR3F = IMPORTFILE(FILE, DATALINES) reads data for
%  the specified row interval(s) of text file FILENAME. Specify
%  DATALINES as a positive scalar integer or a N-by-2 array of positive
%  scalar integers for dis-contiguous row intervals.
%
%  Example:
%  experimenttestsensor3f = importfile("/home/lupasic/Programs/python_ws/tactile-sensor-velostat/src/futek_data/experiment_test_sensor3_f.txt", [1, Inf]);
%
%  See also READTABLE.
%
% Auto-generated by MATLAB on 08-Jun-2021 12:13:06

%% Input handling

% If dataLines is not specified, define defaults
if nargin < 2
    dataLines = [1, Inf];
end

%% Set up the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 3);

% Specify range and delimiter
opts.DataLines = dataLines;
opts.Delimiter = " ";

% Specify column names and types
opts.VariableNames = ["VarName1", "VarName2", "start_exp"];
opts.VariableTypes = ["double", "double", "char"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";
opts.ConsecutiveDelimitersRule = "join";
opts.LeadingDelimitersRule = "ignore";

% Specify variable properties
opts = setvaropts(opts, "start_exp", "WhitespaceRule", "preserve");
opts = setvaropts(opts, "start_exp", "EmptyFieldRule", "auto");

% Import the data
futek_data = readtable(filename, opts);

%% Convert to output type
futek_data = table2cell(futek_data);
numIdx = cellfun(@(x) ~isnan(str2double(x)), futek_data);
futek_data(numIdx) = cellfun(@(x) {str2double(x)}, futek_data(numIdx));
end