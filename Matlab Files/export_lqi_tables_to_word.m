function export_lqi_tables_to_word()
%EXPORT_LQI_TABLES_TO_WORD Export LQI performance tables to a Microsoft Word file.
%
% How to use:
%   1) First run your main LQI simulation file, for example:
%        clear; clear functions; clc;
%        lqi_03_Final
%
%   2) Then run this exporter:
%        export_lqi_tables_to_word
%
% Required workspace tables from the LQI code:
%   rmseSummaryTable
%   itaeSummaryTable
%   steadyStateErrorSummaryTable
%
% Optional workspace table:
%   rawITAEAccumulatedTable
%
% Notes:
%   - This script creates a .docx file using Microsoft Word ActiveX.
%   - It works on Windows with Microsoft Word installed.
%   - If Microsoft Word is not installed, use the CSV fallback at the bottom
%     of this file or install MATLAB Report Generator and adapt accordingly.

%% USER OPTIONS
outputFile = fullfile(pwd, 'LQI_Performance_Tables.docx');
wordVisible = true;              % true = show Word while exporting; false = run in background
includeRawITAEAccumulated = false; % true = also export raw ITAE table with m*s^2/rad*s^2 units
openAfterSave = true;            % true = leave/open the Word file after exporting
numberFormat = '%.10g';          % numeric format inside Word tables

%% GET TABLES FROM BASE WORKSPACE
requiredTables = {'rmseSummaryTable', 'itaeSummaryTable', 'steadyStateErrorSummaryTable'};
for k = 1:numel(requiredTables)
    if ~evalin('base', sprintf('exist(''%s'', ''var'')', requiredTables{k}))
        error(['Required table "%s" was not found in the MATLAB base workspace.\n' ...
               'Run your LQI simulation first, then run export_lqi_tables_to_word again.'], requiredTables{k});
    end
end

rmseSummaryTable = evalin('base','rmseSummaryTable');
itaeSummaryTable = evalin('base','itaeSummaryTable');
steadyStateErrorSummaryTable = evalin('base','steadyStateErrorSummaryTable');

hasRawITAE = evalin('base','exist(''rawITAEAccumulatedTable'', ''var'')');
if hasRawITAE
    rawITAEAccumulatedTable = evalin('base','rawITAEAccumulatedTable'); %#ok<NASGU>
end

%% START MICROSOFT WORD
try
    word = actxserver('Word.Application');
catch ME
    error(['Could not start Microsoft Word using ActiveX.\n' ...
           'This exporter requires Windows + Microsoft Word installed.\n' ...
           'Original error: %s'], ME.message);
end

cleanupObj = onCleanup(@() safeQuitWord(word, openAfterSave));
word.Visible = wordVisible;
doc = word.Documents.Add;
selection = word.Selection;

%% DOCUMENT TITLE
insertTitle(selection, 'LQI Quadcopter Tracking Performance Tables');
insertNormalText(selection, sprintf('Generated on: %s\n', datestr(now)));
insertNormalText(selection, 'The tables below are exported from the MATLAB workspace after running the LQI torque-disturbance simulation.\n\n');

%% TABLE 1: RMSE
insertSectionTitle(selection, 'Table 1. RMSE Summary');
insertNormalText(selection, 'RMSE values include position channels and roll/pitch/yaw attitude channels.\n');
insertWordTable(selection, rmseSummaryTable, numberFormat);

%% TABLE 2: ITAE
insertSectionTitle(selection, 'Table 2. ITAE Summary');
insertNormalText(selection, ['The ITAE values in this table are normalized to physical units: ' ...
    'meters for position and radians for attitude.\n']);
insertWordTable(selection, itaeSummaryTable, numberFormat);

%% TABLE 3: STEADY-STATE ERROR
insertSectionTitle(selection, 'Table 3. Steady-State Error Summary');
insertNormalText(selection, 'Steady-state values are calculated over the final 10 percent of the simulation time.\n');
insertWordTable(selection, steadyStateErrorSummaryTable, numberFormat);

%% OPTIONAL TABLE 4: RAW ACCUMULATED ITAE
if includeRawITAEAccumulated && hasRawITAE
    insertSectionTitle(selection, 'Table 4. Raw Accumulated ITAE');
    insertNormalText(selection, ['This optional table shows raw accumulated ITAE. ' ...
        'Units are m*s^2 for position and rad*s^2 for attitude.\n']);
    insertWordTable(selection, rawITAEAccumulatedTable, numberFormat);
end

%% SAVE FILE
if exist(outputFile,'file')
    delete(outputFile);
end
doc.SaveAs2(outputFile);

fprintf('\nWord export completed successfully.\n');
fprintf('Saved file:\n%s\n', outputFile);

if ~openAfterSave
    doc.Close(false);
    word.Quit;
else
    word.Visible = true;
end

end

%% ========================================================================
% LOCAL HELPER FUNCTIONS
% ========================================================================

function insertTitle(selection, txt)
    selection.Font.Name = 'Times New Roman';
    selection.Font.Size = 16;
    selection.Font.Bold = true;
    selection.TypeText(txt);
    selection.TypeParagraph;
    selection.Font.Bold = false;
    selection.Font.Size = 12;
end

function insertSectionTitle(selection, txt)
    selection.TypeParagraph;
    selection.Font.Name = 'Times New Roman';
    selection.Font.Size = 13;
    selection.Font.Bold = true;
    selection.TypeText(txt);
    selection.TypeParagraph;
    selection.Font.Bold = false;
    selection.Font.Size = 11;
end

function insertNormalText(selection, txt)
    selection.Font.Name = 'Times New Roman';
    selection.Font.Size = 11;
    selection.Font.Bold = false;
    selection.TypeText(txt);
end

function insertWordTable(selection, T, numberFormat)
    % Convert MATLAB table to cell text data.
    headers = T.Properties.VariableNames;
    nRows = height(T) + 1;
    nCols = width(T);

    % Add Word table at current selection.
    wordTable = selection.Document.Tables.Add(selection.Range, nRows, nCols);
    wordTable.Borders.Enable = 1;

    % Header row.
    for c = 1:nCols
        wordTable.Cell(1,c).Range.Text = makeReadableHeader(headers{c});
        wordTable.Cell(1,c).Range.Bold = true;
    end

    % Data rows.
    for r = 1:height(T)
        for c = 1:nCols
            value = T{r,c};
            wordTable.Cell(r+1,c).Range.Text = valueToText(value, numberFormat);
        end
    end

    % Formatting.
    try
        wordTable.Rows.Item(1).Shading.BackgroundPatternColor = hex2wordColor('D9EAF7');
        wordTable.Rows.Item(1).Range.Font.Bold = true;
    catch
    end

    try
        wordTable.Range.Font.Name = 'Times New Roman';
        wordTable.Range.Font.Size = 10;
        wordTable.AutoFitBehavior(2); % wdAutoFitWindow
    catch
    end

    % Move cursor after table.
    selection.EndKey(6); % wdStory
    selection.TypeParagraph;
end

function s = valueToText(value, numberFormat)
    % Handle cells, strings, chars, numeric values, logicals.
    if iscell(value)
        if isempty(value)
            s = '';
        else
            s = valueToText(value{1}, numberFormat);
        end
    elseif isstring(value)
        if isscalar(value)
            s = char(value);
        else
            s = strjoin(cellstr(value), ', ');
        end
    elseif ischar(value)
        s = value;
    elseif isnumeric(value)
        if isempty(value)
            s = '';
        elseif isscalar(value)
            s = sprintf(numberFormat, value);
        else
            s = mat2str(value, 6);
        end
    elseif islogical(value)
        s = string(value);
        s = char(s);
    else
        try
            s = char(string(value));
        catch
            s = '<unsupported>';
        end
    end
end

function header = makeReadableHeader(header)
    % Improve Word headings without changing MATLAB variable names.
    header = strrep(header, '_', ' ');
    header = regexprep(header, '([a-z])([A-Z])', '$1 $2');
    header = strtrim(header);
end

function safeQuitWord(word, openAfterSave)
    % Avoid leaving hidden Word processes open if an error occurs.
    try
        if ~openAfterSave
            word.Quit;
        end
    catch
    end
end

function color = hex2wordColor(hex)
    % Convert RGB hex string into Word BGR integer color.
    % Example: 'D9EAF7'
    r = hex2dec(hex(1:2));
    g = hex2dec(hex(3:4));
    b = hex2dec(hex(5:6));
    color = r + 256*g + 65536*b;
end
