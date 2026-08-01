function save_all_figures_to_pdf(pdfFileName)
%SAVE_ALL_FIGURES_TO_PDF Export every open figure into one multi-page PDF.
%
% USAGE:
%   1) Run your main simulation script first (lqi_03_Final), so that
%      whichever figures you want (Figure 1..6, the dashboard window,
%      etc.) are open on screen.
%   2) Then, from the MATLAB command window, run:
%
%           save_all_figures_to_pdf
%
%      or give it a specific output filename:
%
%           save_all_figures_to_pdf('MyResults.pdf')
%
% WHAT THIS DOES:
%   - Finds every open figure window (regular figures AND uifigures).
%   - If a figure contains a uitabgroup (like the one-window dashboard
%     viewer in lqi_03_Final), it exports EACH TAB as its own page,
%     instead of only the tab that happens to be selected.
%   - Exports everything, in order, into ONE combined PDF using
%     exportgraphics with vector content where possible.
%   - Does NOT close or modify your open figures.
%
% This script is intentionally separate from the simulation script so it
% can be reused for any project: it does not depend on any variable
% (trajTitle, torqueDist, etc.) existing in the base workspace.

    %% 1) Resolve output folder and filename.
    % Portable output location: an 'LQI_Results' folder beside this file.
    % Edit this one line if you prefer a fixed path of your own.
    outputFolder = fullfile(fileparts(mfilename('fullpath')), 'LQI_Results');
    if ~exist(outputFolder, 'dir')
        mkdir(outputFolder);
    end

    if nargin < 1 || isempty(pdfFileName)
        timeStamp = datestr(now, 'yyyy-mm-dd_HHMMSS');
        pdfFileName = sprintf('All_Figures_%s.pdf', timeStamp);
    end
    if ~endsWith(pdfFileName, '.pdf', 'IgnoreCase', true)
        pdfFileName = [pdfFileName, '.pdf'];
    end
    pdfFullPath = fullfile(outputFolder, pdfFileName);

    % Start clean: exportgraphics with 'Append' keeps adding pages to an
    % existing file, so delete any leftover file with the same name first.
    if exist(pdfFullPath, 'file')
        delete(pdfFullPath);
    end

    %% 2) Collect all open figures (regular figures + uifigures/dashboards).
    allFigs = findall(groot, 'Type', 'figure');
    if isempty(allFigs)
        warning('save_all_figures_to_pdf:NoFigures', ...
            'No open figures were found. Nothing was exported.');
        return;
    end

    % Sort by the figure "Number" property so pages come out in a
    % predictable order (Figure 1, Figure 2, ..., dashboard windows last
    % if they were opened after the numbered figures).
    figNumbers = arrayfun(@(f) getSortableNumber(f), allFigs);
    [~, sortIdx] = sort(figNumbers);
    allFigs = allFigs(sortIdx);

    %% 3) Export each figure. Expand tab groups into one page per tab.
    pagesWritten = 0;
    fprintf('Exporting figures to PDF:\n%s\n', pdfFullPath);

    for k = 1:numel(allFigs)
        fig = allFigs(k);
        if ~isgraphics(fig)
            continue;   % figure was closed while we were working
        end

        tabGroups = findobj(fig, 'Type', 'uitabgroup');

        if isempty(tabGroups)
            % Plain figure (or uifigure without tabs): export as-is.
            pagesWritten = exportOnePage(fig, pdfFullPath, pagesWritten);
        else
            % Dashboard-style figure: export one page per tab so every
            % segment (position tracking, errors, 3D view, etc.) is kept.
            tg = tabGroups(1);
            originalTab = tg.SelectedTab;
            allTabs = tg.Children;   % uitab objects, in creation order

            for tIdx = 1:numel(allTabs)
                if ~isgraphics(fig)
                    break;
                end
                tg.SelectedTab = allTabs(tIdx);
                drawnow;   % let MATLAB actually render the newly selected tab

                % IMPORTANT: export the uitab OBJECT itself, not the parent
                % figure. exportgraphics(fig, ...) fails with "Figure has
                % more than one container" because all 15 tabs are still
                % children of the tabgroup even when only one is visible.
                % Exporting the tab directly captures just its contents.
                pagesWritten = exportOnePage(allTabs(tIdx), pdfFullPath, pagesWritten, ...
                    allTabs(tIdx).Title, fig);
            end

            % Restore whichever tab was selected before we started.
            if isgraphics(tg) && isgraphics(originalTab)
                tg.SelectedTab = originalTab;
            end
        end
    end

    if pagesWritten == 0
        warning('save_all_figures_to_pdf:NothingExported', ...
            'No pages were written. Check that figures were actually open.');
    else
        fprintf('Done. %d page(s) written to:\n%s\n', pagesWritten, pdfFullPath);
    end
end

%% ---- Local helper functions ----

function pagesWritten = exportOnePage(graphicsObj, pdfFullPath, pagesWritten, pageLabel, parentFigForName)
%EXPORTONEPAGE Append one graphics object (a whole figure, OR a single
% uitab object) as a page in the target PDF. Wrapped in try/catch so one
% bad figure/tab does not stop the export of everything else.
%
%   graphicsObj        - the object actually passed to exportgraphics
%                         (a figure handle, or a uitab handle)
%   parentFigForName    - optional: the parent figure, used only to build
%                         a readable name in the printed log / warnings
    if nargin < 4
        pageLabel = '';
    end
    if nargin < 5 || isempty(parentFigForName)
        nameSource = graphicsObj;
    else
        nameSource = parentFigForName;
    end

    try
        appendFlag = exist(pdfFullPath, 'file') == 2;
        exportgraphics(graphicsObj, pdfFullPath, 'Append', appendFlag, ...
            'ContentType', 'vector');
        pagesWritten = pagesWritten + 1;
        if isempty(pageLabel)
            fprintf('  Page %d: "%s"\n', pagesWritten, safeFigName(nameSource));
        else
            fprintf('  Page %d: "%s" - tab "%s"\n', pagesWritten, safeFigName(nameSource), pageLabel);
        end
    catch ME
        warning('save_all_figures_to_pdf:ExportFailed', ...
            'Could not export figure "%s" (%s). Skipping. Reason: %s', ...
            safeFigName(nameSource), pageLabel, ME.message);
    end
end

function name = safeFigName(fig)
%SAFEFIGNAME Return a readable name for a figure, even if Name is empty.
    name = get(fig, 'Name');
    if isempty(name)
        name = sprintf('Figure %s', num2str(getSortableNumber(fig)));
    end
end

function num = getSortableNumber(fig)
%GETSORTABLENUMBER Return a numeric sort key for a figure handle.
% Regular figures have an integer Number. Some uifigures/dashboards may
% report an empty or non-numeric Number; those are pushed to the end.
    try
        n = fig.Number;
        if isempty(n) || ~isnumeric(n)
            num = Inf;
        else
            num = double(n);
        end
    catch
        num = Inf;
    end
end
