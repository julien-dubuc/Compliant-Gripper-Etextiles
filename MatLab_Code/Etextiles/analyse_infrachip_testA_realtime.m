%% analyse_infrachip_testA_csv_filtered_preload.m
% Analyse one CSV file collected from the InfraChip Test A-like Arduino routine.
%
% Expected CSV columns:
%   time_s, arduino_ms, force_N, rx_ohm, state, step, target_strain_pct
%
% Analysis:
%   - Both force_N and rx_ohm are filtered before extracting step values.
%   - PRELOAD_HOLD at 0.1 N is included as an additional measurement point.
%   - Preload and intermediate strain steps: skip first 4 s of the hold and
%     average the following 5 s.
%   - Final 5% step: skip first 10 s of FINAL_HOLD and average the following 10 s.
%
% Outputs:
%   1) *_step_summary_filtered_preload.csv
%   2) *_with_filtered_signals.csv
%   3) *_analysis_plot_filtered_preload.png
%   4) *_mean_values_barplot_preload.png
%
% Filtering method:
%   - moving median removes short spikes
%   - moving mean removes medium/high-frequency noise
%
% This avoids requiring the Signal Processing Toolbox.

clear; clc;

%% ---------------- USER SETTINGS ----------------

% Preload and intermediate steps:
standardSkipSec = 4;
standardWindowSec = 5;

% Final step:
finalSkipSec = 10;
finalWindowSec = 10;

finalTargetPct = 5.0;
targetTolerancePct = 0.02;

% Force stability warning threshold in each averaging window.
forceStdWarningN = 0.10;

% ---------------- FILTER SETTINGS ----------------

forceMedianWindowSec = 0.5;
forceMeanWindowSec   = 1.5;

rxMedianWindowSec = 0.5;
rxMeanWindowSec   = 1.5;

% Use filtered data for summary values.
useFilteredForSummary = true;

%% ---------------- SELECT CSV FILE ----------------

[fileName, folderName] = uigetfile('*.csv', 'Select InfraChip Test A CSV file');

if isequal(fileName, 0)
    error('No file selected.');
end

csvPath = fullfile(folderName, fileName);
fprintf('Reading:\n%s\n\n', csvPath);

T = readtable(csvPath);

%% ---------------- CHECK REQUIRED COLUMNS ----------------

requiredColumns = {'time_s', 'arduino_ms', 'force_N', 'rx_ohm', ...
                   'state', 'step', 'target_strain_pct'};

for i = 1:numel(requiredColumns)
    if ~ismember(requiredColumns{i}, T.Properties.VariableNames)
        error('Missing required column: %s', requiredColumns{i});
    end
end

T.state = string(T.state);

validRows = isfinite(T.time_s) & ...
            isfinite(T.force_N) & ...
            isfinite(T.rx_ohm) & ...
            isfinite(T.target_strain_pct);

T = T(validRows, :);

if isempty(T)
    error('No valid rows found after filtering invalid numerical data.');
end

%% ---------------- ESTIMATE SAMPLING RATE ----------------

dt = median(diff(T.time_s));

if ~isfinite(dt) || dt <= 0
    error('Could not estimate sampling interval from time_s.');
end

fs = 1 / dt;

fprintf('Estimated sampling frequency: %.2f Hz\n', fs);

forceMedianWindowSamples = max(1, round(forceMedianWindowSec * fs));
forceMeanWindowSamples   = max(1, round(forceMeanWindowSec * fs));

rxMedianWindowSamples = max(1, round(rxMedianWindowSec * fs));
rxMeanWindowSamples   = max(1, round(rxMeanWindowSec * fs));

if mod(forceMedianWindowSamples, 2) == 0
    forceMedianWindowSamples = forceMedianWindowSamples + 1;
end

if mod(rxMedianWindowSamples, 2) == 0
    rxMedianWindowSamples = rxMedianWindowSamples + 1;
end

fprintf('Force filter: movmedian %d samples, then movmean %d samples\n', ...
    forceMedianWindowSamples, forceMeanWindowSamples);
fprintf('Rx filter:    movmedian %d samples, then movmean %d samples\n\n', ...
    rxMedianWindowSamples, rxMeanWindowSamples);

%% ---------------- FILTER SIGNALS ----------------

T.force_N_raw = T.force_N;
T.rx_ohm_raw = T.rx_ohm;

force_med = movmedian(T.force_N_raw, forceMedianWindowSamples, 'omitnan');
rx_med = movmedian(T.rx_ohm_raw, rxMedianWindowSamples, 'omitnan');

T.force_N_filt = movmean(force_med, forceMeanWindowSamples, 'omitnan');
T.rx_ohm_filt = movmean(rx_med, rxMeanWindowSamples, 'omitnan');

if useFilteredForSummary
    forceForAnalysis = T.force_N_filt;
    rxForAnalysis = T.rx_ohm_filt;
    summarySignalLabel = 'filtered';
else
    forceForAnalysis = T.force_N_raw;
    rxForAnalysis = T.rx_ohm_raw;
    summarySignalLabel = 'raw';
end

fprintf('Summary values will be calculated from %s signals.\n\n', summarySignalLabel);

%% ---------------- IDENTIFY CONTINUOUS SEGMENTS ----------------

state = T.state;
target = T.target_strain_pct;

segmentStartIdx = [];
segmentEndIdx = [];

startIdx = 1;

for i = 2:height(T)
    stateChanged = state(i) ~= state(i-1);
    targetChanged = abs(target(i) - target(i-1)) > targetTolerancePct;

    if stateChanged || targetChanged
        segmentStartIdx(end+1, 1) = startIdx; %#ok<SAGROW>
        segmentEndIdx(end+1, 1) = i - 1; %#ok<SAGROW>
        startIdx = i;
    end
end

segmentStartIdx(end+1, 1) = startIdx;
segmentEndIdx(end+1, 1) = height(T);

%% ---------------- ANALYSE STEP WINDOWS ----------------

summaryRows = struct( ...
    'step', {}, ...
    'target_strain_pct', {}, ...
    'label', {}, ...
    'state', {}, ...
    'segment_start_s', {}, ...
    'segment_end_s', {}, ...
    'window_start_s', {}, ...
    'window_end_s', {}, ...
    'n_samples', {}, ...
    'mean_rx_ohm', {}, ...
    'std_rx_ohm', {}, ...
    'mean_force_N', {}, ...
    'std_force_N', {}, ...
    'min_force_N', {}, ...
    'max_force_N', {}, ...
    'force_range_N', {}, ...
    'force_stability_flag', {}, ...
    'signal_used', {} ...
);

for seg = 1:numel(segmentStartIdx)

    idx1 = segmentStartIdx(seg);
    idx2 = segmentEndIdx(seg);

    segState = string(T.state(idx1));
    segTarget = T.target_strain_pct(idx1);
    segStep = T.step(idx1);

    isPreloadHold = segState == 'PRELOAD_HOLD';
    isIntermediateHold = segState == 'STEP_HOLD';
    isFinalHold = segState == 'FINAL_HOLD';
    isForceLimitHold = segState == 'FORCE_LIMIT_HOLD';

    if ~(isPreloadHold || isIntermediateHold || isFinalHold || isForceLimitHold)
        continue;
    end

    segStartTime = T.time_s(idx1);
    segEndTime = T.time_s(idx2);
    segDuration = segEndTime - segStartTime;

    if isPreloadHold
        skipSec = standardSkipSec;
        windowSec = standardWindowSec;
        stepLabel = "0.1 N preload";
    elseif isFinalHold || isForceLimitHold || abs(segTarget - finalTargetPct) <= targetTolerancePct
        skipSec = finalSkipSec;
        windowSec = finalWindowSec;
        if isForceLimitHold
            stepLabel = "force limit hold";
        else
            stepLabel = sprintf('%.1f%%', segTarget);
        end
    else
        skipSec = standardSkipSec;
        windowSec = standardWindowSec;
        stepLabel = sprintf('%.1f%%', segTarget);
    end

    winStart = segStartTime + skipSec;
    winEnd = winStart + windowSec;

    segmentMask = false(height(T), 1);
    segmentMask(idx1:idx2) = true;

    windowMask = T.time_s >= winStart & T.time_s <= winEnd & segmentMask;

    n = sum(windowMask);

    if n > 0
        rxMean = mean(rxForAnalysis(windowMask), 'omitnan');
        rxStd = std(rxForAnalysis(windowMask), 'omitnan');

        forceMean = mean(forceForAnalysis(windowMask), 'omitnan');
        forceStd = std(forceForAnalysis(windowMask), 'omitnan');
        forceMin = min(forceForAnalysis(windowMask));
        forceMax = max(forceForAnalysis(windowMask));
        forceRange = forceMax - forceMin;

        if forceStd > forceStdWarningN
            forceFlag = 'CHECK_FORCE_STABILITY';
        else
            forceFlag = 'OK';
        end
    else
        rxMean = NaN;
        rxStd = NaN;
        forceMean = NaN;
        forceStd = NaN;
        forceMin = NaN;
        forceMax = NaN;
        forceRange = NaN;

        if segDuration < skipSec
            forceFlag = 'SEGMENT_TOO_SHORT_BEFORE_WINDOW';
        else
            forceFlag = 'NO_SAMPLES_IN_WINDOW';
        end
    end

    row.step = segStep;
    row.target_strain_pct = segTarget;
    row.label = string(stepLabel);
    row.state = segState;
    row.segment_start_s = segStartTime;
    row.segment_end_s = segEndTime;
    row.window_start_s = winStart;
    row.window_end_s = winEnd;
    row.n_samples = n;
    row.mean_rx_ohm = rxMean;
    row.std_rx_ohm = rxStd;
    row.mean_force_N = forceMean;
    row.std_force_N = forceStd;
    row.min_force_N = forceMin;
    row.max_force_N = forceMax;
    row.force_range_N = forceRange;
    row.force_stability_flag = string(forceFlag);
    row.signal_used = string(summarySignalLabel);

    summaryRows(end+1) = row; %#ok<SAGROW>
end

if isempty(summaryRows)
    warning('No PRELOAD_HOLD, STEP_HOLD, FINAL_HOLD, or FORCE_LIMIT_HOLD segments found.');
    Summary = table();
else
    Summary = struct2table(summaryRows);
end

%% ---------------- ADD DELTA-R METRICS ----------------

if ~isempty(Summary)
    validMean = isfinite(Summary.mean_rx_ohm);

    if any(validMean)
        firstIdx = find(validMean, 1, 'first');
        R0 = Summary.mean_rx_ohm(firstIdx);

        Summary.delta_rx_ohm = Summary.mean_rx_ohm - R0;
        Summary.frac_delta_rx = Summary.delta_rx_ohm ./ R0;
        Summary.percent_delta_rx = 100 .* Summary.frac_delta_rx;
    end
end

%% ---------------- DISPLAY AND SAVE SUMMARY ----------------

disp('Summary of resistance and force windows:');
disp(Summary);

[~, baseName, ~] = fileparts(fileName);

summaryCsvPath = fullfile(folderName, [baseName '_step_summary_filtered_preload.csv']);
writetable(Summary, summaryCsvPath);

fprintf('\nSaved summary CSV:\n%s\n', summaryCsvPath);

%% ---------------- SAVE FILTERED FULL DATA ----------------

filteredDataPath = fullfile(folderName, [baseName '_with_filtered_signals.csv']);
writetable(T, filteredDataPath);

fprintf('Saved full data with filtered signals:\n%s\n', filteredDataPath);

%% ---------------- PLOT RAW AND FILTERED DATA ----------------

fig = figure('Name', 'InfraChip Test A filtered analysis with preload', ...
             'NumberTitle', 'off', ...
             'Position', [100 100 1200 750]);

tiledlayout(fig, 2, 1);

ax1 = nexttile;
plot(ax1, T.time_s, T.force_N_raw, '-', 'LineWidth', 0.5);
hold(ax1, 'on');
plot(ax1, T.time_s, T.force_N_filt, '-', 'LineWidth', 1.5);
grid(ax1, 'on');
xlabel(ax1, 'Time (s)');
ylabel(ax1, 'Force (N)');
title(ax1, 'Force: raw and filtered');
legend(ax1, {'Raw', 'Filtered'}, 'Location', 'best');

ax2 = nexttile;
plot(ax2, T.time_s, T.rx_ohm_raw, '-', 'LineWidth', 0.5);
hold(ax2, 'on');
plot(ax2, T.time_s, T.rx_ohm_filt, '-', 'LineWidth', 1.5);
grid(ax2, 'on');
xlabel(ax2, 'Time (s)');
ylabel(ax2, 'Resistance (Ohm)');
title(ax2, 'Resistance: raw and filtered');
legend(ax2, {'Raw', 'Filtered'}, 'Location', 'best');

if ~isempty(Summary)
    for i = 1:height(Summary)
        ws = Summary.window_start_s(i);
        we = Summary.window_end_s(i);
        centreTime = mean([ws, we]);

        xline(ax1, ws, '--');
        xline(ax1, we, ':');

        xline(ax2, ws, '--');
        xline(ax2, we, ':');

        if isfinite(Summary.mean_force_N(i))
            plot(ax1, centreTime, Summary.mean_force_N(i), 'o', 'MarkerSize', 6);
        end

        if isfinite(Summary.mean_rx_ohm(i))
            plot(ax2, centreTime, Summary.mean_rx_ohm(i), 'o', 'MarkerSize', 6);
            text(ax2, centreTime, Summary.mean_rx_ohm(i), "  " + Summary.label(i), ...
                 'VerticalAlignment', 'bottom');
        end
    end
end

linkaxes([ax1, ax2], 'x');

plotPngPath = fullfile(folderName, [baseName '_analysis_plot_filtered_preload.png']);
saveas(fig, plotPngPath);

fprintf('Saved filtered analysis plot:\n%s\n', plotPngPath);

%% ---------------- EXTRA FIGURE: BAR CHART OF MEAN VALUES ----------------

barFig = figure('Name', 'Mean values by preload/strain step', ...
                'NumberTitle', 'off', ...
                'Position', [150 150 1200 650]);

tiledlayout(barFig, 2, 1);

if isempty(Summary)
    warning('No summary data available for bar plot.');
else
    xLabels = cellstr(Summary.label);
    x = 1:height(Summary);

    axB1 = nexttile;
    bar(axB1, x, Summary.mean_rx_ohm);
    hold(axB1, 'on');
    errorbar(axB1, x, Summary.mean_rx_ohm, Summary.std_rx_ohm, ...
        'k.', 'LineWidth', 1);
    grid(axB1, 'on');
    ylabel(axB1, 'Mean resistance (Ohm)');
    title(axB1, 'Mean resistance by preload/strain step');
    set(axB1, 'XTick', x, 'XTickLabel', xLabels);
    xtickangle(axB1, 35);

    axB2 = nexttile;
    bar(axB2, x, Summary.mean_force_N);
    hold(axB2, 'on');
    errorbar(axB2, x, Summary.mean_force_N, Summary.std_force_N, ...
        'k.', 'LineWidth', 1);
    grid(axB2, 'on');
    xlabel(axB2, 'Step');
    ylabel(axB2, 'Mean force (N)');
    title(axB2, 'Mean force in the same analysis windows');
    set(axB2, 'XTick', x, 'XTickLabel', xLabels);
    xtickangle(axB2, 35);
end

barPlotPath = fullfile(folderName, [baseName '_mean_values_barplot_preload.png']);
saveas(barFig, barPlotPath);

fprintf('Saved mean values bar plot:\n%s\n', barPlotPath);

disp('Analysis complete.');
