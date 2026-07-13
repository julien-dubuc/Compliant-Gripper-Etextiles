clc; clear; close all;
%% 1. SETTINGS
file_basename   = 'C3_F_'; %C1_F_   C2_F_  C3_F_ 
gripper_label   = regexprep(file_basename, '_F_?$', ''); 
num_trials      = 5;
marker_names    = {'BL', 'TR', 'MRR', 'ML', 'TM', 'TL', 'MLL', 'MR', 'BR'};
num_markers     = length(marker_names);
max_deviation   = 50;   % mm - Max deviation for intra-trial outlier filtering
match_thresh    = 20;   % mm - Max distance to match a column to a marker
header_lines    = 7;    % Motive header lines
% Fixed colors for each marker
colors = [
    0.00, 0.45, 0.74; % BL
    0.85, 0.33, 0.10; % TR
    0.93, 0.69, 0.13; % MRR
    0.49, 0.18, 0.56; % ML
    0.47, 0.67, 0.19; % TM
    0.30, 0.75, 0.93; % TL
    0.64, 0.08, 0.18; % MLL
    0.00, 0.50, 0.50; % MR
    0.95, 0.85, 0.00  % BR
];
% Markers used to detect the max force frame (minimum distance = closed fingers)
force_marker_A = 'BL';
force_marker_B = 'BR';
%% 2. RAW DATA LOADING & SPATIAL MATCHING
% Load raw data
raw_time  = cell(1, num_trials);
raw_coord = cell(1, num_trials);
for trial = 1:num_trials
    filename = sprintf('%s%d.csv', file_basename, trial);
    if ~isfile(filename), continue; end
    data = readmatrix(filename, 'NumHeaderLines', header_lines);
    data(all(isnan(data(:,3:end)), 2), :) = []; % Remove empty rows
    raw_time{trial}  = data(:,2);
    raw_coord{trial} = data(:,3:end);
end
if isempty(raw_coord{1})
    error('Trial 1 is required to build the reference template.');
end
% Build reference template from Trial 1 based on spatial rows (Bottom, Mid, Top)
[tpl_pos, tpl_valid] = get_slot_positions(raw_coord{1}, 0.5);
real_idx = find(tpl_valid);
real_pos = tpl_pos(real_idx, :);
row_layout = { {'BR','BL'}, {'MRR','MR','ML','MLL'}, {'TR','TM','TL'} }; 
[~, order_y] = sort(real_pos(:,2));
gaps = diff(real_pos(order_y,2));
[~, gap_rank] = sort(gaps, 'descend');
cuts = sort(gap_rank(1:2))';  
bounds = [0, cuts, numel(order_y)];
template = nan(num_markers, 2);
name_to_idx = containers.Map(marker_names, num2cell(1:num_markers));
for row = 1:3
    grp = order_y(bounds(row)+1 : bounds(row+1));
    [~, ox] = sort(real_pos(grp,1));
    grp_sorted = grp(ox);
    expected_names = row_layout{row};
    for k = 1:numel(grp_sorted)
        nm = expected_names{k};
        mi = name_to_idx(nm);
        template(mi, :) = real_pos(grp_sorted(k), :);
    end
end
% Match slots to markers for each trial
col_map = nan(num_markers, num_trials);
for trial = 1:num_trials
    if isempty(raw_coord{trial}), continue; end
    [pos, valid] = get_slot_positions(raw_coord{trial}, 0.3);
    [assign, ~] = match_to_template(pos, valid, template, match_thresh);
    for m = 1:num_markers
        if ~isnan(assign(m))
            col_map(m, trial) = (assign(m)-1)*3 + 1;
        end
    end
end
%% 3. PREPROCESSING: OUTLIER FILTERING
X = cell(num_markers, num_trials);
Y = cell(num_markers, num_trials);
T = cell(1, num_trials);
max_force_frame = nan(1, num_trials);
for trial = 1:num_trials
    if isempty(raw_coord{trial}), continue; end
    coords = raw_coord{trial};
    T{trial} = raw_time{trial};
    for m = 1:num_markers
        c = col_map(m, trial);
        if isnan(c)
            X{m,trial} = nan(size(coords,1),1);
            Y{m,trial} = nan(size(coords,1),1);
            continue;
        end
        x = coords(:, c);
        y = coords(:, c+1);
        
        % Filter outliers based on median deviation
        mX = median(x, 'omitnan'); mY = median(y, 'omitnan');
        d  = sqrt((x-mX).^2 + (y-mY).^2);
        x(d > max_deviation) = NaN;
        y(d > max_deviation) = NaN;
        X{m,trial} = x;
        Y{m,trial} = y;
    end
    
    % Calculate max force frame (minimum distance between A and B)
    iA = find(strcmp(marker_names, force_marker_A));
    iB = find(strcmp(marker_names, force_marker_B));
    if ~isnan(col_map(iA,trial)) && ~isnan(col_map(iB,trial))
        dAB = sqrt((X{iA,trial}-X{iB,trial}).^2 + (Y{iA,trial}-Y{iB,trial}).^2);
        dAB(isnan(dAB)) = Inf;
        [~, mf] = min(dAB);
        max_force_frame(trial) = mf;
    else
        max_force_frame(trial) = size(coords,1);
    end
end
%% 4. KINEMATIC TRAJECTORIES PLOT
figure('Name', ['Gripper ' gripper_label ' Kinematics'], 'Color', 'w', 'Position', [100, 100, 1000, 800]);
hold on; grid on; axis equal;
xlabel('X (mm)'); ylabel('Y (mm)');
h_trajectories = gobjects(1, num_markers);
for trial = 1:num_trials
    if isempty(raw_coord{trial}), continue; end
    mf = max_force_frame(trial);
    for m = 1:num_markers
        if isnan(col_map(m,trial)), continue; end
        Xi = -X{m,trial}(1:mf);   % Invert X axis
        Yi =  Y{m,trial}(1:mf);
        valid = ~isnan(Xi) & ~isnan(Yi);
        if ~any(valid), continue; end
        
        h_line = plot(Xi(valid), Yi(valid), '-', 'Color', colors(m,:), ...
            'LineWidth', 1.5, 'DisplayName', marker_names{m});
        if trial == 1, h_trajectories(m) = h_line; end
        
        first_i = find(valid, 1, 'first');
        last_i  = find(valid, 1, 'last');
        scatter(Xi(first_i), Yi(first_i), 30, 'o', 'MarkerEdgeColor', 'k', ...
            'MarkerFaceColor', colors(m,:), 'HandleVisibility', 'off');
        scatter(Xi(last_i), Yi(last_i), 80, 'hexagram', 'MarkerEdgeColor', 'k', ...
            'MarkerFaceColor', colors(m,:), 'HandleVisibility', 'off');
    end
end
ok = arrayfun(@(h) isgraphics(h), h_trajectories);
h_start = scatter(NaN, NaN, 30, 'o', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', [0.5 0.5 0.5]);
h_end   = scatter(NaN, NaN, 80, 'hexagram', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', [0.5 0.5 0.5]);
legend([h_trajectories(ok), h_start, h_end], [marker_names(ok), {'Start Position', 'Max Force'}], ...
    'Location', 'eastoutside', 'FontSize', 10);
title(sprintf('Gripper %s: Max Force Kinematics (5 Trials)', gripper_label), 'FontSize', 14);
%% 5. Y(t) EVOLUTION PLOTS (VERTICAL "BUILDING" STACK)
% Display order: Top to Bottom
vertical_order = {'TL', 'TM', 'TR', 'MLL', 'ML', 'MR', 'MRR', 'BL', 'BR'};
% Create a tall vertical figure
figure('Name', ['Gripper ' gripper_label ' Y(t) Stack'], 'Color', 'w', 'Position', [100, 50, 900, 1200]);
% tiledlayout for zero-spacing stacking
t_layout = tiledlayout(length(vertical_order), 1, 'TileSpacing', 'none', 'Padding', 'compact');
title(t_layout, sprintf('Gripper %s: Y(t) Evolution', gripper_label), 'FontSize', 14, 'FontWeight', 'bold');
xlabel(t_layout, 'Time (s)', 'FontSize', 12);
ylabel(t_layout, 'Y (mm)', 'FontSize', 12);
gap_s = 1.0; % Gap duration between trials in seconds
for k = 1:length(vertical_order)
    m_name = vertical_order{k};
    m = find(strcmp(marker_names, m_name)); % Find index
    
    ax = nexttile;
    hold(ax, 'on'); grid(ax, 'on');
    
    t_offset = 0;
    baseline_ref = [];
    all_y_in_row = []; % To track min/max for dynamic margins
    
    for trial = 1:num_trials
        if isnan(col_map(m, trial)) || isempty(T{trial})
            t_offset = t_offset + gap_s;
            continue; 
        end
        
        t = T{trial} - T{trial}(1) + t_offset;
        y = Y{m, trial};
        valid = ~isnan(y);
        
        if ~any(valid)
            t_offset = t_offset + (T{trial}(end)-T{trial}(1)) + gap_s;
            continue;
        end
        
        all_y_in_row = [all_y_in_row; y(valid)];
        
        % Plot line
        plot(ax, t(valid), y(valid), '-', 'Color', colors(m,:), 'LineWidth', 1.5);
        
        % Dotted line separating trials
        xline(ax, t_offset, ':', 'Color', [0.7 0.7 0.7]);
        
        % Baseline
        if isempty(baseline_ref)
            n0 = max(1, round(0.01*sum(valid)));
            idxv = find(valid, n0, 'first');
            baseline_ref = median(y(idxv));
            yline(ax, baseline_ref, '--', 'Color', [0.85 0.2 0.2 0.5]);
        end
        
        t_offset = t_offset + (T{trial}(end)-T{trial}(1)) + gap_s;
    end
    
    % Marker label in the top-left of the subplot
    text(ax, 0.01, 0.85, m_name, 'Units', 'normalized', 'FontWeight', 'bold', ...
         'FontSize', 10, 'Color', colors(m,:), 'BackgroundColor', 'w', 'EdgeColor', 'k');
         
    xlim(ax, [0, max(t_offset - gap_s, 1)]);
    
    % --- OVERLAP FIX (UNIVERSAL VERSION) ---
    if ~isempty(all_y_in_row)
        % 1. Add 15% padding to Y limits so data doesn't touch the very edge
        y_min = min(all_y_in_row);
        y_max = max(all_y_in_row);
        y_range = y_max - y_min;
        if y_range == 0, y_range = 1; end % Fallback safety
        ylim(ax, [y_min - 0.15*y_range, y_max + 0.15*y_range]);
    end
    
    % 2. Limit the number of Y-ticks by manually filtering the generated ticks
    current_ticks = ax.YTick;
    if length(current_ticks) >= 3
        % Keep only the first, middle, and last tick to reduce edge clutter
        ax.YTick = [current_ticks(1), current_ticks(round(end/2)), current_ticks(end)];
    end
    
    % Remove X-axis labels for all but the last graph
    if k < length(vertical_order)
        xticklabels(ax, {});
    end
end
%% 6. EXPORT GRAPHICS
disp('Exporting images...');
exportgraphics(figure(1), 'C3_pos.png', 'Resolution', 600);
exportgraphics(figure(2), 'C3_Y.png', 'Resolution', 600);
disp('Export complete!');
%% LOCAL FUNCTIONS
function [pos, valid] = get_slot_positions(coords, min_frac)
    % Returns median (X,Y) and validity mask for each slot
    if nargin < 2, min_frac = 0.5; end
    n_frames = size(coords, 1);
    n_slots = size(coords, 2) / 3;
    pos = nan(n_slots, 2);
    valid = false(n_slots, 1);
    
    for i = 1:n_slots
        c = (i-1)*3 + 1;
        x = coords(:, c);
        y = coords(:, c+1);
        m = ~isnan(x);
        if sum(m)/n_frames >= min_frac
            pos(i,:) = [median(x(m)), median(y(m))];
            valid(i) = true;
        end
    end
end
function [assign, dist] = match_to_template(pos, valid, template, thresh)
    % Nearest-neighbor assignment based on spatial distance
    num_markers = size(template, 1);
    slot_idx = find(valid);
    real_pos = pos(slot_idx, :);
    assign = nan(num_markers, 1);
    dist   = nan(num_markers, 1);
    
    if isempty(slot_idx), return; end
    
    D = sqrt((template(:,1) - real_pos(:,1)').^2 + (template(:,2) - real_pos(:,2)').^2);
    used_slots = false(size(real_pos,1), 1);
    used_markers = false(num_markers, 1);
    
    [sorted_d, order] = sort(D(:));
    [mi_list, si_list] = ind2sub(size(D), order);
    
    for k = 1:numel(sorted_d)
        mi = mi_list(k); si = si_list(k);
        if used_markers(mi) || used_slots(si), continue; end
        if sorted_d(k) > thresh, continue; end
        
        assign(mi) = slot_idx(si);
        dist(mi) = sorted_d(k);
        used_markers(mi) = true;
        used_slots(si) = true;
    end
end