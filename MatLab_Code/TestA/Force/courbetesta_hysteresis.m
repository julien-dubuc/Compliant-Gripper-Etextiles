clc; clear; close all;

%% 1. SETTINGS
file_basename   = 'CM_F_'; % C1_F_, C2_F_, C3_F_, CM_F_
gripper_label   = regexprep(file_basename, '_F_?$', '');
num_trials      = 5;
marker_names    = {'BL', 'TR', 'MRR', 'ML', 'TM', 'TL', 'MLL', 'MR', 'BR'};
num_markers     = length(marker_names);
max_deviation   = 50;  % mm - Max deviation for intra-trial outlier filtering
match_thresh    = 20;  % mm - Max distance to match a column to a marker
header_lines    = 7;   % Motive header lines
fps             = 200; % Capture frame rate

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

force_marker_A = 'BL';
force_marker_B = 'BR';

%% 2. RAW DATA LOADING & SPATIAL MATCHING
raw_time  = cell(1, num_trials);
raw_coord = cell(1, num_trials);
for trial = 1:num_trials
    filename = sprintf('%s%d.csv', file_basename, trial);
    if ~isfile(filename), continue; end
    data = readmatrix(filename, 'NumHeaderLines', header_lines);
    data(all(isnan(data(:,3:end)), 2), :) = []; 
    raw_time{trial}  = data(:,2);
    raw_coord{trial} = data(:,3:end);
end

if isempty(raw_coord{1})
    error('Trial 1 is required to build the reference template.');
end

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

%% 3. PREPROCESSING: OUTLIER FILTERING + LOCAL DESPIKING
despike_window  = 9; % frames, odd
despike_k       = 4; % local MAD threshold

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
        
        % Global filter
        mX = median(x, 'omitnan'); mY = median(y, 'omitnan');
        d  = sqrt((x-mX).^2 + (y-mY).^2);
        x(d > max_deviation) = NaN;
        y(d > max_deviation) = NaN;
        
        % Local despiking for high-frequency tracking jitter
        x = despike_local(x, despike_window, despike_k);
        y = despike_local(y, despike_window, despike_k);
        X{m,trial} = x;
        Y{m,trial} = y;
    end
    
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
        Xi = -X{m,trial}(1:mf);
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

%% 5. Y(t) EVOLUTION PLOTS (VERTICAL STACK)
vertical_order = {'TL', 'TM', 'TR', 'MLL', 'ML', 'MR', 'MRR', 'BL', 'BR'};
figure('Name', ['Gripper ' gripper_label ' Y(t) Stack'], 'Color', 'w', 'Position', [100, 50, 900, 1200]);
t_layout = tiledlayout(length(vertical_order), 1, 'TileSpacing', 'none', 'Padding', 'compact');
title(t_layout, sprintf('Gripper %s: Y(t) Evolution', gripper_label), 'FontSize', 14, 'FontWeight', 'bold');
xlabel(t_layout, 'Time (s)', 'FontSize', 12);
ylabel(t_layout, 'Y (mm)', 'FontSize', 12);
gap_s = 1.0;

for k = 1:length(vertical_order)
    m_name = vertical_order{k};
    m = find(strcmp(marker_names, m_name));
    ax = nexttile;
    hold(ax, 'on'); grid(ax, 'on');
    t_offset = 0;
    baseline_ref = [];
    all_y_in_row = [];
    
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
        plot(ax, t(valid), y(valid), '-', 'Color', colors(m,:), 'LineWidth', 1.5);
        xline(ax, t_offset, ':', 'Color', [0.7 0.7 0.7]);
        
        if isempty(baseline_ref)
            n0 = max(1, round(0.01*sum(valid)));
            idxv = find(valid, n0, 'first');
            baseline_ref = median(y(idxv));
            yline(ax, baseline_ref, '--', 'Color', [0.85 0.2 0.2 0.5]);
        end
        t_offset = t_offset + (T{trial}(end)-T{trial}(1)) + gap_s;
    end
    text(ax, 0.01, 0.85, m_name, 'Units', 'normalized', 'FontWeight', 'bold', ...
         'FontSize', 10, 'Color', colors(m,:), 'BackgroundColor', 'w', 'EdgeColor', 'k');
    xlim(ax, [0, max(t_offset - gap_s, 1)]);
    
    if ~isempty(all_y_in_row)
        y_min = min(all_y_in_row);
        y_max = max(all_y_in_row);
        y_range = y_max - y_min;
        if y_range == 0, y_range = 1; end
        ylim(ax, [y_min - 0.15*y_range, y_max + 0.15*y_range]);
    end
    current_ticks = ax.YTick;
    if length(current_ticks) >= 3
        ax.YTick = [current_ticks(1), current_ticks(round(end/2)), current_ticks(end)];
    end
    if k < length(vertical_order)
        xticklabels(ax, {});
    end
end

%% 6. TRIAL TREND: OPEN POSITION vs MAX-FORCE POSITION
plateau_win_s     = 0.3;   % s
plateau_win       = round(plateau_win_s*fps);
open_search_frac  = 0.6;   
close_half_win_s  = 0.125; % s
close_half_win    = round(close_half_win_s*fps);
cross_trial_k     = 5;     
cross_trial_floor = 0.03;  % mm

y_open  = nan(num_markers, num_trials);
x_open  = nan(num_markers, num_trials);
y_close = nan(num_markers, num_trials);
open_plateau_win = round(1.2*fps);

for m = 1:num_markers
    for trial = 1:num_trials
        if isnan(col_map(m,trial)), continue; end
        y  = Y{m,trial};
        x  = X{m,trial};
        mf = max_force_frame(trial);
        search_end = max(open_plateau_win, round(open_search_frac*mf));
        [val, ~, w0, w1] = best_plateau(y, 1, search_end, open_plateau_win);
        y_open(m,trial) = val;
        x_open(m,trial) = median(x(w0:w1), 'omitnan');
    end
end

close_noise_thresh  = 0.10; 
close_fallback_win  = round(0.5*fps);
close_fallback_band = round(1.0*fps);

for m = 1:num_markers
    for trial = 1:num_trials
        if isnan(col_map(m,trial)), continue; end
        y  = Y{m,trial};
        mf = max_force_frame(trial);
        w0 = max(1, mf-close_half_win); w1 = min(numel(y), mf+close_half_win);
        [val, sd] = best_plateau(y, w0, w1, plateau_win);
        
        if ~isnan(sd) && sd > close_noise_thresh
            w0b = max(1, mf-close_fallback_band); w1b = min(numel(y), mf+close_fallback_band);
            [val_fb, ~] = best_plateau(y, w0b, w1b, close_fallback_win);
            if ~isnan(val_fb)
                val = val_fb;
            end
        end
        y_close(m,trial) = val;
    end
end

open_ok  = cross_trial_flag(y_open, cross_trial_k, cross_trial_floor);
close_ok = cross_trial_flag(y_close, cross_trial_k, cross_trial_floor);

%% 6b. HYSTERESIS / DEFORMATION SUMMARY
closing_amp = y_close - y_open;
open_drift  = hypot(x_open - x_open(:,1), y_open - y_open(:,1));

figure('Name', ['Gripper ' gripper_label ' Hysteresis Summary'], 'Color', 'w', 'Position', [150, 80, 1100, 500]);
tl2 = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile; hold on; grid on;
for m = 1:num_markers
    plot(1:num_trials, closing_amp(m,:), '-o', 'Color', colors(m,:), ...
        'LineWidth', 1.5, 'MarkerFaceColor', colors(m,:), 'MarkerSize', 4, 'DisplayName', marker_names{m});
end
title('Closing amplitude (MaxForce - Open)', 'FontSize', 11);
xlabel('Trial number'); ylabel('Position Y (mm)'); xlim([0.5, num_trials+0.5]); xticks(1:num_trials);
legend('Location', 'eastoutside', 'FontSize', 8);

nexttile; hold on; grid on;
for m = 1:num_markers
    plot(1:num_trials, open_drift(m,:), '-', 'Color', [colors(m,:) 0.35], 'LineWidth', 1, 'HandleVisibility', 'off');
end
drift_mean = mean(open_drift, 1, 'omitnan');
plot(1:num_trials, drift_mean, '-o', 'Color', [0.1 0.1 0.1], 'LineWidth', 2.5, ...
    'MarkerFaceColor', [0.1 0.1 0.1], 'MarkerSize', 5, 'DisplayName', 'Mean (all markers)');
title('Cumulative drift |Open(trial n) - Open(trial 1)|', 'FontSize', 11);
xlabel('Trial number'); ylabel('Position Y (mm)'); xlim([0.5, num_trials+0.5]); xticks(1:num_trials);
ylim([0, max(open_drift(:), [], 'omitnan')*1.15]);
legend('Location', 'best', 'FontSize', 8);

vertical_order = {'TL', 'TM', 'TR', 'MLL', 'ML', 'MR', 'MRR', 'BL', 'BR'};
figure('Name', ['Gripper ' gripper_label ' Trial Trend'], 'Color', 'w', 'Position', [150, 80, 1100, 900]);
tl = tiledlayout(3, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, sprintf('Gripper %s: Open vs Max-Force Position Across the 5 Trials', gripper_label), ...
    'FontSize', 14, 'FontWeight', 'bold');
xlabel(tl, 'Trial number', 'FontSize', 12);
ylabel(tl, 'Position Y (mm)', 'FontSize', 12);

for k = 1:length(vertical_order)
    m_name = vertical_order{k};
    m = find(strcmp(marker_names, m_name));
    nexttile; hold on; grid on;
    plot(1:num_trials, y_open(m,:), '-o', 'Color', [0.20 0.45 0.85], 'LineWidth', 1.8, ...
        'MarkerFaceColor', [0.20 0.45 0.85], 'DisplayName', 'Open');
    plot(1:num_trials, y_close(m,:), '-o', 'Color', [0.85 0.20 0.20], 'LineWidth', 1.8, ...
        'MarkerFaceColor', [0.85 0.20 0.20], 'DisplayName', 'Max Force');
    title(m_name, 'FontSize', 11);
    xlim([0.5, num_trials+0.5]); xticks(1:num_trials);
    if k == 1
        legend('Location', 'best', 'FontSize', 8);
    end
end

%% 8. EXPORT GRAPHICS
disp('Exporting images...');
exportgraphics(figure(1), [gripper_label '_pos.png'], 'Resolution', 600);
exportgraphics(figure(2), [gripper_label '_Y.png'], 'Resolution', 600);
exportgraphics(figure(3), [gripper_label '_Hysteresis.png'], 'Resolution', 600);
exportgraphics(figure(4), [gripper_label '_TrialTrend.png'], 'Resolution', 600);
disp('Export complete!');

%% LOCAL FUNCTIONS

function [pos, valid] = get_slot_positions(coords, min_frac)
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

function y = despike_local(y, win, k)
    n = numel(y);
    if n < win, return; end
    half = floor(win/2);
    y_out = y;
    for i = 1:n
        i0 = max(1, i-half); i1 = min(n, i+half);
        w = y(i0:i1);
        w = w(~isnan(w));
        if numel(w) < 3, continue; end
        med = median(w);
        mad_ = median(abs(w - med));
        if mad_ < 1e-6, mad_ = 1e-3; end
        if abs(y(i) - med) > k*mad_ && abs(y(i)-med) > 0.02
            y_out(i) = NaN;
        end
    end
    y = y_out;
end

function ok = cross_trial_flag(vals, k, floor_mm)
    [num_markers, num_trials] = size(vals);
    ok = false(num_markers, num_trials);
    for m = 1:num_markers
        v = vals(m,:);
        good = ~isnan(v);
        if sum(good) < 3
            ok(m, good) = true;
            continue;
        end
        med = median(v(good));
        mad_ = median(abs(v(good) - med));
        thresh = max(floor_mm, k*mad_);
        ok(m, good) = abs(v(good) - med) <= thresh;
    end
end

function [val, sd, w0, w1] = best_plateau(y, i0, i1, win)
    i0 = max(1, round(i0)); i1 = min(numel(y), round(i1));
    win = min(win, max(3, i1-i0+1));
    if i1 - i0 + 1 < win
        seg = y(i0:i1); seg = seg(~isnan(seg));
        w0 = i0; w1 = i1;
        if isempty(seg), val = NaN; sd = NaN; return; end
        val = median(seg); sd = std(seg);
        return;
    end
    step = max(1, round(win/4));
    best_sd = Inf; best_val = NaN; w0 = i0; w1 = i0+win-1;
    for s = i0:step:(i1-win+1)
        seg = y(s:s+win-1);
        seg = seg(~isnan(seg));
        if numel(seg) < round(0.5*win), continue; end
        sd = std(seg);
        if sd < best_sd
            best_sd = sd;
            best_val = median(seg);
            w0 = s; w1 = s+win-1;
        end
    end
    val = best_val; sd = best_sd;
    if isinf(sd), sd = NaN; end
end