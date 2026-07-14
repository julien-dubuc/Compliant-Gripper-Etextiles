clc; clear; close all;
file_basename   = 'C1_D_'; % (C1_D_, C2_D_, C3_D_)
gripper_label   = regexprep(file_basename, '_D_?$', '');
num_trials      = 3;
% T=Top, M=Middle, B=Bottom ; L=Left, R=Right ; LL/RR = outer 
marker_names = {'BL', 'TR', 'MRR', 'ML', 'TM', 'TL', 'MLL', 'MR', 'BR'};
num_markers  = length(marker_names);

max_deviation   = 50;   % mm - filtre d'outliers INTRA-essai
match_thresh    = 20;   % mm - distance max pour associer une colonne a un marqueur
header_lines    = 7;

colors = [
    0.0, 0.45, 0.74; % BL
    0.85, 0.33, 0.10; % TR
    0.93, 0.69, 0.13; % MRR
    0.49, 0.18, 0.56; % ML
    0.47, 0.67, 0.19; % TM
    0.30, 0.75, 0.93; % TL
    0.64, 0.08, 0.18; % MLL
    0.00, 0.50, 0.50; % MR
    0.95, 0.85, 0.00  % BR
];

closing_marker_A = 'BL';
closing_marker_B = 'BR';

% --- Parametres de detection des paliers ---
smooth_win_s   = 1.0;  % s - fenetre de lissage (mediane glissante)
slope_win_s    = 1.0;  % s - fenetre de calcul de la pente
slope_thresh   = 0.15; % mm/s - pente max pour etre considere "stable"
plateau_min_dur_s   = 1.0;  % s - duree minimale d'un palier
plateau_merge_gap_s = 0.5;  % s - fusion de paliers proches separes par un court trou
merge_value_tol  = 0.15; % mm
merge_gap_max_s  = 2.0;  % s

raw_time  = cell(1, num_trials);
raw_coord = cell(1, num_trials);

for trial = 1:num_trials
    filename = sprintf('%s%d.csv', file_basename, trial);
    if ~isfile(filename)
        warning('Fichier introuvable : %s -> essai ignore.', filename);
        continue;
    end
    data = readmatrix(filename, 'NumHeaderLines', header_lines);
    data(all(isnan(data(:,3:end)), 2), :) = [];
    raw_time{trial}  = data(:,2);
    raw_coord{trial} = data(:,3:end);
end

if isempty(raw_coord{1})
    error('Le Trial 1 est requis pour construire le gabarit de reference.');
end

fr_names   = {'BG', 'HD', 'MDD', 'MG', 'HM', 'HG', 'MGG', 'MD', 'BD'};
name_map   = containers.Map(fr_names, marker_names); % FR -> EN

[tpl_pos, tpl_valid] = get_slot_positions(raw_coord{1}, 0.5);
real_idx = find(tpl_valid);
real_pos = tpl_pos(real_idx, :);

if numel(real_idx) ~= num_markers
    warning('Trial 1 : %d slots "reels" trouves, %d attendus.', numel(real_idx), num_markers);
end

row_layout = { {'BD','BG'}, {'MDD','MD','MG','MGG'}, {'HD','HM','HG'} };
[~, order_y] = sort(real_pos(:,2));
gaps = diff(real_pos(order_y,2));
[~, gap_rank] = sort(gaps, 'descend');
cuts = sort(gap_rank(1:2))';
bounds = [0, cuts, numel(order_y)];
template = nan(num_markers, 2);
name_to_idx = containers.Map(marker_names, num2cell(1:num_markers));

fprintf('=== Gabarit geometrique (Trial 1) ===\n');
for row = 1:3
    grp = order_y(bounds(row)+1 : bounds(row+1));
    [~, ox] = sort(real_pos(grp,1));
    grp_sorted = grp(ox);
    expected_names_fr = row_layout{row};
    if numel(grp_sorted) ~= numel(expected_names_fr)
        warning('Rangee %d : %d marqueurs trouves, %d attendus.', row, numel(grp_sorted), numel(expected_names_fr));
        continue;
    end
    for k = 1:numel(grp_sorted)
        nm_en = name_map(expected_names_fr{k});
        mi = name_to_idx(nm_en);
        template(mi, :) = real_pos(grp_sorted(k), :);
        fprintf('  %-4s : (%.1f, %.1f)\n', nm_en, template(mi,1), template(mi,2));
    end
end

if any(isnan(template(:,1)))
    error('Gabarit incomplet : %s', strjoin(marker_names(isnan(template(:,1))), ', '));
end

col_map = nan(num_markers, num_trials);
for trial = 1:num_trials
    if isempty(raw_coord{trial}), continue; end
    [pos, valid] = get_slot_positions(raw_coord{trial}, 0.3);
    [assign, dist] = match_to_template(pos, valid, template, match_thresh);
    for m = 1:num_markers
        if isnan(assign(m))
            warning('Trial %d : marqueur %s non retrouve (matching).', trial, marker_names{m});
        else
            col_map(m, trial) = (assign(m)-1)*3 + 1;
            if dist(m) > match_thresh/2
                warning('Trial %d : %s matche a %.1f mm (verifie).', trial, marker_names{m}, dist(m));
            end
        end
    end
end

%  X,Y filtres + metrique d'ouverture + pentes

X = cell(num_markers, num_trials);
Y = cell(num_markers, num_trials);
T = cell(1, num_trials);
aperture     = cell(1, num_trials);  
aperture_slope = cell(1, num_trials);
motion_slope   = cell(1, num_trials); 
fs_trial = nan(1, num_trials);

iA = find(strcmp(marker_names, closing_marker_A));
iB = find(strcmp(marker_names, closing_marker_B));

for trial = 1:num_trials
    if isempty(raw_coord{trial}), continue; end
    coords = raw_coord{trial};
    T{trial} = raw_time{trial};
    fs_trial(trial) = 1 / median(diff(T{trial}), 'omitnan');
    
    for m = 1:num_markers
        c = col_map(m, trial);
        if isnan(c)
            X{m,trial} = nan(size(coords,1),1);
            Y{m,trial} = nan(size(coords,1),1);
            continue;
        end
        x = coords(:, c); y = coords(:, c+1);
        mX = median(x, 'omitnan'); mY = median(y, 'omitnan');
        d  = sqrt((x-mX).^2 + (y-mY).^2);
        x(d > max_deviation) = NaN;
        y(d > max_deviation) = NaN;
        X{m,trial} = x; Y{m,trial} = y;
    end
    
    if ~isnan(col_map(iA,trial)) && ~isnan(col_map(iB,trial))
        aperture{trial} = sqrt((X{iA,trial}-X{iB,trial}).^2 + (Y{iA,trial}-Y{iB,trial}).^2);
    else
        warning('Trial %d : %s ou %s manquant -> pas de metrique d''ouverture.', trial, closing_marker_A, closing_marker_B);
        aperture{trial} = nan(size(coords,1),1);
    end
    
    [aperture_slope{trial}, ~] = slope_magnitude(aperture{trial}, fs_trial(trial), smooth_win_s, slope_win_s);
    n_frames = size(coords,1);
    per_marker_slope = nan(num_markers, n_frames);
    
    for m = 1:num_markers
        sx = slope_magnitude(X{m,trial}, fs_trial(trial), smooth_win_s, slope_win_s);
        sy = slope_magnitude(Y{m,trial}, fs_trial(trial), smooth_win_s, slope_win_s);
        per_marker_slope(m,:) = hypot(sx, sy)';
    end
    motion_slope{trial} = median(per_marker_slope, 1, 'omitnan')';
end

% DETECTION DES PALIERS : "Grip min" et "Grip max" uniquement
stage_range = cell(1, num_trials);
stage_label = cell(1, num_trials);
stage_value = cell(1, num_trials);

fprintf('\n=== Detection des paliers par essai (Grip min / Grip max) ===\n');
for trial = 1:num_trials
    if isempty(aperture{trial}) || all(isnan(aperture{trial}))
        warning('Trial %d : aucune donnee d''ouverture exploitable.', trial);
        continue;
    end
    
    stable = (aperture_slope{trial} < slope_thresh) & (motion_slope{trial} < slope_thresh) & ...
        ~isnan(aperture_slope{trial}) & ~isnan(motion_slope{trial});
        
    plats = mask_to_runs(stable, fs_trial(trial), plateau_min_dur_s, plateau_merge_gap_s);
    plats = merge_plateaus_by_value(plats, aperture{trial}, fs_trial(trial), merge_value_tol, merge_gap_max_s);
    
    n_plat_total = size(plats, 1);
    if n_plat_total < 2
        warning('Trial %d : seulement %d palier(s) detecte(s) -> essai ignore.', trial, n_plat_total);
        continue;
    end
    
    vals_all = nan(n_plat_total, 1);
    for p = 1:n_plat_total
        vals_all(p) = median(aperture{trial}(plats(p,1):plats(p,2)), 'omitnan');
    end
    
    [~, idx_max] = min(vals_all); 
    if idx_max < 2
        warning(['Trial %d : le palier d''ouverture minimale est le PREMIER palier detecte ' ...
            '-> aucun palier "Grip min" avant lui -> essai ignore.'], trial);
        continue;
    end
    
    plats  = plats(idx_max-1 : idx_max, :);
    labels = {'Grip min', 'Grip max'};
    vals   = [vals_all(idx_max-1); vals_all(idx_max)];
    
    fprintf('--- Trial %d (%d paliers au total detectes) ---\n', trial, n_plat_total);
    for k = 1:2
        fprintf('  %-9s : t=[%.2f, %.2f]s  aperture=%.2f mm  (n=%d frames)\n', ...
            labels{k}, T{trial}(plats(k,1))-T{trial}(1), T{trial}(plats(k,2))-T{trial}(1), ...
            vals(k), plats(k,2)-plats(k,1)+1);
    end
    stage_range{trial} = plats;
    stage_label{trial} = labels;
    stage_value{trial} = vals;
end

%  POSITION (X,Y) DE CHAQUE MARQUEUR A CHAQUE PALIER
stage_pos = cell(1, num_trials);
for trial = 1:num_trials
    if isempty(stage_range{trial}), continue; end
    n_plat = size(stage_range{trial}, 1);
    pos = nan(num_markers, 2, n_plat);
    for k = 1:n_plat
        r = stage_range{trial}(k,:);
        for m = 1:num_markers
            pos(m,1,k) = median(X{m,trial}(r(1):r(2)), 'omitnan');
            pos(m,2,k) = median(Y{m,trial}(r(1):r(2)), 'omitnan');
        end
    end
    stage_pos{trial} = pos;
end

%% ========================================================================
%  6. TABLEAU DES DEPLACEMENTS PAR MARQUEUR (Grip min -> Grip max)
%  ========================================================================
fprintf('\n=== Deplacement de chaque marqueur : Grip min -> Grip max (mm) ===\n');
for trial = 1:num_trials
    if isempty(stage_pos{trial}), continue; end
    pos = stage_pos{trial};
    fprintf('--- Trial %d ---\n', trial);
    for m = 1:num_markers
        dx = pos(m,1,2) - pos(m,1,1);
        dy = pos(m,2,2) - pos(m,2,1);
        dmag = hypot(dx, dy);
        fprintf('    %-4s : dX=%+6.2f  dY=%+6.2f  |d|=%5.2f mm\n', marker_names{m}, dx, dy, dmag);
    end
end

% FIGURE 1 : POSITIONS Grip min / Grip max (3 essais)

figure('Name', ['Gripper ' gripper_label ' - Test B'], 'Color', 'w', 'Position', [100 100 1000 800]);
hold on; grid on; axis equal;
xlabel('X (mm)'); ylabel('Y (mm)');
title(sprintf('Gripper %s : Grip min / Grip max positions (%d trials)', gripper_label, num_trials), 'FontSize', 13);
stage_markers = {'^', 'p'}; 
h_leg_marker = gobjects(1, num_markers);

for trial = 1:num_trials
    if isempty(stage_pos{trial}), continue; end
    pos = stage_pos{trial};
    for m = 1:num_markers
        Xi = -pos(m,1,:); Xi = Xi(:); 
        Yi =  pos(m,2,:); Yi = Yi(:);
        valid = ~isnan(Xi);
        if sum(valid) < 1, continue; end
        
        h = plot(Xi(valid), Yi(valid), '-', 'Color', [colors(m,:) 0.5], 'LineWidth', 1.3, 'HandleVisibility','off');
        if trial == 1 && ~isgraphics(h_leg_marker(m))
            h_leg_marker(m) = plot(NaN, NaN, 'o', 'Color', colors(m,:), 'MarkerFaceColor', colors(m,:), ...
                'DisplayName', marker_names{m});
        end
        for k = 1:2
            if isnan(Xi(k)), continue; end
            scatter(Xi(k), Yi(k), 45, stage_markers{k}, 'MarkerEdgeColor', 'k', ...
                'MarkerFaceColor', colors(m,:), 'HandleVisibility', 'off');
        end
    end
end
ok = arrayfun(@isgraphics, h_leg_marker);
h_s1 = scatter(NaN,NaN,45,'^','MarkerEdgeColor','k','MarkerFaceColor',[.5 .5 .5]);
h_s2 = scatter(NaN,NaN,45,'p','MarkerEdgeColor','k','MarkerFaceColor',[.5 .5 .5]);
legend([h_leg_marker(ok), h_s1, h_s2], [marker_names(ok), {'Grip min','Grip max'}], ...
    'Location', 'eastoutside', 'FontSize', 9);

% FIGURE 2 : Y(t) PAR MARQUEUR EN SOUS-GRAPHES
gap_s = 1.0;
figure('Name', 'Y evolution - All Markers', 'Color', 'w', 'Position', [100 50 1000 1000]);
t_layout = tiledlayout(num_markers, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
title(t_layout, sprintf('Gripper %s : Y evolution from Grip min to Grip max (%d trials)', ...
    gripper_label, num_trials), 'FontSize', 14, 'FontWeight', 'bold');

% L'ORDRE D'AFFICHAGE
desired_order = {'TL', 'TM', 'TR', 'MLL', 'ML', 'MR', 'MRR', 'BL', 'BR'};

for step = 1:num_markers
    % l'index d'origine 'm' correspondant au nom dans desired_order
    m = find(strcmp(marker_names, desired_order{step}));
    
    nexttile; 
    hold on; grid on;
    
    t_offset = 0;
    y_all_valid = []; % Pour ajuster l'axe Y proprement plus tard
    
    for trial = 1:num_trials
        if isnan(col_map(m,trial)) || isempty(T{trial}) || isempty(stage_range{trial})
            continue;
        end
        plats = stage_range{trial};
        labels = stage_label{trial};
        i0 = plats(1,1); i1 = plats(end,2);
        t_full = T{trial} - T{trial}(1);
        t_vec = t_full(i0:i1) - t_full(i0) + t_offset; 
        y = Y{m,trial}(i0:i1);
        valid = ~isnan(y);
        
        if any(valid)
            plot(t_vec(valid), y(valid), '-', 'Color', colors(m,:), 'LineWidth', 1.5, 'HandleVisibility','off');
            y_all_valid = [y_all_valid; y(valid)];
        end
        
        % Séparateur entre les essais 
        if trial < num_trials
            xline(t_offset + (t_full(i1)-t_full(i0)) + gap_s/2, '-', 'Color', [0.8 0.8 0.8], 'LineWidth', 1.5, 'HandleVisibility','off');
        end
            
        for k = 1:size(plats,1)
            tk0 = t_full(plats(k,1)) - t_full(i0) + t_offset;
            yk = median(Y{m,trial}(plats(k,1):plats(k,2)), 'omitnan');
            
            xline(tk0, '--', 'Color', [0.85 0.2 0.2 0.4], 'HandleVisibility','off');
            
            % Texte "Grip min/max"
            text(tk0 + 0.2, yk, labels{k}, 'FontSize', 7, 'Color', [0.6 0 0], 'VerticalAlignment', 'bottom');
        end
        t_offset = t_offset + (t_full(i1)-t_full(i0)) + gap_s;
    end
    
    xlim([0, max(t_offset-gap_s, 1)]);
    
    ylabel({sprintf('\\bf{%s}', marker_names{m}); 'Y (mm)'}, 'FontSize', 9);
    
    % NETTOYAGE AXE Y : on force 3 graduations maximum 
    if ~isempty(y_all_valid)
        y_min = min(y_all_valid);
        y_max = max(y_all_valid);
        y_margin = (y_max - y_min) * 0.2; % 20% de marge
        if y_margin == 0, y_margin = 1; end
        
        ylim([y_min - y_margin, y_max + y_margin]);
        yticks(linspace(y_min, y_max, 3)); 
        yticklabels(string(round(yticks, 1))); % Arrondi à 1 décimale
    end
    
    set(gca, 'FontSize', 8);
    
    % On cache l'axe X pour tous sauf le dernier graph
    if step == num_markers
        xlabel('Time (s)', 'FontSize', 11, 'FontWeight', 'bold');
    else
        xticklabels({});
    end
end
exportgraphics(figure(2), 'C1_Deformation.png', 'Resolution', 600);

%  LOCAL FUNCTIONS
function [pos, valid] = get_slot_positions(coords, min_frac)
    if nargin < 2, min_frac = 0.5; end
    n_frames = size(coords, 1);
    n_slots = size(coords, 2) / 3;
    pos = nan(n_slots, 2);
    valid = false(n_slots, 1);
    for i = 1:n_slots
        c = (i-1)*3 + 1;
        x = coords(:, c); y = coords(:, c+1);
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
        assign(mi) = slot_idx(si); dist(mi) = sorted_d(k);
        used_markers(mi) = true; used_slots(si) = true;
    end
end

function [slope, valid] = slope_magnitude(sig, fs, smooth_win_s, slope_win_s)
    valid = ~isnan(sig);
    filled = sig;
    if any(~valid)
        filled(~valid) = median(sig, 'omitnan');
    end
    smooth_win = max(3, round(smooth_win_s*fs));
    smooth = movmedian(filled, smooth_win);
    slope_win = max(1, round(slope_win_s*fs));
    n = numel(smooth);
    slope = nan(n,1);
    if n > slope_win
        slope(slope_win+1:end) = (smooth(slope_win+1:end) - smooth(1:end-slope_win)) / slope_win_s;
    end
    slope = abs(slope);
end

function plateaus = mask_to_runs(stable, fs, min_dur_s, merge_gap_s)
    d = diff([0; stable(:); 0]);
    starts = find(d == 1);
    ends   = find(d == -1) - 1;
    runs = [starts, ends];
    merged = zeros(0,2);
    for k = 1:size(runs,1)
        if ~isempty(merged) && (runs(k,1) - merged(end,2)) < merge_gap_s*fs
            merged(end,2) = runs(k,2);
        else
            merged = [merged; runs(k,:)]; 
        end
    end
    if isempty(merged)
        plateaus = merged;
        return;
    end
    dur = (merged(:,2) - merged(:,1)) / fs;
    plateaus = merged(dur >= min_dur_s, :);
end

function plateaus = merge_plateaus_by_value(plateaus, aperture, fs, value_tol, time_gap_max_s)
    if size(plateaus,1) < 2
        return;
    end
    merged = plateaus(1,:);
    vals = median(aperture(plateaus(1,1):plateaus(1,2)), 'omitnan');
    for k = 2:size(plateaus,1)
        r = plateaus(k,:);
        v = median(aperture(r(1):r(2)), 'omitnan');
        gap_s = (r(1) - merged(end,2)) / fs;
        if abs(v - vals(end)) < value_tol && gap_s < time_gap_max_s
            merged(end,2) = r(2);
        else
            merged = [merged; r]; 
            vals(end+1) = v; 
        end
    end
    plateaus = merged;
end