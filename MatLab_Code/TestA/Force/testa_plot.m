clc; clear; close all;

%% 1. SETTINGS & INITIALIZATION
gripper_prefixes = {'C1_F_', 'C2_F_', 'C3_F_'};
gripper_labels   = {'Baseline', '-20% Dist', '-40% Dist'};
gripper_colors   = [
    0.00, 0.45, 0.74; % C1: Blue
    0.85, 0.33, 0.10; % C2: Red / Orange
    0.47, 0.67, 0.19  % C3: Green
];
num_trials       = 5;

% Nomenclature
marker_names     = {'BL', 'TR', 'MRR', 'ML', 'TM', 'TL', 'MLL', 'MR', 'BR'};
num_markers      = length(marker_names);
idx_BR = find(strcmp(marker_names, 'BR'));
idx_MR = find(strcmp(marker_names, 'MR'));
idx_BL = find(strcmp(marker_names, 'BL'));
idx_ML = find(strcmp(marker_names, 'ML'));

max_deviation    = 50;   
match_thresh     = 20;   
header_lines     = 7;    

%% 2. PROCESS ALL GRIPPERS
all_mean_X = cell(3, num_markers);
all_mean_Y = cell(3, num_markers);
all_start_X = nan(3, num_markers);
all_start_Y = nan(3, num_markers);
all_end_X   = nan(3, num_markers);
all_end_Y   = nan(3, num_markers);

for g = 1:3
    file_basename = gripper_prefixes{g};
    
    % Temporary storage for the 5 trials
    T_X = cell(num_markers, num_trials);
    T_Y = cell(num_markers, num_trials);
    
    for trial = 1:num_trials
        filename = sprintf('%s%d.csv', file_basename, trial);
        if ~isfile(filename), continue; end
        
        data = readmatrix(filename, 'NumHeaderLines', header_lines);
        data(all(isnan(data(:,3:end)), 2), :) = [];
        raw_coord = data(:,3:end);
        
        if trial == 1
            [tpl_pos, tpl_valid] = get_slot_positions(raw_coord, 0.5);
            real_pos = tpl_pos(find(tpl_valid), :);
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
                for k = 1:numel(grp_sorted)
                    template(name_to_idx(row_layout{row}{k}), :) = real_pos(grp_sorted(k), :);
                end
            end
        end
        
        [pos, valid] = get_slot_positions(raw_coord, 0.3);
        [assign, ~] = match_to_template(pos, valid, template, match_thresh);
        
        % Extract max force frame (min distance BL-BR)
        cA = (assign(idx_BL)-1)*3 + 1;
        cB = (assign(idx_BR)-1)*3 + 1;
        if ~isnan(cA) && ~isnan(cB)
            dAB = sqrt((raw_coord(:,cA)-raw_coord(:,cB)).^2 + (raw_coord(:,cA+1)-raw_coord(:,cB+1)).^2);
            [~, mf] = min(dAB);
        else
            mf = size(raw_coord,1);
        end
        
        % Store valid trajectories
        for m = 1:num_markers
            if isnan(assign(m)), continue; end
            c = (assign(m)-1)*3 + 1;
            x = raw_coord(1:mf, c); y = raw_coord(1:mf, c+1);
            
            % Remove outliers
            mX = median(x, 'omitnan'); mY = median(y, 'omitnan');
            d = sqrt((x-mX).^2 + (y-mY).^2);
            x(d > max_deviation) = NaN; y(d > max_deviation) = NaN;
            
            T_X{m, trial} = -x; % Invert X axis as in original code
            T_Y{m, trial} = y;
        end
    end
    
    % Calculate Mean Trajectory for this gripper
    for m = 1:num_markers
        valid_trials = 0;
        min_len = Inf;
        
        % Find shortest trajectory length to average them safely
        for trial = 1:num_trials
            if ~isempty(T_X{m, trial})
                min_len = min(min_len, length(T_X{m, trial}));
            end
        end
        
        if isinf(min_len), continue; end
        
        agg_X = zeros(min_len, 1);
        agg_Y = zeros(min_len, 1);
        
        for trial = 1:num_trials
            if ~isempty(T_X{m, trial})
                % Simple padding/alignment (taking the first min_len points)
                % A more complex DTW could be used, but this is usually sufficient for monotonic closing
                x_clean = fillmissing(T_X{m, trial}(1:min_len), 'linear');
                y_clean = fillmissing(T_Y{m, trial}(1:min_len), 'linear');
                agg_X = agg_X + x_clean;
                agg_Y = agg_Y + y_clean;
                valid_trials = valid_trials + 1;
            end
        end
        
        if valid_trials > 0
            all_mean_X{g, m} = agg_X / valid_trials;
            all_mean_Y{g, m} = agg_Y / valid_trials;
            all_start_X(g, m) = all_mean_X{g, m}(1);
            all_start_Y(g, m) = all_mean_Y{g, m}(1);
            all_end_X(g, m)   = all_mean_X{g, m}(end);
            all_end_Y(g, m)   = all_mean_Y{g, m}(end);
        end
    end
end

%% 3. CALCULATE AND PRINT DISPLACEMENT RATIOS (LOAD SIDE)
fprintf('=== KINEMATIC EFFICIENCY RATIOS (LOAD SIDE) ===\n');
fprintf('Ratio = (Load Cell Finger Displacement BL) / (Hinge Displacement ML)\n');
fprintf('Higher ratio = More energy goes to closing the fingers instead of bending the structure.\n\n');

for g = 1:3
    % Displacement of finger under load (BL)
    dx_BL = all_end_X(g, idx_BL) - all_start_X(g, idx_BL);
    dy_BL = all_end_Y(g, idx_BL) - all_start_Y(g, idx_BL);
    disp_BL = sqrt(dx_BL^2 + dy_BL^2);
    
    % Displacement of hinge under load (ML)
    dx_ML = all_end_X(g, idx_ML) - all_start_X(g, idx_ML);
    dy_ML = all_end_Y(g, idx_ML) - all_start_Y(g, idx_ML);
    disp_ML = sqrt(dx_ML^2 + dy_ML^2);
    
    ratio = disp_BL / disp_ML;
    
    fprintf('%s :\n', gripper_labels{g});
    fprintf('  - BL Displacement: %5.2f mm\n', disp_BL);
    fprintf('  - ML Displacement: %5.2f mm\n', disp_ML);
    fprintf('  - EFFICIENCY RATIO: %5.2f\n\n', ratio);
end

%% 4. PLOT COMPARATIVE TRAJECTORIES
figure('Name', 'Comparative Kinematics', 'Color', 'w', 'Position', [100, 100, 1000, 800]);
hold on; grid on; axis equal;
xlabel('X Position (mm)', 'FontSize', 12, 'FontWeight', 'bold'); 
ylabel('Y Position (mm)', 'FontSize', 12, 'FontWeight', 'bold');

% Create dummy objects for the legend
h_leg = gobjects(1, 3);

for g = 1:3
    c = gripper_colors(g, :);
    
    for m = 1:num_markers
        if isempty(all_mean_X{g, m}), continue; end
        
        Xi = all_mean_X{g, m};
        Yi = all_mean_Y{g, m};
        
        % Plot line
        h = plot(Xi, Yi, '-', 'Color', c, 'LineWidth', 2.0, 'HandleVisibility', 'off');
        if m == 1, h_leg(g) = h; end % Save for legend
        
        % Start point (circle)
        scatter(Xi(1), Yi(1), 30, 'o', 'MarkerEdgeColor', c, 'MarkerFaceColor', 'w', 'HandleVisibility', 'off');
        % End point (hexagram)
        scatter(Xi(end), Yi(end), 80, 'hexagram', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', c, 'HandleVisibility', 'off');
        
        % --- LABELS INTELLIGENTS ET SUR-MESURE (Seulement pour Baseline) ---
        if g == 1
            % On definit l'ecartement ideal POUR CHAQUE marqueur precisement
            switch marker_names{m}
                case 'TL'
                    align_h = 'right'; off_x = -5; off_y = 6;
                case 'TM'
                    align_h = 'center'; off_x = 0; off_y = 14;
                case 'TR'
                    align_h = 'left'; off_x = 4; off_y = 6;
                case 'MLL'
                    align_h = 'right'; off_x = -4; off_y = 0;
                case 'ML'
                    align_h = 'right'; off_x = -2; off_y = -6;
                case 'MR'
                    align_h = 'left'; off_x = 4; off_y = -6;
                case 'MRR'
                    align_h = 'left'; off_x = 6; off_y = 0;
                case 'BL'
                    align_h = 'right'; off_x = -4; off_y = -2;
                case 'BR'
                    align_h = 'left'; off_x = 4; off_y = -2;
                otherwise
                    align_h = 'center'; off_x = 0; off_y = 5;
            end
            
            % Affichage avec fond blanc pour masquer les lignes
            text(Xi(1) + off_x, Yi(1) + off_y, marker_names{m}, ...
                'FontSize', 9, 'FontWeight', 'bold', 'Color', 'k', ...
                'HorizontalAlignment', align_h, ...
                'BackgroundColor', 'w', 'EdgeColor', [0.8 0.8 0.8], 'Margin', 2);
        end
    end
end

% Set strict universal limits so it's perfectly comparable
all_valid_X = [all_start_X(:); all_end_X(:)];
all_valid_Y = [all_start_Y(:); all_end_Y(:)];

% On ajoute une grande marge de 25 mm pour être sûr que les textes ne soient pas coupés
xlim([min(all_valid_X)-25, max(all_valid_X)+25]);
ylim([min(all_valid_Y)-25, max(all_valid_Y)+25]);

h_start = scatter(NaN, NaN, 30, 'o', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'w');
h_end   = scatter(NaN, NaN, 80, 'hexagram', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k');

legend([h_leg, h_start, h_end], [gripper_labels, {'Start Position', 'Max Force'}], ...
    'Location', 'best', 'FontSize', 11);
title('Comparative Kinematics (Mean of 5 Trials)', 'FontSize', 14, 'FontWeight', 'bold');

%% 5. EXPORT
disp('Exporting comparative image...');
exportgraphics(figure(1), 'Comparative_Kinematics.png', 'Resolution', 600);
disp('Export complete! Check the console for the Displacement Ratios.');


%% LOCAL FUNCTIONS
function [pos, valid] = get_slot_positions(coords, min_frac)
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

function [X_norm, Y_norm] = ds2nfu(X, Y)
    ax = gca;
    xlim = ax.XLim;
    ylim = ax.YLim;
    pos = ax.Position;
    
    X_norm = pos(1) + (X - xlim(1)) / (xlim(2) - xlim(1)) * pos(3);
    Y_norm = pos(2) + (Y - ylim(1)) / (ylim(2) - ylim(1)) * pos(4);
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