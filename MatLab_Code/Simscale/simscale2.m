clc; clear; close all;
% SimScale export files (full mesh field, final state = max force/displacement).
% FIXED columns by POSITION:
%   1-6  : Cauchy stress tensor (unused)
%   7-9  : Displacement X,Y,Z
%   10-15: Strain tensor (unused)
%   16   : von Mises stress
%   17-19: Node coordinates X,Y,Z (CAD reference)
files = struct( ...
    'label', {'C1 (Baseline)', 'C2 (-20%)', 'C3 (-40%)'}, ...
    'path',  {'c1.csv', 'c2.csv', 'c3.csv'}, ...
    'color', {[0 0.45 0.74], [0.85 0.33 0.10], [0.47 0.67 0.19]} );

num_bins = 25; % Number of bins along the WHOLE gripper (0=fixed constraint, 1=tip)
tip_frac = 0.97;  % Threshold defining the "tip zone" (last 3%)

% Real flexible-finger length (base-to-tip
% to mark the rigid->flexible transition on the plots -
% the profile itself covers the WHOLE gripper (rigid mount + fingers).
FLEX_LENGTH_MM = struct('C1', 100.0, 'C2', 89.8, 'C3', 79.6);

%  LOADING & NORMALIZED PROFILES EXTRACTION
% Y_FIXED = SimScale fixed constraint (top of the gripper / rigid mount).
% Constant across all 3 variants (verified: -137.92mm identically) since
% only the finger tip-to-middle geometry changes between designs.
% Normalizing on this gives the WHOLE gripper (0=fixed base, 1=fingertip),
% so the plot shows both the rigid section (flat/low-stress) AND the
% flexible fingers (rising displacement, stress peak at the transition).
Y_FIXED = -0.13792;

n_files = numel(files);
profiles = cell(1, n_files);
raw = cell(1, n_files);

fprintf('=== Loading and extracting profiles (whole gripper) ===\n');
for i = 1:n_files
    fn = files(i).path;
    if ~isfile(fn)
        warning('File not found: %s', fn);
        continue;
    end
    data = readmatrix(fn, 'NumHeaderLines', 1);

    dispX = data(:,7); dispY = data(:,8); dispZ = data(:,9);
    vonMises = data(:,16);
    X = data(:,17); Y = data(:,18); Z = data(:,19);

    disp_mag = sqrt(dispX.^2 + dispY.^2 + dispZ.^2);
    y_tip = max(Y);
    y_norm = (Y - Y_FIXED) / (y_tip - Y_FIXED);

    % Position (in %) of the rigid->flexible transition for THIS design.
    % Same absolute distance from Y_FIXED on every design (~71mm), but a
    % different % since total gripper length differs.
    flex_len = FLEX_LENGTH_MM.(files(i).label(1:2));
    y_transition = y_tip - flex_len/1000;
    transition_pct = (y_transition - Y_FIXED) / (y_tip - Y_FIXED) * 100;

    raw{i} = struct('X',X, 'Y',Y, 'Z',Z, 'y_norm',y_norm, ...
        'disp_mag',disp_mag, 'vonMises',vonMises, 'y_tip',y_tip, ...
        'transition_pct', transition_pct);

    % --- Binned profile along the WHOLE gripper length ---
    edges = linspace(0, 1, num_bins+1);
    bin_idx = discretize(y_norm, edges);
    bin_center = (edges(1:end-1) + edges(2:end)) / 2;

    disp_max = accumarray(bin_idx(~isnan(bin_idx)), disp_mag(~isnan(bin_idx)), [num_bins,1], @max, NaN);
    vm_max   = accumarray(bin_idx(~isnan(bin_idx)), vonMises(~isnan(bin_idx)), [num_bins,1], @max, NaN);

    profiles{i} = struct('bin_center', bin_center(:), 'disp_max', disp_max, 'vm_max', vm_max);

    fprintf(['  %-14s : %d nodes, total length = %.1f mm, rigid mount = %.1f mm, ' ...
        'flexible fingers = %.1f mm (transition at %.1f%% of total length)\n'], ...
        files(i).label, numel(Y), (y_tip-Y_FIXED)*1000, (y_transition-Y_FIXED)*1000, flex_len, transition_pct);
end

%  DIAGNOSTICS: PEAK STRESS & TIP DISPLACEMENT
fprintf('\n=== von Mises Peak Stress (Location & Magnitude) ===\n');
for i = 1:n_files
    if isempty(raw{i}), continue; end
    r = raw{i};
    [vm_peak, idx] = max(r.vonMises);
    fprintf('  %-14s : %.1f MPa at %.1f%% of total length (X=%.1f, Y=%.1f, Z=%.1f mm)\n', ...
        files(i).label, vm_peak/1e6, r.y_norm(idx)*100, r.X(idx)*1000, r.Y(idx)*1000, r.Z(idx)*1000);
end

fprintf('\n=== Max Displacement in Tip Zone (last %.0f%% of total length) ===\n', (1-tip_frac)*100);
for i = 1:n_files
    if isempty(raw{i}), continue; end
    r = raw{i};
    mask = r.y_norm > tip_frac;
    fprintf('  %-14s : %.2f mm (n=%d nodes)\n', files(i).label, max(r.disp_mag(mask))*1000, sum(mask));
end

% FIGURE 1: COMPARATIVE PROFILES (whole gripper)
fig1 = figure('Name', 'SimScale Profile Comparison', 'Color', 'w', 'Position', [100 100 1150 460]);

subplot(1,2,1); hold on; grid on;
for i = 1:n_files
    if isempty(profiles{i}), continue; end
    p = profiles{i};
    plot(p.bin_center*100, p.disp_max*1000, '-o', 'Color', files(i).color, ...
        'LineWidth', 1.6, 'MarkerSize', 4, 'DisplayName', files(i).label);
    xline(raw{i}.transition_pct, '--', 'Color', files(i).color, 'HandleVisibility', 'off');
end
xlabel('Normalized Gripper Length (%) | 0=Fixed base, 100=Fingertip');
ylabel('Max Displacement (mm)');
title('Displacement Profile');
legend('Location','northwest'); box on;

subplot(1,2,2); hold on; grid on;
for i = 1:n_files
    if isempty(profiles{i}), continue; end
    p = profiles{i};
    plot(p.bin_center*100, p.vm_max/1e6, '-o', 'Color', files(i).color, ...
        'LineWidth', 1.6, 'MarkerSize', 4, 'DisplayName', files(i).label);
    xline(raw{i}.transition_pct, '--', 'Color', files(i).color, 'HandleVisibility', 'off');
end
xlabel('Normalized Gripper Length (%) | 0=Fixed base, 100=Fingertip');
ylabel('Max Von Mises Stress (MPa)');
title('Stress Profile');
legend('Location','northeast'); box on;

exportgraphics(fig1, 'SimScale_Profiles.png', 'Resolution', 300);

%  FIGURE 2: PEAK STRESS LOCATION (TOP VIEW)
fig2 = figure('Name', 'Peak Stress Location', 'Color', 'w', 'Position', [150 150 700 550]);
hold on; grid on; axis equal;

for i = 1:n_files
    if isempty(raw{i}), continue; end
    r = raw{i};
    idx_sample = 1:20:numel(r.X);
    scatter(r.X(idx_sample)*1000, r.Y(idx_sample)*1000, 4, [0.85 0.85 0.85], 'HandleVisibility','off');
    [~, idx_peak] = max(r.vonMises);
    scatter(r.X(idx_peak)*1000, r.Y(idx_peak)*1000, 120, files(i).color, 'p', 'filled', ...
        'MarkerEdgeColor','k', 'DisplayName', [files(i).label ' - Peak']);
end

xlabel('X (mm)'); ylabel('Y (mm)');
title('Von Mises Peak Stress Location');
legend('Location','best');

% exportgraphics(fig2, 'SimScale_Peak_Location.png', 'Resolution', 300);