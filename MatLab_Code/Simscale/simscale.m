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
    'color', {[0 0.45 0.74], [0.85 0.33 0.10], [0.47 0.67 0.19]}, ...
    'finger_length_mm', {100.0, 89.8, 79.6} ); % REAL flexible-finger length (base to tip), measured on the physical part

num_bins = 20; % Number of bins along the finger (0=base, 1=tip)
tip_frac = 0.97;  % Threshold defining the "tip zone" (last 3%)

%  LOADING & NORMALIZED PROFILES EXTRACTION
% IMPORTANT: the SimScale fixed constraint (~Y=-137.92mm) is NOT the root
% of the flexible finger - the CAD also includes a rigid mounting section
% between the constraint and the actual flexible material. Using it as
% the normalization base was WRONG (it diluted the finger-length % with
% ~71mm of rigid, non-deforming structure).
% The true flexible-finger base is computed per file as
% (tip Y - real finger length), using the physical dimensions above.
% Verified: this gives Y=-66.82mm IDENTICALLY on all 3 files (confirms
% the rigid mount is exactly the same ~71mm on every variant, only the
% flexible tip-to-middle section was shortened by design).
n_files = numel(files);
profiles = cell(1, n_files);
raw = cell(1, n_files);

fprintf('=== Loading and extracting profiles ===\n');
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
    y_base = y_tip - files(i).finger_length_mm/1000; % TRUE flexible-finger root
    y_norm = (Y - y_base) / (y_tip - y_base);
    
    raw{i} = struct('X',X, 'Y',Y, 'Z',Z, 'y_norm',y_norm, ...
        'disp_mag',disp_mag, 'vonMises',vonMises, 'y_tip',y_tip, 'y_base',y_base);
        
    % --- Binned profile along the finger length ---
    % Nodes belonging to the rigid mount (y_norm<0) fall outside [0,1]
    % and are excluded from the profile - only the flexible section is binned.
    edges = linspace(0, 1, num_bins+1);
    bin_idx = discretize(y_norm, edges);
    bin_center = (edges(1:end-1) + edges(2:end)) / 2;
    
    disp_mean = accumarray(bin_idx(~isnan(bin_idx)), disp_mag(~isnan(bin_idx)), [num_bins,1], @mean, NaN);
    disp_max  = accumarray(bin_idx(~isnan(bin_idx)), disp_mag(~isnan(bin_idx)), [num_bins,1], @max, NaN);
    vm_mean   = accumarray(bin_idx(~isnan(bin_idx)), vonMises(~isnan(bin_idx)), [num_bins,1], @mean, NaN);
    vm_max    = accumarray(bin_idx(~isnan(bin_idx)), vonMises(~isnan(bin_idx)), [num_bins,1], @max, NaN);
    
    profiles{i} = struct('bin_center', bin_center(:), ...
        'disp_mean', disp_mean, 'disp_max', disp_max, ...
        'vm_mean', vm_mean, 'vm_max', vm_max);
        
    n_rigid = sum(y_norm < 0);
    fprintf(['  %-14s : %d nodes total (%d in rigid mount, excluded), ' ...
        'flexible length = %.1f mm (base Y=%.2f, tip Y=%.2f mm)\n'], ...
        files(i).label, numel(Y), n_rigid, files(i).finger_length_mm, y_base*1000, y_tip*1000);
end

%  DIAGNOSTICS: PEAK STRESS & TIP DISPLACEMENT
fprintf('\n=== von Mises Peak Stress (Location & Magnitude), FLEXIBLE SECTION ONLY ===\n');
for i = 1:n_files
    if isempty(raw{i}), continue; end
    r = raw{i};
    mask = r.y_norm >= 0; % exclude rigid mount from the peak search
    idx_list = find(mask);
    [vm_peak, k] = max(r.vonMises(mask));
    idx = idx_list(k);
    fprintf('  %-14s : %.1f MPa at y_norm=%.3f (X=%.1f, Y=%.1f, Z=%.1f mm)\n', ...
        files(i).label, vm_peak/1e6, r.y_norm(idx), r.X(idx)*1000, r.Y(idx)*1000, r.Z(idx)*1000);
end

fprintf('\n=== Max Displacement in Tip Zone (y_norm > %.2f) ===\n', tip_frac);
for i = 1:n_files
    if isempty(raw{i}), continue; end
    r = raw{i};
    mask = r.y_norm > tip_frac;
    fprintf('  %-14s : %.2f mm (n=%d nodes)\n', files(i).label, max(r.disp_mag(mask))*1000, sum(mask));
end

% FIGURE 1: COMPARATIVE PROFILES
fig1 = figure('Name', 'SimScale Profile Comparison', 'Color', 'w', 'Position', [100 100 1100 450]);

subplot(1,2,1); hold on; grid on;
for i = 1:n_files
    if isempty(profiles{i}), continue; end
    p = profiles{i};
    plot(p.bin_center*100, p.disp_max*1000, '-o', 'Color', files(i).color, ...
        'LineWidth', 1.6, 'MarkerSize', 4, 'DisplayName', files(i).label);
end
xlabel('Normalized Finger Length (%) | 0=Base, 100=Tip');
ylabel('Max Displacement (mm)');
title('Displacement Profile');
legend('Location','northwest'); box on;

subplot(1,2,2); hold on; grid on;
for i = 1:n_files
    if isempty(profiles{i}), continue; end
    p = profiles{i};
    plot(p.bin_center*100, p.vm_max/1e6, '-o', 'Color', files(i).color, ...
        'LineWidth', 1.6, 'MarkerSize', 4, 'DisplayName', files(i).label);
end
xlabel('Normalized Finger Length (%) | 0=Base, 100=Tip');
ylabel('Max Von Mises Stress (MPa)');
title('Stress Profile');
legend('Location','northeast'); box on;

sgtitle('SimScale Analysis: C1 vs C2 vs C3 (flexible finger only, rigid mount excluded)');

% exportgraphics(fig1, 'SimScale_Profiles.png', 'Resolution', 300);

%  FIGURE 2: PEAK STRESS LOCATION (TOP VIEW)
% Verifies if the structural hotspot remains on the same hinge across designs.
fig2 = figure('Name', 'Peak Stress Location', 'Color', 'w', 'Position', [150 150 700 550]);
hold on; grid on; axis equal;

for i = 1:n_files
    if isempty(raw{i}), continue; end
    r = raw{i};
    
    % Subsampling for rendering performance
    idx_sample = 1:20:numel(r.X);
    scatter(r.X(idx_sample)*1000, r.Y(idx_sample)*1000, 4, [0.85 0.85 0.85], 'HandleVisibility','off');
    
    mask = r.y_norm >= 0;
    idx_list = find(mask);
    [~, k] = max(r.vonMises(mask));
    idx_peak = idx_list(k);
    scatter(r.X(idx_peak)*1000, r.Y(idx_peak)*1000, 120, files(i).color, 'p', 'filled', ...
        'MarkerEdgeColor','k', 'DisplayName', [files(i).label ' - Peak']);
end

xlabel('X (mm)'); ylabel('Y (mm)');
title('von Mises Peak Stress Location (flexible section only)');
legend('Location','best');

% exportgraphics(fig2, 'SimScale_Peak_Location.png', 'Resolution', 300);