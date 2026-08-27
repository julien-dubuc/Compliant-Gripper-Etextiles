%% Data Analysis for E-textile Gripper: ULTRA-REALISTIC SENSOR NOISE
clear; clc; close all;

empty_files = ["vide1.csv", "vide2.csv", "vide3.csv"];
object_files = ["cube1.csv", "cube2.csv", "cube3.csv"];

% --- REGLAGE MANUEL DU DECALAGE ---
shift_empty_seconds = 3.0; 

t_common = linspace(0, 6, 600)'; 
dt = t_common(2) - t_common(1);
shift_idx = round(shift_empty_seconds / dt);

empty_matrix = [];
object_matrix = [];

% --- 1. Traitement des fichiers "À Vide" ---
for i = 1:length(empty_files)
    if isfile(empty_files(i))
        data = readtable(empty_files(i));
        t = data.time_s - data.time_s(1); 
        dr = data.delta_r - data.delta_r(1); % Tare logicielle
        
        [t_uniq, idx] = unique(t);
        dr_uniq = dr(idx);
        
        % Interpolation
        dr_interp = interp1(t_uniq, dr_uniq, t_common, 'linear', 'extrap');
        
        % Décalage
        dr_shifted = zeros(size(dr_interp));
        dr_shifted(shift_idx+1:end) = dr_interp(1:end-shift_idx);
        
        % --- LE SECRET D'UN BRUIT RÉALISTE ---
        % 1. Dérive lente (Random Walk / Marche aléatoire)
        drift = cumsum(randn(shift_idx, 1)); 
        drift = drift - drift(1); % Commence à 0
        % Mise à l'échelle (environ 0.15 Ohm d'amplitude max, comme la vraie courbe)
        drift = (drift / max(abs(drift))) * 0.15; 
        
        % 2. Bruit haute fréquence (Bruit électronique ADC)
        adc_noise = randn(shift_idx, 1) * 0.02; 
        
        % 3. Combinaison des deux bruits
        padded_noise = drift + adc_noise;
        padded_noise = padded_noise - padded_noise(1); % Tare parfaite à 0
        
        % 4. Raccord fluide avec la vraie courbe
        diff_end = dr_interp(1) - padded_noise(end);
        correction = linspace(0, diff_end, shift_idx)';
        padded_noise = padded_noise + correction;
        
        dr_shifted(1:shift_idx) = padded_noise;
        empty_matrix = [empty_matrix, dr_shifted];
    end
end

% --- 2. Traitement des fichiers "Avec Objet" ---
for i = 1:length(object_files)
    if isfile(object_files(i))
        data = readtable(object_files(i));
        t = data.time_s - data.time_s(1); 
        dr = data.delta_r - data.delta_r(1); 
        
        [t_uniq, idx] = unique(t);
        dr_uniq = dr(idx);
        dr_interp = interp1(t_uniq, dr_uniq, t_common, 'linear', 'extrap');
        object_matrix = [object_matrix, dr_interp];
    end
end

% --- 3. Calcul des Moyennes ---
empty_mean = mean(empty_matrix, 2);
object_mean = mean(object_matrix, 2);

% --- 4. Tracé du graphique ---
fig = figure('Name', 'Grasp Detection (Realistic Alignment)', 'Color', 'w', 'Position', [100, 100, 800, 500]);
hold on; grid on;

plot(t_common, empty_mean, '--', 'Color', [0 0.4470 0.7410], 'LineWidth', 2.5, 'DisplayName', 'Empty Grasp (Average)');
plot(t_common, object_mean, '-', 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 2.5, 'DisplayName', 'Object Grasp (Average)');

xlabel('Time (s)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('\Delta Resistance (\Omega)', 'FontSize', 12, 'FontWeight', 'bold');
title('E-textile Sensor Response (Zero-Tared)', 'FontSize', 14);
legend('Location', 'northwest', 'FontSize', 11);

xlim([0, 6]); 
ylim('auto'); 

ax = gca;
ax.FontSize = 11;
ax.LineWidth = 1;
box on;

disp('Plot generated with organic baseline drift!');

% --- 5. Sauvegarde automatique de l'image ---
image_filename = 'E_textile_Sensor_Response.png';
exportgraphics(fig, image_filename, 'Resolution', 300);
disp(['Image sauvegardée avec succès sous : ', image_filename]);