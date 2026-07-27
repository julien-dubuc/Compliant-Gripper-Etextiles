% 1. Charger les données
force = readmatrix('f.txt');

% 2. Extraction ultra-rapide des pics par fenêtre glissante
fenetre = 50; 
pics_max = zeros(size(force));

for i = 1 : length(force)
    i_debut = max(1, i - fenetre);
    i_fin = min(length(force), i + fenetre);
    
    % Si la valeur actuelle est le maximum local dans sa fenêtre
    if force(i) == max(force(i_debut:i_fin)) && force(i) > 5
        pics_max(i) = force(i);
    end
end

% Nettoyer les zéros pour ne garder que les vrais pics
idx_valides = pics_max > 0;
force_pics = pics_max(idx_valides);

% Éliminer les éventuels doublons si le sommet est un plateau plat
% (On ne garde le point que s'il est séparé du précédent d'au moins 20 index)
locs = find(idx_valides);
vrais_pics = true(size(locs));
for i = 2:length(locs)
    if locs(i) - locs(i-1) < 20
        vrais_pics(i) = false;
    end
end
force_finale = force_pics(vrais_pics);

% 3. Créer l'axe des cycles (1, 2, 3... jusqu'au nombre total de pics)
cycles = 1 : length(force_finale);

% Afficher le compte exact dans la console
fprintf('\n---> Nombre total de cycles (forces maximales) détectés : %d\n\n', length(force_finale));

% 4. Tracer la courbe en fonction du numéro de cycle
figure('Name', 'Fatigue Test', 'Color', 'w');
plot(cycles, force_finale, '-o', 'Color', [0 0.447 0.741], 'LineWidth', 1.5, 'MarkerSize', 4, 'MarkerFaceColor', [0 0.447 0.741]);

grid on;
title('Fatigue Test', 'FontSize', 14);
xlabel('Number of Cycles', 'FontSize', 12);
ylabel('Max Force (N)', 'FontSize', 12);

% Forcer l'axe X à n'afficher que des nombres entiers (on ne fait pas de "demi-cycle")
xticks(round(linspace(1, length(force_finale), min(10, length(force_finale)))));