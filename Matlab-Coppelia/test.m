%% Animación del dron hostil en línea recta (solo MATLAB)
clear; clc; close all;

% Parámetros
rangeX        = [-10 10];
rangeY        = [-10 10];
posIniLeader  = [-9, 9];     % posición inicial
objetivo      = [0, 0];      % activo protegido
pasoLeader    = 0.08;        % avance por iteración
radioObjetivo = 0.5;         % radio de defensa fallida
numIter       = 1000;

leader = posIniLeader;
trj    = zeros(numIter, 2);

figure('Color', 'w');
for t = 1:numIter
    trj(t, :) = leader;

    % Avance en línea recta hacia el objetivo
    d    = objetivo - leader;
    dist = norm(d);
    if dist > 1e-9
        leader = leader + d / dist * min(pasoLeader, dist);
    end

end
trj = trj(1:t, :);   % trayectoria registrada