
% ==========================================
% CREAR CSV DE LA SIMULACION
% ==========================================

carpeta = 'C:\Users\User\Documents\pythonPrj\resultados_matlab-coppelia';

% Fecha y hora de creacion
fechaHora = datetime('now', ...
                     'Format', ...
                     'yyyyMMdd_HHmmss');

% Nombre del archivo
nombreArchivo = ['Simulacion_Enjambre_' ...
                 char(fechaHora) '.csv'];

rutaArchivo = fullfile(carpeta, nombreArchivo);

% Numero de drones del enjambre
numDrones = 8;

% Crear encabezados
encabezados = {'Iteracion', 'Enemy_target x', 'Enemy_target y','Enemy_real x', 'Enemy Real y'};

for k = 1:numDrones
    encabezados{end+1} = sprintf('Def_target x %d', k);
    encabezados{end+1} = sprintf('Def_target y %d', k);
    encabezados{end+1} = sprintf('Def_real x %d', k);
    encabezados{end+1} = sprintf('Def_real y %d', k);
end

for k = 1:numDrones
    
    
end

% Abrir archivo CSV
fid = fopen(rutaArchivo, 'w');

if fid == -1
    error('No se pudo crear el archivo CSV.');
end

% Escribir encabezados
fprintf(fid, '%s', encabezados{1});

for k = 2:length(encabezados)
    fprintf(fid, ',%s', encabezados{k});
end
fprintf(fid, '\n');

% Inicializar coordenadas de los 8 drones
swarmXY= [+5  -0;   %Quadcopter[0]
          +5  -5;   %Quadcopter[1]
          -0  -5;   %Quadcopter[2]
          -5  -5;   %Quadcopter[3]
          -5  +0;   %Quadcopter[4]
          -5  +5;   %Quadcopter[5]
          +0  +5;   %Quadcopter[6]
          +5  +5];  %Quadcopter[7]

swarmXY_R= [+5  -0;   %Quadcopter[0]
            +5  -5;   %Quadcopter[1]
            -0  -5;   %Quadcopter[2]
            -5  -5;   %Quadcopter[3]
            -5  +0;   %Quadcopter[4]
            -5  +5;   %Quadcopter[5]
            +0  +5;   %Quadcopter[6]
            +5  +5];  %Quadcopter[7]

for i=1:t
    iteracion = i;

     enemyX = trajectory01(i,1);
     enemyY = trajectory01(i,2);

     enemyX_R = realpos01(i,1);
     enemyY_R = realpos01(i,2);

     swarmXY(1,1)= trajectory02(i,1);
     swarmXY(1,2)= trajectory02(i,2);

     swarmXY(2,1)= trajectory03(i,1);
     swarmXY(2,2)= trajectory03(i,2);

     swarmXY(3,1)= trajectory04(i,1);
     swarmXY(3,2)= trajectory04(i,2);

     swarmXY(4,1)= trajectory05(i,1);
     swarmXY(4,2)= trajectory05(i,2);

     swarmXY(5,1)= trajectory06(i,1);
     swarmXY(5,2)= trajectory06(i,2);

     swarmXY(6,1)= trajectory07(i,1);
     swarmXY(6,2)= trajectory07(i,2);

     swarmXY(7,1)= trajectory08(i,1);
     swarmXY(7,2)= trajectory08(i,2);

     swarmXY(8,1)= trajectory09(i,1);
     swarmXY(8,2)= trajectory09(i,2);

     swarmXY_R(1,1)= realpos02(i,1);
     swarmXY_R(1,2)= realpos02(i,2);

     swarmXY_R(2,1)= realpos03(i,1);
     swarmXY_R(2,2)= realpos03(i,2);

     swarmXY_R(3,1)= realpos04(i,1);
     swarmXY_R(3,2)= realpos04(i,2);

     swarmXY_R(4,1)= realpos05(i,1);
     swarmXY_R(4,2)= realpos05(i,2);

     swarmXY_R(5,1)= realpos06(i,1);
     swarmXY_R(5,2)= realpos06(i,2);

     swarmXY_R(6,1)= realpos07(i,1);
     swarmXY_R(6,2)= realpos07(i,2);

     swarmXY_R(7,1)= realpos08(i,1);
     swarmXY_R(7,2)= realpos08(i,2);

     swarmXY_R(8,1)= realpos09(i,1);
     swarmXY_R(8,2)= realpos09(i,2);


    fila = [iteracion, ...
            enemyX, enemyY, enemyX_R, enemyY_R...
            swarmXY(1,1), swarmXY(1,2), swarmXY_R(1,1), swarmXY_R(1,2)...
            swarmXY(2,1), swarmXY(2,2), swarmXY_R(2,1), swarmXY_R(2,2)...
            swarmXY(3,1), swarmXY(3,2), swarmXY_R(3,1), swarmXY_R(3,2)...
            swarmXY(4,1), swarmXY(4,2), swarmXY_R(4,1), swarmXY_R(4,2)...
            swarmXY(5,1), swarmXY(5,2), swarmXY_R(5,1), swarmXY_R(5,2)...
            swarmXY(6,1), swarmXY(6,2), swarmXY_R(6,1), swarmXY_R(6,2)...
            swarmXY(7,1), swarmXY(7,2), swarmXY_R(7,1), swarmXY_R(7,2)...
            swarmXY(8,1), swarmXY(8,2), swarmXY_R(8,1), swarmXY_R(8,2)];

    fprintf(fid, '%d', fila(1));
    fprintf(fid, ',%.4f', fila(2:end));
    fprintf(fid, '\n');


end

% Cerrar archivo
fclose(fid);

fprintf('Archivo creado: %s\n', rutaArchivo);
