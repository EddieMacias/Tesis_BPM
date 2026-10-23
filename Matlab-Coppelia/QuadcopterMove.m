%% 
clc
num_defeders = 8;

range_x                     = [-10, 10];
range_y                     = [-10, 10];
boidAlt                     = 4;
num_iterations              = 200;
cohesion_factor             = 0.02;
separation_factor           = 0.05;
alignment_factor            = 0.01;
Enemy_avoidance_factor      = 0.01;
Enemy_separation_distance   = 0.01;  

pos_ini_enemy               = [-9.9, 9.9];
enemy_obj                   = [0,  0];
enemy_Vel                   = 0.08;
Enemy                       = pos_ini_enemy;

defenders = [+5  -0;   %Quadcopter[0]
             +5  -5;   %Quadcopter[1]
             -0  -5;   %Quadcopter[2]
             -5  -5;   %Quadcopter[3]
             -5  +0;   %Quadcopter[4]
             -5  +5;   %Quadcopter[5]
             +0  +5;   %Quadcopter[6]
             +5  +5];  %Quadcopter[7]

trajectory01 = zeros(num_iterations,3);
trajectory02 = zeros(num_iterations,3);
trajectory03 = zeros(num_iterations,3);
trajectory04 = zeros(num_iterations,3);
trajectory05 = zeros(num_iterations,3);
trajectory06 = zeros(num_iterations,3);
trajectory07 = zeros(num_iterations,3);
trajectory08 = zeros(num_iterations,3);
trajectory09 = zeros(num_iterations,3);

realpos01    = zeros(num_iterations,3);
realpos02    = zeros(num_iterations,3);
realpos03    = zeros(num_iterations,3);
realpos04    = zeros(num_iterations,3);
realpos05    = zeros(num_iterations,3);
realpos06    = zeros(num_iterations,3);
realpos07    = zeros(num_iterations,3);
realpos08    = zeros(num_iterations,3);
realpos09    = zeros(num_iterations,3);

velocity_D = [0, 0];
velocities = rand(num_defeders, 2) - 0.5;

% Conectar a CoppeliaSim
sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);

% Verificar la conexión
if (clientID > -1)
    disp('Conectado a CoppeliaSim');
    
    % Obtener el handle del target del cuadricóptero
    [res1, target_handle1] = sim.simxGetObjectHandle (clientID, ':/target[0]',sim.simx_opmode_blocking);
    [res2, target_handle2] = sim.simxGetObjectHandle (clientID, ':/target[1]',sim.simx_opmode_blocking);
    [res3, target_handle3] = sim.simxGetObjectHandle (clientID, ':/target[2]',sim.simx_opmode_blocking);
    [res4, target_handle4] = sim.simxGetObjectHandle (clientID, ':/target[3]',sim.simx_opmode_blocking);
    [res5, target_handle5] = sim.simxGetObjectHandle (clientID, ':/target[4]',sim.simx_opmode_blocking);
    [res6, target_handle6] = sim.simxGetObjectHandle (clientID, ':/target[5]',sim.simx_opmode_blocking);
    [res7, target_handle7] = sim.simxGetObjectHandle (clientID, ':/target[6]',sim.simx_opmode_blocking);
    [res8, target_handle8] = sim.simxGetObjectHandle (clientID, ':/target[7]',sim.simx_opmode_blocking);
    [res9, target_handle9] = sim.simxGetObjectHandle (clientID, ':/target[8]',sim.simx_opmode_blocking);


    [resB1, base_handle1] = sim.simxGetObjectHandle (clientID, ':/Quadcopter[0]',sim.simx_opmode_blocking);
    [resB2, base_handle2] = sim.simxGetObjectHandle (clientID, ':/Quadcopter[1]',sim.simx_opmode_blocking);
    [resB3, base_handle3] = sim.simxGetObjectHandle (clientID, ':/Quadcopter[2]',sim.simx_opmode_blocking);
    [resB4, base_handle4] = sim.simxGetObjectHandle (clientID, ':/Quadcopter[3]',sim.simx_opmode_blocking);
    [resB5, base_handle5] = sim.simxGetObjectHandle (clientID, ':/Quadcopter[4]',sim.simx_opmode_blocking);
    [resB6, base_handle6] = sim.simxGetObjectHandle (clientID, ':/Quadcopter[5]',sim.simx_opmode_blocking);
    [resB7, base_handle7] = sim.simxGetObjectHandle (clientID, ':/Quadcopter[6]',sim.simx_opmode_blocking);
    [resB8, base_handle8] = sim.simxGetObjectHandle (clientID, ':/Quadcopter[7]',sim.simx_opmode_blocking);
    [resB9, base_handle9] = sim.simxGetObjectHandle (clientID, ':/Quadcopter[8]',sim.simx_opmode_blocking);

    target_pos1 = [Enemy(1,1), Enemy(1,2), boidAlt];
    target_pos2 = [defenders(1,1), defenders(1,2), boidAlt];
    target_pos3 = [defenders(2,1), defenders(2,2), boidAlt];
    target_pos4 = [defenders(3,1), defenders(3,2), boidAlt];
    target_pos5 = [defenders(4,1), defenders(4,2), boidAlt];
    target_pos6 = [defenders(5,1), defenders(5,2), boidAlt];
    target_pos7 = [defenders(6,1), defenders(6,2), boidAlt];
    target_pos8 = [defenders(7,1), defenders(7,2), boidAlt];
    target_pos9 = [defenders(8,1), defenders(8,2), boidAlt];


    if (resB1 == sim.simx_return_ok && ...
        resB2 == sim.simx_return_ok && ...
        resB3 == sim.simx_return_ok && ...
        resB4 == sim.simx_return_ok && ...
        resB5 == sim.simx_return_ok && ...
        resB6 == sim.simx_return_ok && ...
        resB7 == sim.simx_return_ok && ...
        resB8 == sim.simx_return_ok && ...
        resB9 == sim.simx_return_ok)
        for i=1:20


                [res1]  = sim.simxSetObjectPosition(clientID, target_handle1,  -1, target_pos1, sim.simx_opmode_continuous);
                [res2]  = sim.simxSetObjectPosition(clientID, target_handle2,  -1, target_pos2, sim.simx_opmode_continuous);
                [res3]  = sim.simxSetObjectPosition(clientID, target_handle3,  -1, target_pos3, sim.simx_opmode_continuous);
                [res4]  = sim.simxSetObjectPosition(clientID, target_handle4,  -1, target_pos4, sim.simx_opmode_continuous);
                [res5]  = sim.simxSetObjectPosition(clientID, target_handle5,  -1, target_pos5, sim.simx_opmode_continuous);
                [res6]  = sim.simxSetObjectPosition(clientID, target_handle6,  -1, target_pos6, sim.simx_opmode_continuous);
                [res7]  = sim.simxSetObjectPosition(clientID, target_handle7,  -1, target_pos7, sim.simx_opmode_continuous);
                [res8]  = sim.simxSetObjectPosition(clientID, target_handle8,  -1, target_pos8, sim.simx_opmode_continuous);
                [res9]  = sim.simxSetObjectPosition(clientID, target_handle9,  -1, target_pos9, sim.simx_opmode_continuous);
        
                [resB1, RPQuadcopter0] = sim.simxGetObjectPosition(clientID, base_handle1, -1, sim.simx_opmode_continuous);
                [resB2, RPQuadcopter1] = sim.simxGetObjectPosition(clientID, base_handle2, -1, sim.simx_opmode_continuous);
                [resB3, RPQuadcopter2] = sim.simxGetObjectPosition(clientID, base_handle3, -1, sim.simx_opmode_continuous);
                [resB4, RPQuadcopter3] = sim.simxGetObjectPosition(clientID, base_handle4, -1, sim.simx_opmode_continuous);
                [resB5, RPQuadcopter4] = sim.simxGetObjectPosition(clientID, base_handle5, -1, sim.simx_opmode_continuous);
                [resB6, RPQuadcopter5] = sim.simxGetObjectPosition(clientID, base_handle6, -1, sim.simx_opmode_continuous);
                [resB7, RPQuadcopter6] = sim.simxGetObjectPosition(clientID, base_handle7, -1, sim.simx_opmode_continuous);
                [resB8, RPQuadcopter7] = sim.simxGetObjectPosition(clientID, base_handle8, -1, sim.simx_opmode_continuous);
                [resB9, RPQuadcopter8] = sim.simxGetObjectPosition(clientID, base_handle9, -1, sim.simx_opmode_continuous);
        end
    end 

    
    if (res1 == sim.simx_return_ok && ...
        res2 == sim.simx_return_ok && ...
        res3 == sim.simx_return_ok && ...
        res4 == sim.simx_return_ok && ...
        res5 == sim.simx_return_ok && ...
        res6 == sim.simx_return_ok && ...
        res6 == sim.simx_return_ok && ...
        res7 == sim.simx_return_ok && ...
        res8 == sim.simx_return_ok && ...
        res9 == sim.simx_return_ok)

        disp('Handle del target obtenido correctamente');

       for t = 1:num_iterations
           tic;
           d    = enemy_obj - Enemy;
           dist = norm(d);
  
           if dist > 1e-9
               Enemy = Enemy + d / dist * min(enemy_Vel, dist);
           end

            for i = 1:num_defeders
                % Regla de cohesion enjambre defensor
                cohesion = (Enemy  - defenders(i,:))*cohesion_factor;

                %Regla Separacion Enjambre Defensor
                separation = zeros(1,2);
                for j = 1:num_defeders
                    if i ~= j
                        dist = norm(defenders(i,:) - defenders(j,:));
                        if dist < 0.1                                       % Evitar división por cero
                            separation = separation + rand(1,2);            % Aplicar un pequeño alejamiento aleatorio
                        elseif dist < 0.5
                            separation = separation - (defenders(j,:) - defenders(i,:)) / dist^2;
                        end
                    end
                end
                separation = separation * separation_factor;


                % Regla de separación de lider
                enemy_avoidance = zeros(1,2);
                for k = 1:size(Enemy, 1)
                    dist_to_Enemy = norm(defenders(i,:) - Enemy);
                    if dist_to_Enemy < Enemy_separation_distance
                        enemy_avoidance = enemy_avoidance + (defenders(i,:) - Enemy) / dist_to_Enemy^2;
                    end
                end
                enemy_avoidance = enemy_avoidance * Enemy_avoidance_factor;

                % REGLA DE ALINEACIÓN
                alignment = mean(velocities) * alignment_factor;   


                 % Actualizar velocidad
                velocities(i,:) = velocities(i,:) + cohesion + separation + alignment + enemy_avoidance;
                % 
                max_speed = 0.1;
                speed = norm(velocities(i,:));
                if speed > max_speed
                    velocities(i,:) = velocities(i,:) * max_speed / speed;
                end



                 % Actualizar posición defensores 
                defenders(i,:) = defenders(i,:) + velocities(i,:);


                target_pos1 = [Enemy(1,1), Enemy(1,2), boidAlt];
                target_pos2 = [defenders(1,1), defenders(1,2), boidAlt];
                target_pos3 = [defenders(2,1), defenders(2,2), boidAlt];
                target_pos4 = [defenders(3,1), defenders(3,2), boidAlt];
                target_pos5 = [defenders(4,1), defenders(4,2), boidAlt];
                target_pos6 = [defenders(5,1), defenders(5,2), boidAlt];
                target_pos7 = [defenders(6,1), defenders(6,2), boidAlt];
                target_pos8 = [defenders(7,1), defenders(7,2), boidAlt];
                target_pos9 = [defenders(8,1), defenders(8,2), boidAlt];


                [res1]  = sim.simxSetObjectPosition(clientID, target_handle1,  -1, target_pos1, sim.simx_opmode_continuous);
                [res2]  = sim.simxSetObjectPosition(clientID, target_handle2,  -1, target_pos2, sim.simx_opmode_continuous);
                [res3]  = sim.simxSetObjectPosition(clientID, target_handle3,  -1, target_pos3, sim.simx_opmode_continuous);
                [res4]  = sim.simxSetObjectPosition(clientID, target_handle4,  -1, target_pos4, sim.simx_opmode_continuous);
                [res5]  = sim.simxSetObjectPosition(clientID, target_handle5,  -1, target_pos5, sim.simx_opmode_continuous);
                [res6]  = sim.simxSetObjectPosition(clientID, target_handle6,  -1, target_pos6, sim.simx_opmode_continuous);
                [res7]  = sim.simxSetObjectPosition(clientID, target_handle7,  -1, target_pos7, sim.simx_opmode_continuous);
                [res8]  = sim.simxSetObjectPosition(clientID, target_handle8,  -1, target_pos8, sim.simx_opmode_continuous);
                [res9]  = sim.simxSetObjectPosition(clientID, target_handle9,  -1, target_pos9, sim.simx_opmode_continuous);

                [resB1, RPQuadcopter0] = sim.simxGetObjectPosition(clientID, base_handle1, -1, sim.simx_opmode_continuous);
                [resB2, RPQuadcopter1] = sim.simxGetObjectPosition(clientID, base_handle2, -1, sim.simx_opmode_continuous);
                [resB3, RPQuadcopter2] = sim.simxGetObjectPosition(clientID, base_handle3, -1, sim.simx_opmode_continuous);
                [resB4, RPQuadcopter3] = sim.simxGetObjectPosition(clientID, base_handle4, -1, sim.simx_opmode_continuous);
                [resB5, RPQuadcopter4] = sim.simxGetObjectPosition(clientID, base_handle5, -1, sim.simx_opmode_continuous);
                [resB6, RPQuadcopter5] = sim.simxGetObjectPosition(clientID, base_handle6, -1, sim.simx_opmode_continuous);
                [resB7, RPQuadcopter6] = sim.simxGetObjectPosition(clientID, base_handle7, -1, sim.simx_opmode_continuous);
                [resB8, RPQuadcopter7] = sim.simxGetObjectPosition(clientID, base_handle8, -1, sim.simx_opmode_continuous);
                [resB9, RPQuadcopter8] = sim.simxGetObjectPosition(clientID, base_handle9, -1, sim.simx_opmode_continuous);


                trajectory01(t,:) = [Enemy(1,:),    boidAlt];
                trajectory02(t,:) = [defenders(1,:),boidAlt];
                trajectory03(t,:) = [defenders(2,:),boidAlt];
                trajectory04(t,:) = [defenders(3,:),boidAlt];
                trajectory05(t,:) = [defenders(4,:),boidAlt];
                trajectory06(t,:) = [defenders(5,:),boidAlt];
                trajectory07(t,:) = [defenders(6,:),boidAlt];
                trajectory08(t,:) = [defenders(7,:),boidAlt];
                trajectory09(t,:) = [defenders(8,:),boidAlt];


                realpos01(t,:) = RPQuadcopter0;
                realpos02(t,:) = RPQuadcopter1;
                realpos03(t,:) = RPQuadcopter2;
                realpos04(t,:) = RPQuadcopter3;
                realpos05(t,:) = RPQuadcopter4;
                realpos06(t,:) = RPQuadcopter5;
                realpos07(t,:) = RPQuadcopter6;
                realpos08(t,:) = RPQuadcopter7;
                realpos09(t,:) = RPQuadcopter8;

            end


            if (res1 == sim.simx_return_ok && ...
                res2 == sim.simx_return_ok && ...
                res3 == sim.simx_return_ok && ...
                res4 == sim.simx_return_ok && ...
                res5 == sim.simx_return_ok && ...
                res6 == sim.simx_return_ok && ...
                res6 == sim.simx_return_ok && ...
                res7 == sim.simx_return_ok && ...
                res8 == sim.simx_return_ok && ...
                res9 == sim.simx_return_ok)

                disp(['Posición del target establecida correctamente -- ', num2str(t)]);
                pause(1)
                tiempoIteracion = toc;
            else
                disp('Error al establecer la posición del target');
            end
       end
    else
        disp('Error al obtener el handle del target');
    end
    
    % Detener la simulación y cerrar la conexión con coppelia
    sim.simxStopSimulation(clientID, sim.simx_opmode_blocking);
    sim.simxFinish(clientID);
    sim.delete();
else
    disp('Error al conectar con CoppeliaSim');
end
    disp('Simulación detenida y conexión cerrada');
close all;

if t>199
    Datalogger
end