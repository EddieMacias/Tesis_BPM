clc

num_defeders = 8;

range_x                     = [-10, 10];
range_y                     = [-10, 10];
boidAlt                     = 4;
velocities                  = rand(num_defeders, 2) - 0.5;
num_iterations              = 300;
cohesion_factor             = 0.02;
separation_factor           = 0.05;
alignment_factor            = 0.01;
obstacle_avoidance_factor   = 0.3;
shepherds_avoidance_factor  = 0.4;
shepherd_approach_distance  = 0.5;
separation_radius           = 0.1;
obstacle_radius             = 0.3;
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
trajectory08 = zeros(num_iterations,3);

velocity_D = [0, 0];
velocities = rand(num_defeders, 2) - 0.5;

% Conectar a CoppeliaSim
sim = remApi('remoteApi');
sim.simxFinish(-1);
clientID = sim.simxStart('127.0.0.1',19999,true,true,5000,5);

% Verificar la conexión
if (clientID > -1)
    disp('Conectado a CoppeliaSim');
    
    % Obtener el handle del target del cuadricóptero
    [res1, target_handle1]   = sim.simxGetObjectHandle(clientID,'Quadcopter_target',sim.simx_opmode_blocking);
    [res2, target_handle2]   = sim.simxGetObjectHandle(clientID,'Quadcopter_target#0',sim.simx_opmode_blocking);
    [res3, target_handle3]   = sim.simxGetObjectHandle(clientID,'Quadcopter_target#1',sim.simx_opmode_blocking);
    [res4, target_handle4]   = sim.simxGetObjectHandle(clientID,'Quadcopter_target#2',sim.simx_opmode_blocking);
    [res5, target_handle5]   = sim.simxGetObjectHandle(clientID,'Quadcopter_target#3',sim.simx_opmode_blocking);
    [res6, target_handle6]   = sim.simxGetObjectHandle(clientID,'Quadcopter_target#4',sim.simx_opmode_blocking);
    [res7, target_handle7]   = sim.simxGetObjectHandle(clientID,'Quadcopter_target#5',sim.simx_opmode_blocking);
    [res8, target_handle8]   = sim.simxGetObjectHandle(clientID,'Quadcopter_target#6',sim.simx_opmode_blocking);
    [res9, target_handle9]   = sim.simxGetObjectHandle(clientID,'Quadcopter_target#7',sim.simx_opmode_blocking);

    if (res1 == sim.simx_return_ok && ...
        res2 == sim.simx_return_ok && ...
        res3 == sim.simx_return_ok && ...
        res4 == sim.simx_return_ok && ...
        res5 == sim.simx_return_ok && ...
        res6 == sim.simx_return_ok && ...
        res7 == sim.simx_return_ok && ...
        res8 == sim.simx_return_ok && ...
        res9 == sim.simx_return_ok )
        disp('Handle del target obtenido correctamente');

       for t = 1:num_iterations
           d    = enemy_obj - Enemy;
           dist = norm(d);
  
           if dist > 1e-9
               Enemy = Enemy + d / dist * min(enemy_Vel, dist);
           end
            trajectory01 (t,:) = [Enemy(1,:), boidAlt];
            target_pos1 = [Enemy(1,1), Enemy(1,2), boidAlt];
            [res1]  = sim.simxSetObjectPosition(clientID, target_handle1,  -1, target_pos1, sim.simx_opmode_blocking);

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

                max_speed = 0.1;
                speed = norm(velocities(i,:));
                if speed > max_speed
                    velocities(i,:) = velocities(i,:) * max_speed / speed;
                end

                 % Actualizar posición defensores 
                defenders(i,:) = defenders(i,:) + velocities(i,:);

                trajectory01(t,:) = [Enemy(1,:),    boidAlt];
                trajectory02(t,:)= [defenders(1,:),boidAlt];
                trajectory03(t,:) = [defenders(2,:),boidAlt];
                trajectory04(t,:) = [defenders(3,:),boidAlt];
                trajectory05(t,:) = [defenders(4,:),boidAlt];
                trajectory06(t,:) = [defenders(5,:),boidAlt];
                trajectory07(t,:) = [defenders(6,:),boidAlt];
                trajectory08(t,:) = [defenders(7,:),boidAlt];
                trajectory09(t,:) = [defenders(8,:),boidAlt];

            end


            if (res1 == sim.simx_return_ok)
                disp('Posición del target establecida correctamente');
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
