for t = 1:num_iterations
    % Calcular centro de masa
    center_of_mass = leader;

    trj1 (t,:)=positions(1,:);
    trj2 (t,:)=positions(2,:);
    trj3 (t,:)=positions(3,:);
    trj4 (t,:)=positions(4,:);
    trj6 (t,:)=leader;

    % Calcular dirección de movimiento para cada boid
    for i = 1:num_boids
        % Regla de cohesión
        cohesion = (center_of_mass - positions(i,:)) * cohesion_factor;

        % Regla de separación
        separation = zeros(1,2);
        for j = 1:num_boids
            if i ~= j
                dist = norm(positions(i,:) - positions(j,:));
                if dist < 0.1 % Evitar división por cero
                    separation = separation + rand(1,2); % Aplicar un pequeño alejamiento aleatorio
                elseif dist < 0.5
                    separation = separation - (positions(j,:) - positions(i,:)) / dist^2;
                end
            end
        end
        separation = separation * separation_factor;

        % Regla de separación de lider
        Leader_avoidance = zeros(1,2);
        for k = 1:size(leader, 1)
            dist_to_leader = norm(positions(i,:) - leader);
            if dist_to_leader < leader_separation_distance
                Leader_avoidance = Leader_avoidance + (positions(i,:) - leader) / dist_to_leader^2;
            end
        end
        Leader_avoidance = Leader_avoidance * leader_avoidande_factor;

        % Regla de alineación
        alignment = mean(velocities) * alignment_factor;                            

        % Actualizar velocidad
        velocities(i,:) = velocities(i,:) + cohesion + separation + alignment + Leader_avoidance;

        % Limitar velocidad máxima
        max_speed = 0.1;
        speed = norm(velocities(i,:));
        if speed > max_speed
            velocities(i,:) = velocities(i,:) * max_speed / speed;
        end

        % Actualizar posición boids
        positions(i,:) = positions(i,:) + velocities(i,:);

        % Limitar posición dentro del rango
        positions(i,1) = min(max(positions(i,1), range_x(1)), range_x(2));
        positions(i,2) = min(max(positions(i,2), range_y(1)), range_y(2));
    end