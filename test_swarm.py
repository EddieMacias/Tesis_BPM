import numpy as np
import matplotlib.pyplot as plt

# ============================================================
# FUNCIÓN: ROTACIÓN DEL LEADER
# Equivalente a RotateLeaderPosition.m
# ============================================================
def rotate_leader_position(actual_position, obstacle):

    # Equivalente a:
    # desiredAngle = randi(180);
    desired_angle = np.random.randint(1, 181)

    # Equivalente al switch de MATLAB
    if obstacle == "L":
        if desired_angle >= 90:
            desired_angle = desired_angle + 180
    elif obstacle == "U":
        desired_angle = desired_angle + 180
    elif obstacle == "R":
        desired_angle = desired_angle + 90
    elif obstacle == "D":
        desired_angle = np.random.randint(1, 181)
    else:
        desired_angle = 90

    # Conversión grados → radianes
    angle_rad = np.deg2rad(desired_angle)

    # Matriz de rotación
    R = np.array([
        [np.cos(angle_rad), -np.sin(angle_rad)],
        [np.sin(angle_rad),  np.cos(angle_rad)]
    ])

    # Aplicar rotación
    leader_avoidance_vector = R @ np.asarray(actual_position)
    return leader_avoidance_vector

# ============================================================
# PARÁMETROS DE TABLERO
# ============================================================
num_boids       = 8
range_x         = [-10, 10]
range_y         = [-10, 10]
obstacle_radius = 0.3
num_iterations  = 1000

# ============================================================
# MATRICES PARA ALMACENAR TRAYECTORIAS
# ============================================================
trj1 = np.zeros((num_iterations, 2))
trj2 = np.zeros((num_iterations, 2))
trj3 = np.zeros((num_iterations, 2))
trj4 = np.zeros((num_iterations, 2))
trj5 = np.zeros((num_iterations, 2))
trj6 = np.zeros((num_iterations, 2))
trj7 = np.zeros((num_iterations, 2))
trj8 = np.zeros((num_iterations, 2))
trj9 = np.zeros((num_iterations, 2))

# ============================================================
# INICIALIZACIÓN DE POSICIONES ALEATORIAS
# ============================================================
positions = (
    np.random.rand(num_boids, 2)
    * np.array([ range_x[1] - range_x[0], range_y[1] - range_y[0]])
    + np.array([range_x[0],range_y[0]])
)

# ============================================================
# INICIALIZACIÓN DE VELOCIDADES
# ============================================================
velocities = np.random.rand(num_boids, 2) - 0.5

# ============================================================
# INICIALIZACIÓN DEL LEADER
# ============================================================
leader = (
    np.random.rand(2)
    * np.array([
        range_x[1] - range_x[0],
        range_y[1] - range_y[0]
    ])
    + np.array([
        range_x[0],
        range_y[0]
    ])
)
# Velocidad inicial del líder
leader_velocity = np.array([1.0, 1.0])

# ============================================================
# PARÁMETROS DEL ALGORITMO BOIDS
# ============================================================
cohesion_factor             = 0.02
separation_factor           = 0.1
alignment_factor            = 0.01
obstacle_avoidance_factor   = 0.1

# ============================================================
# PARÁMETROS LEADER-FOLLOWER
# ============================================================
leader_avoidande_factor    = 0.2
leader_separation_distance = 0.1
leader_vel_factor          = 0.08

# ============================================================
# VELOCIDAD MÁXIMA DE LOS BOIDS
# ============================================================
max_speed = 0.09

# ============================================================
# CONFIGURACIÓN DE LA FIGURA
# ============================================================
plt.figure(figsize=(8, 8))

# ============================================================
# SIMULACIÓN
# ============================================================
for t in range(num_iterations):

    # ========================================================
    # CENTRO DE MASA
    # ========================================================
    center_of_mass = leader.copy()

    # ========================================================
    # ALMACENAR TRAYECTORIAS
    # ========================================================
    trj1[t, :] = positions[0, :]
    trj2[t, :] = positions[1, :]
    trj3[t, :] = positions[2, :]
    trj4[t, :] = positions[3, :]
    trj5[t, :] = positions[4, :]
    trj6[t, :] = leader

    # ========================================================
    # ACTUALIZACIÓN DE CADA BOID
    # ========================================================
    for i in range(num_boids):

        # ====================================================
        # REGLA DE COHESIÓN
        # ====================================================
        cohesion = ( center_of_mass - positions[i, :]) * cohesion_factor

        # ====================================================
        # REGLA DE SEPARACIÓN
        # ====================================================
        separation = np.zeros(2)
        for j in range(num_boids):
            if i != j:
                # Vector entre los dos boids
                distance_vector = ( positions[i, :] - positions[j, :])
                # Distancia
                dist = np.linalg.norm( distance_vector)

                # Evitar división por cero
                if dist < 0.1:
                    separation += np.random.rand(2)
                    
                # Zona de separación
                elif dist < 0.5:
                    separation -= ( positions[j, :] - positions[i, :]) / (dist ** 2)

        # Factor de separación
        separation *= separation_factor

        # ====================================================
        # SEPARACIÓN DEL LEADER
        # ====================================================
        leader_avoidance = np.zeros(2)
        dist_to_leader   = np.linalg.norm( positions[i, :] - leader)

        if dist_to_leader < leader_separation_distance:
            # Protección contra división por cero
            if dist_to_leader > 1e-12:
                leader_avoidance += (positions[i, :] - leader) / (dist_to_leader ** 2)
                
        leader_avoidance *= leader_avoidande_factor

        # ====================================================
        # REGLA DE ALINEACIÓN
        # ====================================================
        alignment = ( np.mean( velocities, axis=0 ) * alignment_factor )

        # ====================================================
        # ACTUALIZAR VELOCIDAD
        # ====================================================
        velocities[i, :] = ( velocities[i, :] + cohesion + separation + alignment + leader_avoidance)

        # ====================================================
        # LIMITAR VELOCIDAD MÁXIMA
        # ====================================================
        speed = np.linalg.norm( velocities[i, :] )

        if speed > max_speed:
            velocities[i, :] = ( velocities[i, :] * max_speed / speed )

        # ====================================================
        # ACTUALIZAR POSICIÓN
        # ====================================================
        positions[i, :] = ( positions[i, :] + velocities[i, :])

        # ====================================================
        # LIMITAR POSICIÓN EN X
        # ====================================================
        positions[i, 0] = np.clip(
            positions[i, 0],
            range_x[0],
            range_x[1]
        )

        # ====================================================
        # LIMITAR POSICIÓN EN Y
        # ====================================================
        positions[i, 1] = np.clip(
            positions[i, 1],
            range_y[0],
            range_y[1]
        )

    # ========================================================
    # ACTUALIZACIÓN DEL LEADER
    # ========================================================
    leader = (
        leader
        + leader_velocity * leader_vel_factor
    )

    # ========================================================
    # DETECTAR LÍMITES DEL ÁREA
    # ========================================================

    if leader[0] < range_x[0]:
        leader_velocity = rotate_leader_position(
            [1, 0],
            "L"
        )

    elif leader[0] > range_x[1]:
        leader_velocity = rotate_leader_position(
            [1, 0],
            "R"
        )

    elif leader[1] < range_y[0]:
        leader_velocity = rotate_leader_position(
            [1, 0],
            "D"
        )

    elif leader[1] > range_y[1]:
        leader_velocity = rotate_leader_position(
            [1, 0],
            "U"
        )

    # ========================================================
    # VISUALIZACIÓN
    # ========================================================
    plt.clf()

    # --------------------------------------------------------
    # BOIDS
    # --------------------------------------------------------
    plt.scatter(
        positions[:, 0],
        positions[:, 1],
        s=100,
        c="magenta",
        marker="*",
        label="Boids"
    )

    # --------------------------------------------------------
    # LEADER
    # --------------------------------------------------------
    plt.scatter(
        leader[0],
        leader[1],
        s=200,
        c="cyan",
        marker="o",
        label="Leader"
    )

    # --------------------------------------------------------
    # TRAYECTORIA DEL LEADER
    # --------------------------------------------------------
    plt.plot(
        trj6[:t + 1, 0],
        trj6[:t + 1, 1],
        label="Trayectoria Leader"
    )

    # --------------------------------------------------------
    # CONFIGURACIÓN DE EJES
    # --------------------------------------------------------
    plt.xlim(range_x)
    plt.ylim(range_y)

    # --------------------------------------------------------
    # GRID
    # --------------------------------------------------------
    plt.grid(
        True,
        which="both"
    )

    # --------------------------------------------------------
    # TÍTULO
    # --------------------------------------------------------
    plt.title(
        "Prueba algoritmo Boids con Leader folower"
    )

    # --------------------------------------------------------
    # LEYENDA
    # --------------------------------------------------------
    plt.legend(
        loc="upper left",
        bbox_to_anchor=(1.02, 1)
    )

    # --------------------------------------------------------
    # ACTUALIZAR ANIMACIÓN
    # --------------------------------------------------------
    plt.pause(0.001)

# ============================================================
# FINALIZAR
# ============================================================

plt.show()