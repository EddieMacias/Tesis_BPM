import numpy as np
import matplotlib.pyplot as plt

# ============================================================
# FUNCIÓN: POSICIÓN INICIAL ALEATORIA DEL LEADER
# modo = "tablero"   -> cualquier punto dentro del área
# modo = "perimetro" -> un punto sobre uno de los cuatro bordes
# ============================================================
def posicion_inicial_leader(range_x, range_y, modo="tablero"):

    if modo == "perimetro":
        lado = np.random.randint(4)
        if lado == 0:   # borde izquierdo
            return np.array([range_x[0], np.random.uniform(*range_y)])
        if lado == 1:   # borde derecho
            return np.array([range_x[1], np.random.uniform(*range_y)])
        if lado == 2:   # borde inferior
            return np.array([np.random.uniform(*range_x), range_y[0]])
        return np.array([np.random.uniform(*range_x), range_y[1]])   # borde superior

    # Punto aleatorio dentro del tablero
    return np.array([np.random.uniform(*range_x), np.random.uniform(*range_y)])

# ============================================================
# FUNCIÓN: AVANCE DEL LEADER HACIA EL OBJETIVO
# Avanza un paso en línea recta sin sobrepasar el objetivo
# ============================================================
def mover_hacia_objetivo(pos, objetivo, paso):

    d    = objetivo - pos
    dist = np.linalg.norm(d)

    if dist < 1e-9:
        return pos.copy()

    return pos + d / dist * min(paso, dist)

# ============================================================
# PARÁMETROS DE TABLERO
# ============================================================
num_boids       = 8
range_x         = [-10, 10]
range_y         = [-10, 10]
obstacle_radius = 0.3
num_iterations  = 1000

# ============================================================
# PARÁMETROS DEL DRON HOSTIL (LEADER)
# ============================================================
pos_obj_leader_x = 0.0                   # coordenada x del objetivo (activo protegido)
pos_obj_leader_y = 0.0                   # coordenada y del objetivo (activo protegido)

# Verificar que el objetivo esté dentro del tablero
if not (range_x[0] <= pos_obj_leader_x <= range_x[1]
        and range_y[0] <= pos_obj_leader_y <= range_y[1]):
    raise ValueError(
        f"El objetivo ({pos_obj_leader_x}, {pos_obj_leader_y}) está fuera del tablero: "
        f"range_x = {range_x}, range_y = {range_y}"
    )

objetivo_leader = np.array([pos_obj_leader_x, pos_obj_leader_y])
modo_inicio     = "tablero"              # "tablero" o "perimetro"
paso_leader     = 0.08                   # avance del leader por iteración
radio_objetivo  = 0.5                    # radio de defensa fallida

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
leader = posicion_inicial_leader(range_x, range_y, modo_inicio)
defensa_fallida = False

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
    leader = mover_hacia_objetivo(leader, objetivo_leader, paso_leader)

    # ========================================================
    # VERIFICAR SI EL LEADER ALCANZÓ EL OBJETIVO
    # ========================================================
    if np.linalg.norm(leader - objetivo_leader) < radio_objetivo:
        defensa_fallida = True
        print(f"Defensa fallida: el leader alcanzó el objetivo en la iteración {t}")

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
    # OBJETIVO DEL LEADER (ACTIVO PROTEGIDO)
    # --------------------------------------------------------
    plt.scatter(
        objetivo_leader[0],
        objetivo_leader[1],
        s=250,
        c="navy",
        marker="s",
        label="Objetivo"
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

    # Terminar la corrida si el leader alcanzó el objetivo
    if defensa_fallida:
        break

# ============================================================
# FINALIZAR
# ============================================================

plt.show()