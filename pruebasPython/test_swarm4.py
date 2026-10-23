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
# Drones (trj1 ... trj8): columnas [x, y, iteración, colisión (0/1)]
trj1 = np.zeros((num_iterations, 4))
trj2 = np.zeros((num_iterations, 4))
trj3 = np.zeros((num_iterations, 4))
trj4 = np.zeros((num_iterations, 4))
trj5 = np.zeros((num_iterations, 4))
trj6 = np.zeros((num_iterations, 4))
trj7 = np.zeros((num_iterations, 4))
trj8 = np.zeros((num_iterations, 4))
trj  = [trj1, trj2, trj3, trj4, trj5, trj6, trj7, trj8]

# Leader: columnas [x, y, iteración]
trj_leader = np.zeros((num_iterations, 3))

# ============================================================
# REGISTRO DE COLISIONES CON EL LEADER
# ============================================================
radio_colision  = 0.1      # distancia de contacto dron-leader (configurable)
hubo_colision   = False     # variable global: True si algún dron tocó al leader
eventos_colision = []       # lista de (iteración, número de dron)

# ============================================================
# INICIALIZACIÓN DE POSICIONES DE LOS DRONES
# modo_posiciones = "aleatorio" -> posiciones aleatorias dentro del tablero
# modo_posiciones = "manual"    -> posiciones definidas en posiciones_manuales
# ============================================================
modo_posiciones = "manual"

# Una fila por dron: [x, y]  (deben ser num_boids filas)
posiciones_manuales = np.array([
    [-6.0,  6.0],   # dron 1
    [ 0.0,  7.0],   # dron 2
    [ 6.0,  6.0],   # dron 3
    [ 7.0,  0.0],   # dron 4
    [ 6.0, -6.0],   # dron 5
    [ 0.0, -7.0],   # dron 6
    [-6.0, -6.0],   # dron 7
    [-7.0,  0.0],   # dron 8
])

# Guardar las posiciones iniciales en un CSV (para MATLAB / CoppeliaSim)
guardar_posiciones_csv = True
archivo_posiciones     = "posiciones_iniciales.csv"

if modo_posiciones == "manual":
    # Verificar cantidad de drones
    if posiciones_manuales.shape != (num_boids, 2):
        raise ValueError(
            f"posiciones_manuales debe tener {num_boids} filas de [x, y]; "
            f"tiene la forma {posiciones_manuales.shape}"
        )
    # Verificar que todas las posiciones estén dentro del tablero
    for k, (px, py) in enumerate(posiciones_manuales, start=1):
        if not (range_x[0] <= px <= range_x[1] and range_y[0] <= py <= range_y[1]):
            raise ValueError(
                f"El dron {k} ({px}, {py}) está fuera del tablero: "
                f"range_x = {range_x}, range_y = {range_y}"
            )
    positions = posiciones_manuales.astype(float).copy()
else:
    positions = (
        np.random.rand(num_boids, 2)
        * np.array([ range_x[1] - range_x[0], range_y[1] - range_y[0]])
        + np.array([range_x[0],range_y[0]])
    )

if guardar_posiciones_csv:
    np.savetxt(archivo_posiciones, positions, delimiter=",",
               header="x,y", comments="", fmt="%.4f")
    print(f"Posiciones iniciales guardadas en {archivo_posiciones}")

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
    for k in range(num_boids):
        dist_k  = np.linalg.norm(positions[k, :] - leader)
        colision = 1 if dist_k < radio_colision else 0

        trj[k][t, 0:2] = positions[k, :]
        trj[k][t, 2]   = t
        trj[k][t, 3]   = colision

        if colision:
            hubo_colision = True
            eventos_colision.append((t, k + 1))

    trj_leader[t, 0:2] = leader
    trj_leader[t, 2]   = t
    n_iter = t + 1          # iteraciones efectivamente simuladas

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
        t_objetivo      = t
        print(f"El leader alcanzó el objetivo en la iteración {t}")

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
        trj_leader[:t + 1, 0],
        trj_leader[:t + 1, 1],
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

# ============================================================
# RESULTADO DE LA CORRIDA
# ============================================================
# Recortar los registros a las iteraciones efectivamente simuladas
trj        = [m[:n_iter, :] for m in trj]
trj1, trj2, trj3, trj4, trj5, trj6, trj7, trj8 = trj
trj_leader = trj_leader[:n_iter, :]

# La defensa es exitosa si hubo contacto ANTES de que el leader
# alcanzara el objetivo (o si nunca lo alcanzó)
if not defensa_fallida:
    t_objetivo = None
defensa_exitosa = hubo_colision and (
    t_objetivo is None or eventos_colision[0][0] < t_objetivo
)

print("--------------------------------------------------")
if defensa_exitosa:
    t_primera, dron_primero = eventos_colision[0]
    print("¡¡DEFENSA EXITOSA!!")
    print(f"Primer contacto: dron {dron_primero} en la iteración {t_primera}")
    print(f"Total de registros de contacto: {len(eventos_colision)}")
elif hubo_colision:
    print("DEFENSA FALLIDA: el primer contacto ocurrió después de que "
          f"el leader alcanzara el objetivo (iteración {t_objetivo})")
elif t_objetivo is not None:
    print(f"DEFENSA FALLIDA: el leader alcanzó el objetivo en la iteración "
          f"{t_objetivo} sin contactos a menos de {radio_colision}")
else:
    print("SIN RESULTADO: no hubo contacto y el leader no alcanzó el "
          "objetivo en las iteraciones simuladas")
print("--------------------------------------------------")

plt.show()