import os
from datetime import datetime
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
range_x         = [-10, 10]     # tablero interno: zona de los drones defensores
range_y         = [-10, 10]
obstacle_radius = 0.3
num_iterations  = 1000

# ============================================================
# PERÍMETRO EXTERIOR (zona de aparición del dron hostil)
# El leader aparece sobre el borde de este perímetro y avanza
# hacia el objetivo, atravesando el tablero interno.
# ============================================================
perimetro_x = [-20, 20]
perimetro_y = [-20, 20]

# Verificar que el perímetro contenga al tablero interno
if not (perimetro_x[0] <= range_x[0] and range_x[1] <= perimetro_x[1]
        and perimetro_y[0] <= range_y[0] and range_y[1] <= perimetro_y[1]):
    raise ValueError(
        f"El perímetro exterior x={perimetro_x}, y={perimetro_y} debe contener "
        f"al tablero interno x={range_x}, y={range_y}"
    )

# True  -> los drones defensores no pueden salir del tablero interno
# False -> los drones defensores pueden moverse por todo el perímetro exterior
drones_limitados_al_tablero = True

# ============================================================
# CARPETA DE RESULTADOS (común para todas las corridas)
# ============================================================
Folder_resultados = r"C:\Users\User\Documents\pythonPrj\Resultados"

# True  -> cada corrida en una subcarpeta propia (corrida_AAAAMMDD_HHMMSS)
# False -> todos los archivos directamente en Folder_resultados,
#          con el identificador de la corrida al inicio del nombre
subcarpeta_por_corrida = True

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
# "perimetro" -> aparece en un punto aleatorio del borde del perímetro exterior
# "tablero"   -> aparece en un punto aleatorio dentro del tablero interno
modo_inicio     = "perimetro"
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
dist_min_propios = np.inf   # menor distancia observada entre dos drones propios

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

# Identificador único de la corrida (fecha y hora), para no sobrescribir
# resultados de corridas anteriores
id_corrida = datetime.now().strftime("corrida_%Y%m%d_%H%M%S")

if subcarpeta_por_corrida:
    carpeta_corrida = os.path.join(Folder_resultados, id_corrida)
    prefijo         = ""
else:
    carpeta_corrida = Folder_resultados
    prefijo         = id_corrida + "_"

os.makedirs(carpeta_corrida, exist_ok=True)      # crea las carpetas si no existen

def ruta(nombre):
    """Ruta completa de un archivo de resultados de esta corrida."""
    return os.path.join(carpeta_corrida, prefijo + nombre)

archivo_posiciones = ruta("posiciones_iniciales.csv")

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
if modo_inicio == "perimetro":
    leader = posicion_inicial_leader(perimetro_x, perimetro_y, "perimetro")
else:
    leader = posicion_inicial_leader(range_x, range_y, "tablero")
defensa_fallida = False

# ============================================================
# PARÁMETROS DEL ALGORITMO BOIDS
# ============================================================
cohesion_factor             = 0.02
separation_factor           = 0.1

# Radios de la regla de separación entre drones defensores
radio_separacion            = 1.0    # distancia a la que un dron empieza a alejarse de otro (antes 0.5)
distancia_minima_drones     = 0.1    # por debajo de esta distancia se aplica un empuje aleatorio
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

    # Distancia mínima entre drones propios (seguridad entre agentes)
    for a in range(num_boids):
        for b in range(a + 1, num_boids):
            dist_min_propios = min(dist_min_propios,
                                   np.linalg.norm(positions[a, :] - positions[b, :]))

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
                if dist < distancia_minima_drones:
                    separation += np.random.rand(2)
                    
                # Zona de separación
                elif dist < radio_separacion:
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
        # LIMITAR POSICIÓN (tablero interno o perímetro exterior)
        # ====================================================
        lim_x = range_x if drones_limitados_al_tablero else perimetro_x
        lim_y = range_y if drones_limitados_al_tablero else perimetro_y

        positions[i, 0] = np.clip(positions[i, 0], lim_x[0], lim_x[1])
        positions[i, 1] = np.clip(positions[i, 1], lim_y[0], lim_y[1])

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
    # Tablero interno (zona de los defensores)
    plt.plot(
        [range_x[0], range_x[1], range_x[1], range_x[0], range_x[0]],
        [range_y[0], range_y[0], range_y[1], range_y[1], range_y[0]],
        "k--", linewidth=1.2, label="Tablero interno"
    )
    plt.xlim(perimetro_x)
    plt.ylim(perimetro_y)
    plt.gca().set_aspect("equal")

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

# ============================================================
# LOG DE LA CORRIDA (CSV)
# ============================================================
# 1) Trayectoria de cada dron: x, y, iteración, colisión
for k, m in enumerate(trj, start=1):
    np.savetxt(ruta(f"trayectoria_dron_{k}.csv"), m,
               delimiter=",", header="x,y,iteracion,colision", comments="",
               fmt=["%.4f", "%.4f", "%d", "%d"])

# 2) Trayectoria del leader: x, y, iteración
np.savetxt(ruta("trayectoria_leader.csv"), trj_leader,
           delimiter=",", header="x,y,iteracion", comments="",
           fmt=["%.4f", "%.4f", "%d"])

# 3) Todas las trayectorias en un solo archivo (formato largo):
#    agente, iteración, x, y, colisión   (agente 0 = leader)
filas = [[0, int(r[2]), r[0], r[1], 0] for r in trj_leader]
for k, m in enumerate(trj, start=1):
    filas += [[k, int(r[2]), r[0], r[1], int(r[3])] for r in m]
np.savetxt(ruta("trayectorias_todas.csv"), np.array(filas),
           delimiter=",", header="agente,iteracion,x,y,colision", comments="",
           fmt=["%d", "%d", "%.4f", "%.4f", "%d"])

# 4) Eventos de colisión: iteración, dron
np.savetxt(ruta("colisiones.csv"),
           np.array(eventos_colision, dtype=int).reshape(-1, 2),
           delimiter=",", header="iteracion,dron", comments="", fmt="%d")

# 5) Resumen de la corrida
with open(ruta("resumen.txt"), "w", encoding="utf-8") as f:
    f.write(f"Corrida: {id_corrida}\n")
    f.write(f"Iteraciones simuladas: {n_iter} de {num_iterations}\n")
    f.write(f"Tablero interno: range_x = {range_x}, range_y = {range_y}\n")
    f.write(f"Perímetro exterior: perimetro_x = {perimetro_x}, perimetro_y = {perimetro_y}\n")
    f.write(f"Modo de inicio del leader: {modo_inicio}\n")
    f.write(f"Drones limitados al tablero interno: {drones_limitados_al_tablero}\n")
    f.write(f"Objetivo del leader: ({pos_obj_leader_x}, {pos_obj_leader_y})\n")
    f.write(f"Posición inicial del leader: ({trj_leader[0, 0]:.4f}, {trj_leader[0, 1]:.4f})\n")
    f.write(f"Modo de posiciones de los drones: {modo_posiciones}\n")
    f.write(f"Radio de colisión: {radio_colision}\n")
    f.write(f"Radio de separación entre drones: {radio_separacion}\n")
    f.write(f"Distancia mínima entre drones: {distancia_minima_drones}\n")
    f.write(f"Distancia mínima observada entre drones propios: {dist_min_propios:.4f}\n")
    f.write(f"Leader alcanzó el objetivo: {'sí, iteración ' + str(t_objetivo) if t_objetivo is not None else 'no'}\n")
    f.write(f"Registros de contacto: {len(eventos_colision)}\n")
    if eventos_colision:
        f.write(f"Primer contacto: dron {eventos_colision[0][1]} en la iteración {eventos_colision[0][0]}\n")
    f.write(f"Resultado: {'DEFENSA EXITOSA' if defensa_exitosa else 'DEFENSA FALLIDA / SIN RESULTADO'}\n")

print(f"Log de la corrida guardado en: {os.path.abspath(carpeta_corrida)}")

plt.show()