"""
Simulación visual de la A-2 (Guadalajara -> Madrid) usando Pygame.
- 3 carriles con límites: derecha 90 km/h, medio 100 km/h, izquierda 120 km/h
- Tipos: coche, camión, ambulancia (prioridades)
- Duración: 2 horas simuladas por defecto
"""

import pygame
import random
import sys
from collections import deque

# CONFIGURACIÓN SIMULACIÓN
L_KM = 60.0
ROAD_LENGTH_M = L_KM * 1000.0
SIM_HOURS = 2.0
SIM_SECONDS = int(SIM_HOURS * 3600)

SPEEDUP = 100.0                # x100 velocidad simulada
ARRIVAL_RATE_PER_MIN = 40.0  # vehículos por minuto en promedio
ARRIVAL_INTERVAL = 60.0 / ARRIVAL_RATE_PER_MIN

DT_REAL = 1/60.0             # paso real por frame objetivo (segundos reales)
FPS = 60                     # tasa de refresco gráfica (frames por segundo)

# Límite de vehículos en escena (prevenir saturación)
MAX_VEHICLES = 2000

# Límite de Velocidad: 0 = derecho (lento), 1 = medio, 2 = izquierdo (rápido)
LANE_SPEED_KMH = [120, 100, 90]
LANE_SPEED_MS = [v * 1000.0 / 3600.0 for v in LANE_SPEED_KMH]

# VEHÍCULOS (parámetros por tipo)
VEH_TYPES = {
    "car": {
        "color": (0, 110, 255),
        "length_m": 4.5,
        "width_m": 1.8,
        "preferred_speed_ms": 120/3.6,
        "accel": 1.5,
        "priority": 2
    },
    "truck": {
        "color": (255, 150, 30),
        "length_m": 12.0,
        "width_m": 2.6,
        "preferred_speed_ms": 85/3.6,
        "accel": 0.9,
        "priority": 1
    },
    "ambulance": {  
        "color": (220, 20, 60),
        "length_m": 4.5,
        "width_m": 1.9,
        "preferred_speed_ms": 140/3.6,
        "accel": 2.0,
        "priority": 3
    }
}

SAFE_TIME_HEADWAY = 1.8   # segundos
MIN_GAP = 2.0             # metros
SLOWDOWN_PROB = 0.04      # probabilidad de frenazo aleatorio

# PYGAME - configuración de pantalla
SCREEN_W = 1200
SCREEN_H = 480
MARGIN = 80
LANE_GAP = (SCREEN_H - 2 * MARGIN) / 3.0
LANE_Y = [MARGIN + i * LANE_GAP + LANE_GAP / 2.0 for i in range(3)]  # y center for lanes

# escala: metros -> pixeles en X
SCALE_X = (SCREEN_W - 2 * MARGIN) / ROAD_LENGTH_M

# representación visual (anchura de vehículo en px proporcional)
def meters_to_px(m): return max(2, int(m * SCALE_X))
def pos_to_px(x_m): return int(MARGIN + x_m * SCALE_X)

# CLASE VEHÍCULO
class Vehicle:
    _id_counter = 1
    def __init__(self, vtype, lane, entry_time):
        self.id = Vehicle._id_counter; Vehicle._id_counter += 1
        self.type = vtype
        self.lane = lane  # 0 left,1 mid,2 right
        self.color = VEH_TYPES[vtype]["color"]
        self.length_m = VEH_TYPES[vtype]["length_m"]
        self.width_m = VEH_TYPES[vtype]["width_m"]
        self.preferred_speed = VEH_TYPES[vtype]["preferred_speed_ms"]
        self.accel = VEH_TYPES[vtype]["accel"]
        self.priority = VEH_TYPES[vtype]["priority"]

        self.x = 0.0  # posición en metros desde inicio
        self.v = self.preferred_speed * 0.6  # veloc inicial m/s
        self.entry_time = entry_time
        self.exit_time = None
        self.finished = False

    def front_pos(self):
        return self.x + self.length_m
    
# A2
class Highway:
    def __init__(self, num_lanes=3):
        self.num_lanes = num_lanes
        self.lanes = [[] for _ in range(num_lanes)]
        self.finished_vehicles = {"car": [], "truck": [], "ambulance": []}

    def add_vehicle(self, veh):
        self.lanes[veh.lane].append(veh)
        self.lanes[veh.lane].sort(key=lambda v: v.x)

    def remove_vehicle(self, veh):
        if veh in self.lanes[veh.lane]:
            self.lanes[veh.lane].remove(veh)

    def vehicle_ahead(self, veh):
        lane = self.lanes[veh.lane]
        ahead = [v for v in lane if v.x > veh.x]
        return min(ahead, key=lambda v: v.x) if ahead else None

    def can_insert_at_start(self, lane, veh_length):
        lane_list = self.lanes[lane]
        if not lane_list:
            return True
        first = min(lane_list, key=lambda v: v.x)
        return first.x > (veh_length + MIN_GAP)

    def try_lane_change(self, veh):
        # simple lane change: try left first (index smaller), then right
        for d in (-1, 1):
            target = veh.lane + d
            if target < 0 or target >= self.num_lanes:
                continue
            lane_list = self.lanes[target]
            ahead = [v for v in lane_list if v.x > veh.x]
            behind = [v for v in lane_list if v.x <= veh.x]
            va = min(ahead, key=lambda v: v.x) if ahead else None
            vb = max(behind, key=lambda v: v.x) if behind else None

            gap_a = (va.x - va.length_m - veh.x) if va else float("inf")
            gap_b = (veh.x - (vb.x + vb.length_m)) if vb else float("inf")
            required = max(MIN_GAP, veh.v * SAFE_TIME_HEADWAY)

            # emergency more aggressive: easier change
            if veh.type == "ambulance":
                if gap_a > MIN_GAP and gap_b > MIN_GAP:
                    self._move_vehicle(veh, target); return True

            # otherwise require required gap
            if gap_a > required and gap_b > required:
                # if vehicle ahead in target has higher priority, skip
                if va and va.priority > veh.priority:
                    continue
                self._move_vehicle(veh, target)
                return True
        return False

    def _move_vehicle(self, veh, target):
        try:
            self.lanes[veh.lane].remove(veh)
        except ValueError:
            pass
        veh.lane = target
        self.lanes[target].append(veh)
        self.lanes[target].sort(key=lambda v: v.x)

# UTILIDADES ESTADÍSTICAS
def compute_finished_stats(highway):
    results = {}
    for typ in ["car","truck","ambulance"]:
        finished = highway.finished_vehicles[typ]
        count = len(finished)
        if count == 0:
            avg_kmh = 0.0
        else:
            # compute average speed per vehicle = distance / travel_time
            speeds = []
            for veh in finished:
                travel = veh.exit_time - veh.entry_time
                if travel > 0:
                    speed_ms = ROAD_LENGTH_M / travel
                    speeds.append(speed_ms * 3.6)
            avg_kmh = sum(speeds)/len(speeds) if speeds else 0.0
        results[typ] = (count, avg_kmh)
    return results

# INICIALIZACIÓN PYGAME
pygame.init()
screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
pygame.display.set_caption("Simulación A-2 — Guadalajara → Madrid (x5)")
font = pygame.font.SysFont("Arial", 16)
clock = pygame.time.Clock()

# SIMULACIÓN PRINCIPAL
def run_pygame_sim():
    highway = Highway(num_lanes=3)
    sim_time = 0.0   # segundos simulados
    next_arrival = 0.0
    entry_queue = deque()

    # For stats
    finished_counts = {"car":0,"truck":0,"ambulance":0}

    running = True
    paused = False

    while running:
        # handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    paused = not paused

        # frame time real
        real_dt = clock.tick(FPS) / 1000.0  # segundos reales passed this frame
        if paused:
            continue

        # simulate scaled dt
        sim_dt = real_dt * SPEEDUP
        sim_time += sim_dt

        # arrivals: create vehicle types and add to entry queue
        while sim_time >= next_arrival and sim_time <= SIM_SECONDS:
            vtype = random.choices(["car","truck","ambulance"], weights=[0.82,0.15,0.03])[0]
            entry_queue.append((vtype, sim_time))
            next_arrival += ARRIVAL_INTERVAL

        # try to insert from queue to lanes (priority: ambulance first by queue order)
        if entry_queue:
            # order queue by priority (ambulance first)
            entry_list = sorted(list(entry_queue), key=lambda x: VEH_TYPES[x[0]]["priority"], reverse=True)
            # try to insert the first (highest priority)
            vtype_try, t_entry = entry_list[0]
            inserted = False
            for lane_try in [0,1,2]:  # prefer left if possible? keep lanes shuffled to distribute
                if highway.can_insert_at_start(lane_try, VEH_TYPES[vtype_try]["length_m"]):
                    newveh = Vehicle(vtype_try, lane_try, t_entry)
                    highway.add_vehicle(newveh)
                    # remove from entry_queue first matching element
                    for idx,(vt,tt) in enumerate(entry_queue):
                        if vt==vtype_try and tt==t_entry:
                            entry_queue.remove((vt,tt))
                            break
                    inserted = True
                    break
            # if not inserted, we keep it in queue (congestion)

        # Update each vehicle
        for lane_idx in range(highway.num_lanes):
            for veh in list(highway.lanes[lane_idx]):
                # find vehicle ahead
                ahead = highway.vehicle_ahead(veh)
                # lane speed limit (m/s)
                lane_limit = LANE_SPEED_MS[veh.lane]
                # desired speed baseline: min(preferred, lane_limit)
                desired_top = min(veh.preferred_speed, lane_limit)

                if ahead:
                    gap = ahead.x - (veh.x + veh.length_m)
                    safe_v = gap / max(0.1, SAFE_TIME_HEADWAY)
                    target_v = min(desired_top, safe_v)
                    if veh.v > target_v:
                        veh.v -= min(veh.v - target_v, 6.0 * sim_dt)
                    else:
                        veh.v += min(target_v - veh.v, veh.accel * sim_dt)
                else:
                    veh.v += min(desired_top - veh.v, veh.accel * sim_dt)

                # random slowdown
                if random.random() < SLOWDOWN_PROB:
                    veh.v = max(0.0, veh.v - 3.0)

                # lane change attempt (ambulance aggressive)
                highway.try_lane_change(veh)

                # move
                veh.x += veh.v * sim_dt

                # finished?
                if veh.x >= ROAD_LENGTH_M:
                    veh.exit_time = sim_time
                    veh.finished = True
                    highway.finished_vehicles[veh.type].append(veh)
                    highway.remove_vehicle(veh)

        # DIBUJO
        screen.fill((25,25,25))

        # draw lanes background
        for i in range(3):
            y = int(LANE_Y[i])
            pygame.draw.line(screen, (80,80,80), (MARGIN, y - 30), (SCREEN_W - MARGIN, y - 30), 28)
            # dashed center lines
            for xpx in range(int(MARGIN), SCREEN_W - MARGIN, 30):
                pygame.draw.line(screen, (40,40,40), (xpx, y - 30), (xpx + 15, y - 30), 4)

        # draw vehicles
        for lane_idx in range(highway.num_lanes):
            for veh in highway.lanes[lane_idx]:
                xpx = pos_to_px(veh.x)
                ypx = int(LANE_Y[veh.lane])
                w = max(6, meters_to_px(veh.length_m))
                h = 12 if veh.type=="truck" else 8
                rect = pygame.Rect(xpx - w//2, ypx - h//2, w, h)
                pygame.draw.rect(screen, veh.color, rect)
                # small ID text
                # id_surf = font.render(str(veh.id), True, (220,220,220))
                # screen.blit(id_surf, (xpx - w//2, ypx - h//2 - 12))

        # HUD
        info1 = f"Sim time: {sim_time/60:.1f} min   Vehicles on road: {sum(len(l) for l in highway.lanes)}   Queue: {len(entry_queue)}"
        info2 = f"Lane speed limits (km/h): left {LANE_SPEED_KMH[0]} - mid {LANE_SPEED_KMH[1]} - right {LANE_SPEED_KMH[2]}   Speed x{int(SPEEDUP)}"
        surf1 = font.render(info1, True, (240,240,240))
        surf2 = font.render(info2, True, (200,200,200))
        screen.blit(surf1, (10,10))
        screen.blit(surf2, (10,30))

        # finished counts
        finished_counts = {t: len(highway.finished_vehicles[t]) for t in ["car","truck","ambulance"]}
        ftext = f"Finished -> cars: {finished_counts['car']}   trucks: {finished_counts['truck']}   ambulances: {finished_counts['ambulance']}"
        screen.blit(font.render(ftext, True, (220,220,220)), (10, 50))

        # draw legend
        def draw_legend(x,y,color,label):
            pygame.draw.rect(screen, color, (x,y,14,8))
            screen.blit(font.render(label, True, (220,220,220)), (x+18,y-2))
        draw_legend(SCREEN_W-300, 10, (0,110,255), "Car")
        draw_legend(SCREEN_W-300, 30, (255,150,30), "Truck")
        draw_legend(SCREEN_W-300, 50, (220,20,60), "Ambulance")

        pygame.display.flip()

        # Condiciones de Cortar Simulación: sim_time passes SIM_SECONDS
        if sim_time >= SIM_SECONDS:
            running = False

    # Fin de la simulación
    results = compute_finished_stats(highway)
    print("\n===== RESULTADOS FINALES (2 horas) =====")
    for typ in ["car","truck","ambulance"]:
        count, avg_kmh = results[typ]
        print(f"{typ.upper():8s} - Llegaron: {count:4d}   Vel.media: {avg_kmh:6.2f} km/h")

    # Mantener hasta cerrar la simulación Final
    finished_text = "Simulación finalizada. Cierra la ventana para salir."
    done_surf = font.render(finished_text, True, (240,240,240))
    screen.blit(done_surf, (10, SCREEN_H - 30))
    pygame.display.flip()

    # Esperar a que se cierre manualmente
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit(); sys.exit()
        clock.tick(10)

if __name__ == "__main__":
    run_pygame_sim()