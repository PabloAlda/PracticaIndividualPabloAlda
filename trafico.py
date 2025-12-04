#!/usr/bin/env python3
# traffic_a2_visual.py
# Simulación visual en tiempo real del tráfico en la A-2 (Guadalajara -> Madrid)

import random
import csv
import time
import matplotlib.pyplot as plt
from math import floor

# =========================
# PARÁMETROS
# =========================
L_KM = 60.0
NUM_LANES = 3
SIM_SECONDS = 3600 * 2          # 2 horas simuladas (puedes cambiarlo)
DT = 1.0                        # paso de tiempo
ARRIVAL_RATE_PER_MIN = 40.0     # densidad
MAX_SPEED_KMH = 120.0
SLOWDOWN_PROB = 0.02
SAFE_TIME_HEADWAY = 1.8
MIN_GAP = 2.0
VEHICLE_LENGTH = 4.5
SEED = 42
SPEEDUP = 20                    # aceleración visual: 20× más rápido
MAX_TRAJ = 300_000

random.seed(SEED)

def kmh_to_ms(v): return v * 1000.0 / 3600.0
def ms_to_kmh(v): return v * 3.6

ROAD_LENGTH_M = L_KM * 1000.0
MAX_SPEED = kmh_to_ms(MAX_SPEED_KMH)

# ====================================================
# TIPOS DE VEHÍCULOS Y PRIORIDADES
# ====================================================
# Prioridad alta → pasa antes en cambios de carril + frena menos
VEHICLE_TYPES = {
    "car":        {"color": "blue",   "priority": 1, "max_speed": MAX_SPEED},
    "truck":      {"color": "orange", "priority": 0, "max_speed": MAX_SPEED * 0.7},
    "ambulance":  {"color": "red",    "priority": 2, "max_speed": MAX_SPEED * 1.1}
}

# ====================================================
# CLASE VEHÍCULO
# ====================================================
class Vehicle:
    _next_id = 1
    def __init__(self, lane, vtype, x=0.0):
        self.id = Vehicle._next_id
        Vehicle._next_id += 1
        self.lane = lane
        self.x = x
        self.vtype = vtype
        self.v = VEHICLE_TYPES[vtype]["max_speed"] * 0.6
        self.priority = VEHICLE_TYPES[vtype]["priority"]
        self.length = VEHICLE_LENGTH
        self.finished = False
        self.entry_time = None
        self.exit_time = None

    def front(self):
        return self.x + self.length

# ====================================================
# AUTOPISTA
# ====================================================
class Highway:
    def __init__(self, length_m, num_lanes):
        self.length = length_m
        self.num_lanes = num_lanes
        self.lanes = [[] for _ in range(num_lanes)]
        self.all = []

    def add(self, veh):
        lane = self.lanes[veh.lane]
        lane.append(veh)
        lane.sort(key=lambda v: v.x)
        self.all.append(veh)

    def remove(self, veh):
        if veh in self.lanes[veh.lane]:
            self.lanes[veh.lane].remove(veh)
        if veh in self.all:
            self.all.remove(veh)

    def ahead(self, veh):
        lane = self.lanes[veh.lane]
        ahead = [v for v in lane if v.x > veh.x]
        return min(ahead, key=lambda v: v.x) if ahead else None

    def try_lane_change(self, veh):
        for d in (-1, 1):
            target = veh.lane + d
            if target < 0 or target >= self.num_lanes: continue
            lane_list = self.lanes[target]

            ahead = [v for v in lane_list if v.x > veh.x]
            behind = [v for v in lane_list if v.x <= veh.x]
            va = min(ahead, key=lambda v: v.x) if ahead else None
            vb = max(behind, key=lambda v: v.x) if behind else None

            gap_a = (va.x - va.length - veh.x) if va else float("inf")
            gap_b = (veh.x - (vb.x + vb.length)) if vb else float("inf")

            need = max(MIN_GAP, veh.v * SAFE_TIME_HEADWAY / 2)

            # prioridad afecta facilidad para cambiar
            if gap_a > need and gap_b > need:
                # si tiene más prioridad que el de delante, cambia antes
                if va and veh.priority < va.priority: continue

                self.lanes[veh.lane].remove(veh)
                veh.lane = target
                self.lanes[target].append(veh)
                self.lanes[target].sort(key=lambda v: v.x)
                return True
        return False

# ====================================================
# SIMULACIÓN CON VISUALIZACIÓN
# ====================================================
def run_sim():

    highway = Highway(ROAD_LENGTH_M, NUM_LANES)
    t = 0.0
    next_arrival = 0.0
    arrival_interval = 60.0 / ARRIVAL_RATE_PER_MIN

    # Estadísticas
    stats = {"car": [], "truck": [], "ambulance": []}

    # Matplotlib ventana
    plt.ion()
    fig, ax = plt.subplots(figsize=(12, 5))

    start_real = time.time()
    traj_count = 0

    while t < SIM_SECONDS:

        # ===========================
        # Entrada de vehículos
        # ===========================
        if t >= next_arrival:
            vtype = random.choices(
                ["car", "truck", "ambulance"],
                weights=[0.85, 0.13, 0.02]
            )[0]
            lanes = list(range(NUM_LANES))
            random.shuffle(lanes)
            placed = False
            for lane in lanes:
                if not highway.lanes[lane]:
                    v = Vehicle(lane, vtype)
                    v.entry_time = t
                    highway.add(v)
                    placed = True
                    break
                else:
                    first = min(highway.lanes[lane], key=lambda v: v.x)
                    if first.x > VEHICLE_LENGTH + MIN_GAP:
                        v = Vehicle(lane, vtype)
                        v.entry_time = t
                        highway.add(v)
                        placed = True
                        break
            next_arrival += arrival_interval

        # ===========================
        # Dinámica
        # ===========================
        for lane in range(NUM_LANES):
            for veh in list(highway.lanes[lane]):
                ahead = highway.ahead(veh)

                # Intenta cambio de carril si el de delante frena
                if ahead:
                    dist = ahead.x - (veh.x + veh.length)
                    if dist < veh.v * SAFE_TIME_HEADWAY:
                        highway.try_lane_change(veh)

                ahead = highway.ahead(veh)

                accel = 1.5
                decel = 6.0

                if ahead:
                    gap = ahead.x - (veh.x + veh.length)
                    safe_v = gap / SAFE_TIME_HEADWAY
                    safe_v = min(safe_v, VEHICLE_TYPES[veh.vtype]["max_speed"])

                    if veh.v > safe_v:
                        veh.v -= min(veh.v - safe_v, decel * DT)
                    else:
                        veh.v += min(safe_v - veh.v, accel * DT)
                else:
                    top = VEHICLE_TYPES[veh.vtype]["max_speed"]
                    veh.v += min(top - veh.v, accel * DT)

                if random.random() < SLOWDOWN_PROB:
                    veh.v = max(0, veh.v - 3.0)

                veh.x += veh.v * DT

                if veh.x >= highway.length:
                    veh.exit_time = t
                    veh.finished = True
                    stats[veh.vtype].append(veh)
                    highway.remove(veh)

        t += DT

        # ===========================
        # VISUALIZACIÓN EN TIEMPO REAL
        # ===========================
        if time.time() - start_real >= 1 / SPEEDUP:
            start_real = time.time()

            ax.clear()
            ax.set_title(f"Simulación A-2 Guadalajara → Madrid | t = {t/60:.1f} min")
            ax.set_xlim(0, ROAD_LENGTH_M)
            ax.set_ylim(-1, NUM_LANES)
            ax.set_xlabel("Distancia (m)")
            ax.set_ylabel("Carril")

            for v in highway.all:
                ax.scatter(v.x, v.lane, s=20, c=VEHICLE_TYPES[v.vtype]["color"])

            ax.grid(True)
            plt.pause(0.001)

    plt.ioff()
    plt.show()

    # RESULTADOS
    print("\n===== RESULTADOS =====\n")
    for vtype in ["car", "truck", "ambulance"]:
        finished = stats[vtype]
        print(f"{vtype.upper()}: {len(finished)} vehículos llegaron")
        if finished:
            avg_speed = sum((veh.exit_time - veh.entry_time) for veh in finished)
            print(f"  Velocidad media: {round((60*60*L_KM) / (avg_speed/len(finished)), 2)} km/h")
        print()

if __name__ == "__main__":
    run_sim()
