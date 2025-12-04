#!/usr/bin/env python3
# traffic_a2_with_priority.py
# Simulación del tráfico en la A-2 con colas de entrada y prioridades de vehículos

import random
import csv

# =======================
# Parámetros de la simulación
# =======================
L_KM = 60.0
NUM_LANES = 3
SIM_SECONDS = 3600 * 1
DT = 1.0
ARRIVAL_RATE_PER_MIN = 30.0
MAX_SPEED_KMH = 120.0
SLOWDOWN_PROB = 0.02
SAFE_TIME_HEADWAY = 1.8
MIN_GAP = 2.0
VEHICLE_LENGTH = 4.5
SEED = 42
MAX_LINES = 300000

random.seed(SEED)

def kmh_to_ms(v_kmh):
    return v_kmh * 1000.0 / 3600.0

def ms_to_kmh(v_ms):
    return v_ms * 3.6

ROAD_LENGTH_M = L_KM * 1000.0
MAX_SPEED = kmh_to_ms(MAX_SPEED_KMH)

# =======================
# Clases
# =======================
class Vehicle:
    _next_id = 1
    def __init__(self, lane, x=0.0, v=None, type_="car"):
        """
        type_ define la prioridad del vehículo:
        - "emergency": prioridad máxima, puede entrar primero y cambiar carril con preferencia
        - "truck": prioridad media, suele ser más lento y ocupa más espacio
        - "car": prioridad normal
        """
        self.id = Vehicle._next_id
        Vehicle._next_id += 1
        self.lane = lane
        self.x = x
        self.v = MAX_SPEED if v is None else v
        self.length = VEHICLE_LENGTH
        self.finished = False
        self.entry_time = None
        self.exit_time = None
        self.type = type_
        self.priority = {"emergency": 3, "truck": 2, "car": 1}[type_]

class Highway:
    def __init__(self, length_m, num_lanes):
        self.length = length_m
        self.num_lanes = num_lanes
        self.lanes = [[] for _ in range(num_lanes)]
        self.all_vehicles = []

    def add_vehicle(self, veh):
        lane = self.lanes[veh.lane]
        lane.append(veh)
        lane.sort(key=lambda v: v.x)
        self.all_vehicles.append(veh)

    def remove_vehicle(self, veh):
        if veh in self.lanes[veh.lane]:
            self.lanes[veh.lane].remove(veh)
        if veh in self.all_vehicles:
            self.all_vehicles.remove(veh)

    def vehicle_ahead(self, veh):
        lane = self.lanes[veh.lane]
        vehicles_ahead = [v for v in lane if v.x > veh.x]
        if vehicles_ahead:
            return min(vehicles_ahead, key=lambda v: v.x)
        return None

    def try_change_lane(self, veh):
        # Intento de cambio de carril con prioridad
        # Los vehículos con mayor prioridad tienen más probabilidad de cambiar de carril si hay hueco
        for d in (-1, 1):
            target = veh.lane + d
            if target < 0 or target >= self.num_lanes:
                continue
            lane_list = self.lanes[target]
            ahead = [v for v in lane_list if v.x > veh.x]
            behind = [v for v in lane_list if v.x <= veh.x]
            veh_ahead = min(ahead, key=lambda v: v.x) if ahead else None
            veh_behind = max(behind, key=lambda v: v.x) if behind else None

            gap_ahead = (veh_ahead.x - veh_ahead.length - veh.x) if veh_ahead else float('inf')
            gap_behind = (veh.x - (veh_behind.x + veh_behind.length)) if veh_behind else float('inf')

            # Si hay hueco suficiente y la prioridad permite el cambio
            if gap_ahead > max(MIN_GAP, veh.v * SAFE_TIME_HEADWAY / 2) and gap_behind > max(MIN_GAP, veh.v * SAFE_TIME_HEADWAY / 2):
                # regla de prioridad: solo un vehículo de menor prioridad puede ceder espacio a uno de mayor prioridad
                if veh_ahead and veh.priority > getattr(veh_ahead, 'priority', 1):
                    pass  # vehículo delante cede hueco (conceptual)
                if veh in self.lanes[veh.lane]:
                    self.lanes[veh.lane].remove(veh)
                veh.lane = target
                self.lanes[target].append(veh)
                self.lanes[target].sort(key=lambda v: v.x)
                self.lanes[(target - d)].sort(key=lambda v: v.x)
                return True
        return False

# =======================
# Simulación con colas y prioridades
# =======================
def run_simulation():
    highway = Highway(ROAD_LENGTH_M, NUM_LANES)
    t = 0.0
    next_arrival_time = 0.0
    arrival_interval = 60.0 / ARRIVAL_RATE_PER_MIN
    stats_flow = 0
    total_speed = 0.0
    speed_samples = 0
    lines_written = 0

    # colas de entrada por carril
    queues = [[] for _ in range(NUM_LANES)]

    traj_file = open('trajectories.csv', 'w', newline='')
    csvw = csv.writer(traj_file)
    csvw.writerow(['time_s', 'veh_id', 'lane', 'x_m', 'v_m_s'])
    lines_written += 1

    while t < SIM_SECONDS:
        # generar vehículos en la entrada con tipo aleatorio
        if t >= next_arrival_time:
            veh_type = random.choices(["car", "truck", "emergency"], weights=[0.7, 0.25, 0.05])[0]
            veh = Vehicle(lane=random.randint(0, NUM_LANES-1), x=0.0, v=MAX_SPEED*0.6, type_=veh_type)
            veh.entry_time = t
            queues[veh.lane].append(veh)
            next_arrival_time += arrival_interval

        # intentar meter vehículos de la cola en la autopista según prioridad
        for lane in range(NUM_LANES):
            if queues[lane]:
                # ordenar la cola por prioridad (emergency > truck > car)
                queues[lane].sort(key=lambda v: v.priority, reverse=True)
                first_in_queue = queues[lane][0]
                lane_list = highway.lanes[lane]
                if not lane_list or (lane_list[0].x - VEHICLE_LENGTH > MIN_GAP + VEHICLE_LENGTH):
                    highway.add_vehicle(first_in_queue)
                    queues[lane].pop(0)

        # actualizar vehículos
        for lane in range(NUM_LANES):
            for veh in list(highway.lanes[lane]):
                ahead = highway.vehicle_ahead(veh)
                if ahead:
                    desired_gap = veh.v * SAFE_TIME_HEADWAY + MIN_GAP
                    dist_to_ahead = ahead.x - (veh.x + veh.length)
                    if dist_to_ahead < desired_gap or ahead.v < veh.v * 0.8:
                        highway.try_change_lane(veh)

                ahead = highway.vehicle_ahead(veh)
                accel = 1.5
                decel_max = 6.0
                if ahead:
                    gap = ahead.x - (veh.x + veh.length)
                    safe_v = gap / max(1e-6, SAFE_TIME_HEADWAY)
                    target_v = min(MAX_SPEED, safe_v)
                    if veh.v > target_v:
                        dv = min((veh.v - target_v), decel_max * DT)
                        veh.v -= dv
                    else:
                        dv = min((target_v - veh.v), accel * DT)
                        veh.v += dv
                else:
                    dv = min((MAX_SPEED - veh.v), accel * DT)
                    veh.v += dv

                if random.random() < SLOWDOWN_PROB:
                    veh.v = max(0.0, veh.v - 3.0)

                veh.x += veh.v * DT

                if lines_written < MAX_LINES:
                    csvw.writerow([round(t,2), veh.id, veh.lane, round(veh.x,3), round(veh.v,3)])
                    total_speed += veh.v
                    speed_samples += 1
                    lines_written += 1

                if veh.x >= highway.length:
                    veh.finished = True
                    veh.exit_time = t
                    stats_flow += 1
                    highway.remove_vehicle(veh)

        t += DT

    traj_file.close()
    avg_speed_ms = (total_speed / speed_samples) if speed_samples else 0.0
    avg_speed_kmh = ms_to_kmh(avg_speed_ms)
    print("=== Resultados de la simulación con prioridades ===")
    print(f"Longitud: {L_KM} km, carriles: {NUM_LANES}, tiempo sim.: {SIM_SECONDS/3600:.2f} h")
    print(f"Vehículos que han salido correctamente (flujo): {stats_flow}")
    print(f"Velocidad media (muestra): {avg_speed_kmh:.2f} km/h")
    print(f"Trayectorias guardadas en 'trajectories.csv'")

if __name__ == "__main__":
    run_simulation()
