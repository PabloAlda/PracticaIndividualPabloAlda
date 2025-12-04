import time
import random
import os

# ==========================================================
#   CONFIGURACIÓN GENERAL  
# ==========================================================

SIMULATION_SPEED = 3      # La simulación corre 3 veces más rápido
ROAD_LENGTH = 60          # km
TIME_LIMIT = 2            # horas
STEP_TIME = 1 / SIMULATION_SPEED  # segundos por paso visual

# Límites de velocidad por carril (km/h)
LANE_SPEED = {
    0: 90,    # Carril derecho
    1: 100,   # Carril central
    2: 120    # Carril izquierdo
}

# ==========================================================
#   VEHÍCULO
# ==========================================================

class Vehicle:
    def __init__(self, vtype, lane):
        self.type = vtype
        self.position = 0.0
        self.lane = lane

        # Velocidad base según tipo
        if vtype == "coche":
            self.speed = random.uniform(80, 120)
        elif vtype == "camion":
            self.speed = random.uniform(70, 90)
        elif vtype == "ambulancia":
            self.speed = random.uniform(110, 140)

        # Prioridades para adelantamiento
        # 3 = máxima prioridad
        self.priority = {"camion": 1, "coche": 2, "ambulancia": 3}[vtype]

    def update_speed_by_lane(self):
        """Ajusta la velocidad máxima según el carril."""
        max_lane_speed = LANE_SPEED[self.lane]
        if self.speed > max_lane_speed:
            self.speed = max_lane_speed

    def move(self, dt):
        self.update_speed_by_lane()
        self.position += self.speed * (dt/3600.0)  # km/h → km/s


# ==========================================================
#   SIMULADOR A2
# ==========================================================

class A2Simulator:
    def __init__(self):
        self.vehicles = []
        self.finished = {"coche": 0, "camion": 0, "ambulancia": 0}
        self.speeds_finished = {"coche": [], "camion": [], "ambulancia": []}

    def spawn_vehicles(self):
        """Introduce vehículos de forma aleatoria."""
        if random.random() < 0.3:
            vtype = random.choice(["coche", "camion", "ambulancia"])
            lane = random.randint(0, 2)
            self.vehicles.append(Vehicle(vtype, lane))

    def update_positions(self, dt):
        """Actualiza posiciones y gestiona adelantamientos."""
        for v in self.vehicles:
            v.move(dt)

        # Gestión básica de adelantamientos por prioridad
        self.vehicles.sort(key=lambda x: (x.position, x.priority), reverse=True)

    def remove_finished(self):
        """Elimina vehículos que alcanzaron los 60 km."""
        remaining = []
        for v in self.vehicles:
            if v.position >= ROAD_LENGTH:
                self.finished[v.type] += 1
                self.speeds_finished[v.type].append(v.speed)
            else:
                remaining.append(v)
        self.vehicles = remaining

    def display(self):
        """Representación visual simple de la carretera."""
        os.system("cls" if os.name == "nt" else "clear")
        print("=== SIMULACIÓN DE LA A-2 (Guadalajara → Madrid) ===\n")
        print("Carriles: 0 = derecho | 1 = central | 2 = izquierdo")
        print("Vel. máx: 90 | 100 | 120 km/h respectivamente\n")
        print("Simulación x3 más rápida\n")

        road = [["." for _ in range(60)] for _ in range(3)]

        for v in self.vehicles:
            pos = min(int(v.position), 59)
            symbol = {"coche": "C", "camion": "T", "ambulancia": "A"}[v.type]
            road[v.lane][pos] = symbol

        for lane in reversed(range(3)):
            print(f"Carril {lane}: " + "".join(road[lane]))

        print("\nVehículos en carretera:", len(self.vehicles))
        print("Finalizados:", self.finished)

    def run(self):
        print("Iniciando simulación visual A-2...\n")
        total_seconds = TIME_LIMIT * 3600
        elapsed = 0

        while elapsed < total_seconds:
            self.spawn_vehicles()
            self.update_positions(dt=1)
            self.remove_finished()
            self.display()

            time.sleep(STEP_TIME)
            elapsed += 1

        print("\n=== RESULTADOS FINALES ===")
        for vtype in ["camion", "coche", "ambulancia"]:
            count = self.finished[vtype]
            speeds = self.speeds_finished[vtype]
            avg_speed = sum(speeds)/len(speeds) if speeds else 0
            print(f"{vtype.upper()}: {count} vehículos terminados | Velocidad media: {avg_speed:.1f} km/h")


# ==========================================================
#   EJECUCIÓN
# ==========================================================
sim = A2Simulator()
sim.run()
