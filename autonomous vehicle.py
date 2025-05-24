import time
import math

# -----------------------------
# 1. Simulated Object Detection
# -----------------------------
class ObjectDetector:
    def detect(self, image):
        print("[Object Detection] Simulated: car, pedestrian, stop sign")
        return ['car', 'pedestrian', 'stop sign']

# -----------------------------
# 2. A* Path Planning
# -----------------------------
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path

def astar(grid, start, goal):
    open_set = [start]
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:
        current = min(open_set, key=lambda x: f_score.get(x, float('inf')))
        if current == goal:
            return reconstruct_path(came_from, current)
        open_set.remove(current)

        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
            neighbor = (current[0]+dx, current[1]+dy)
            if (0 <= neighbor[0] < len(grid)) and (0 <= neighbor[1] < len(grid[0])) and grid[neighbor[0]][neighbor[1]] == 0:
                tentative_g = g_score[current] + 1
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                    if neighbor not in open_set:
                        open_set.append(neighbor)

# -----------------------------
# 3. Sensor Fusion (Averaging)
# -----------------------------
class SensorFusion:
    def __init__(self):
        self.readings = []

    def update(self, reading):
        self.readings.append(reading)
        if len(self.readings) > 5:
            self.readings.pop(0)

    def get_average_position(self):
        return sum(self.readings) / len(self.readings) if self.readings else 0

# -----------------------------
# 4. PID Controller
# -----------------------------
class PID:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, setpoint, measured):
        error = setpoint - measured
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error
        return self.kp*error + self.ki*self.integral + self.kd*derivative

# -----------------------------
# 5. Failsafe
# -----------------------------
def emergency_stop(condition):
    if condition:
        print("[FAILSAFE] Stop triggered!")
        return True
    return False

# -----------------------------
# 6. Logger
# -----------------------------
def log(task, latency, success):
    ts = time.strftime("%Y-%m-%d %H:%M:%S")
    msg = f"{ts} - {task}: {latency:.2f}s, Success: {success*100:.1f}%"
    print(msg)

# -----------------------------
# 7. Main Simulation
# -----------------------------
if __name__ == "__main__":
    print("=== Simulated Autonomous Vehicle System ===")

    # 1. Object Detection
    detector = ObjectDetector()
    objects = detector.detect("image")
    
    # 2. Path Planning
    grid =  [0,0,0,0],
    [1,1,0,1],
