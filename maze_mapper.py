import threading
import pygame
import time
import paho.mqtt.client as mqtt
import json
import csv
import os

"""
MAZE MASTER CONTROL SYSTEM
(Full UI: Map Builder + PID Buttons + Lidar Calib Buttons)

Updates:
- Added PID TUNING BUTTONS: เพิ่มปุ่มกดบนหน้าจอสำหรับปรับ Kp, Ki, Kd (ไม่ต้องจำคีย์ลัด)
- Layout Update: จัดหน้าจอขวาให้วางปุ่มได้พอดี
"""

# --- 1. MQTT Settings ---
MQTT_BROKER_IP = "broker.hivemq.com"
MQTT_PORT = 1883

# Topics
TOPIC_LIDAR_DATA = "robot/lidar_data1"       
TOPIC_ANGLE_DATA = "robot/anglem51"          
TOPIC_ARUCO_DATA = "robot/tracking_data"    
TOPIC_ARUCO_STATE = "robot/state"           
TOPIC_ROBOT_COMMAND = "robot/mecanum_command1" 
TOPIC_PID_TUNE = "robot/pid_tune"

# --- 2. Map Settings ---
MAZE_WIDTH = 8
MAZE_HEIGHT = 8
CELL_SIZE = 70 

# UI Colors
C_BG = (30, 30, 30)
C_GRID_BG = (255, 255, 255)
C_GRID_LINE = (220, 220, 220)
C_GRID_POINT = (200, 200, 200)
C_ROBOT = (0, 100, 255)
C_WALL_CONFIRMED = (255, 0, 0)      
C_WALL_DETECTING = (255, 200, 200)  
C_WALL_SAVED = (0, 0, 0)            
C_TEXT = (255, 255, 255)
C_HIGHLIGHT = (0, 255, 0) 
C_BTN_BG = (60, 60, 60)
C_BTN_HOVER = (100, 100, 100)
C_BTN_TEXT = (255, 255, 255)
C_COORD_TEXT = (180, 180, 180) 
C_STATE_NORMAL = (200, 200, 200)
C_STATE_CHECK = (255, 200, 0) 
C_MANUAL_MODE = (0, 255, 255)
C_OFFSET_TEXT = (100, 255, 100)

# --- 3. Global Variables ---
data_lock = threading.Lock()
running = True

# Sensor Data
current_lidar = {"F": 2000.0, "L": 2000.0, "R": 2000.0, "B": 2000.0}
current_yaw = 0.0
aruco_state = "WAITING..." 

LIDAR_SMOOTH_ALPHA = 0.4

# Lidar Offsets
lidar_offset_l = 0.0
lidar_offset_r = 0.0

# Robot Position
robot_x = 0
robot_y = 0
robot_dir = 2 

# Map Data
map_h_walls = [[0 for _ in range(MAZE_WIDTH)] for _ in range(MAZE_HEIGHT + 1)]
map_v_walls = [[0 for _ in range(MAZE_WIDTH + 1)] for _ in range(MAZE_HEIGHT)]

# Logic Variables
WALL_THRESHOLD = 900      
WALL_TIME_TH = 1.5        
wall_start_times = {"F": 0, "L": 0, "R": 0, "B": 0} 
walls_confirmed = {"F": False, "L": False, "R": False, "B": False} 
last_plotted_pos = (-1, -1) 

# State Machine
controller_state = "IDLE" 
proposed_action = "NONE" 
target_heading = 0.0      
logic_reason_idx = -1 

# Manual Control Vars
manual_vx = 0.0
manual_vy = 0.0
manual_wz = 0.0
manual_target_angle = -1.0

# Heading PID Tuning Vars
h_pid_kp = 0.025
h_pid_ki = 0.000
h_pid_kd = 0.030

# Center PID (Python Side)
class PIDController:
    def __init__(self, kp, ki, kd, max_out=1.0):
        self.kp = kp; self.ki = ki; self.kd = kd; self.max_out = max_out
        self.prev_error = 0; self.integral = 0; self.last_time = time.time()
        
    def compute(self, error):
        now = time.time(); dt = now - self.last_time if now - self.last_time > 0 else 0.001
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.prev_error = error; self.last_time = now
        return max(min(output, self.max_out), -self.max_out)

center_pid = PIDController(kp=0.0015, ki=0.0, kd=0.0005, max_out=0.5) 

# --- 4. MQTT Functions ---

def on_mqtt_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        print(f"Connected to MQTT ({MQTT_BROKER_IP})")
        client.subscribe(TOPIC_LIDAR_DATA)
        client.subscribe(TOPIC_ANGLE_DATA)
        client.subscribe(TOPIC_ARUCO_DATA)
        client.subscribe(TOPIC_ARUCO_STATE)
    else:
        print(f"Failed to connect rc={rc}")

def on_message(client, userdata, msg):
    global current_lidar, current_yaw, robot_x, robot_y, robot_dir, aruco_state
    
    try:
        if msg.topic == TOPIC_LIDAR_DATA:
            data = json.loads(msg.payload.decode("utf-8"))
            
            with data_lock:
                raw_f = float(data.get("F", 2000))
                raw_l = float(data.get("L", 2000)) + lidar_offset_l
                raw_r = float(data.get("R", 2000)) + lidar_offset_r
                raw_b = float(data.get("B", 2000))
            
                alpha = LIDAR_SMOOTH_ALPHA
                current_lidar["F"] = (current_lidar["F"] * (1.0 - alpha)) + (raw_f * alpha)
                current_lidar["L"] = (current_lidar["L"] * (1.0 - alpha)) + (raw_l * alpha)
                current_lidar["R"] = (current_lidar["R"] * (1.0 - alpha)) + (raw_r * alpha)
                current_lidar["B"] = (current_lidar["B"] * (1.0 - alpha)) + (raw_b * alpha)
                
        elif msg.topic == TOPIC_ANGLE_DATA:
            val = float(msg.payload.decode("utf-8"))
            with data_lock:
                current_yaw = val
                yaw = current_yaw % 360
                if yaw >= 315 or yaw < 45: robot_dir = 2 
                elif 45 <= yaw < 135: robot_dir = 1      
                elif 135 <= yaw < 225: robot_dir = 0     
                elif 225 <= yaw < 315: robot_dir = 3     
                
        elif msg.topic == TOPIC_ARUCO_DATA:
            data = json.loads(msg.payload.decode("utf-8"))
            with data_lock:
                robot_x = data.get("grid_x", robot_x)
                robot_y = data.get("grid_y", robot_y)
        
        elif msg.topic == TOPIC_ARUCO_STATE:
            aruco_state = msg.payload.decode("utf-8")

    except Exception as e:
        pass

def send_pid_update(client):
    payload = { "kp": h_pid_kp, "ki": h_pid_ki, "kd": h_pid_kd, "db": 2.0 }
    client.publish(TOPIC_PID_TUNE, json.dumps(payload))
    print(f"PID Sent: P{h_pid_kp:.3f} I{h_pid_ki:.3f} D{h_pid_kd:.3f}")

def mqtt_client_loop(client):
    client.on_connect = on_mqtt_connect
    client.on_message = on_message
    client.connect(MQTT_BROKER_IP, MQTT_PORT, 60)
    client.loop_forever()

# --- 5. Logic & Functions ---

def update_wall_timers():
    global wall_start_times, walls_confirmed
    now = time.time()
    directions = ["F", "L", "R"] 
    for d in directions:
        dist = current_lidar[d]
        if dist < WALL_THRESHOLD:
            if wall_start_times[d] == 0: wall_start_times[d] = now
            if (now - wall_start_times[d]) >= WALL_TIME_TH: walls_confirmed[d] = True
            else: walls_confirmed[d] = False
        else:
            wall_start_times[d] = 0
            walls_confirmed[d] = False
    walls_confirmed["B"] = False; wall_start_times["B"] = 0

def plot_current_walls():
    global map_h_walls, map_v_walls, last_plotted_pos
    rx, ry, rdir = robot_x, robot_y, robot_dir
    dirs = ["Top", "Right", "Bottom", "Left"] 
    f_idx, r_idx = rdir, (rdir + 1) % 4
    b_idx, l_idx = (rdir + 2) % 4, (rdir + 3) % 4
    
    walls = {"Top": False, "Right": False, "Bottom": False, "Left": False}
    if current_lidar["F"] < WALL_THRESHOLD: walls[dirs[f_idx]] = True
    if current_lidar["R"] < WALL_THRESHOLD: walls[dirs[r_idx]] = True
    if current_lidar["L"] < WALL_THRESHOLD: walls[dirs[l_idx]] = True
    
    if walls["Top"]: map_h_walls[ry][rx] = 1
    if walls["Bottom"]: map_h_walls[ry + 1][rx] = 1
    if walls["Left"]: map_v_walls[ry][rx] = 1
    if walls["Right"]: map_v_walls[ry][rx + 1] = 1
    
    last_plotted_pos = (rx, ry)
    print(f"Auto-Plotted walls at ({rx},{ry})")

def clear_current_cell_walls():
    global map_h_walls, map_v_walls
    rx, ry = robot_x, robot_y
    map_h_walls[ry][rx] = 0; map_h_walls[ry + 1][rx] = 0
    map_v_walls[ry][rx] = 0; map_v_walls[ry][rx + 1] = 0
    print(f"Cleared walls at ({rx},{ry})")

def save_map_to_csv():
    try:
        with open("map_horizontal.csv", "w", newline="") as f:
            csv.writer(f).writerows(map_h_walls)
        with open("map_vertical.csv", "w", newline="") as f:
            csv.writer(f).writerows(map_v_walls)
        print("Map Saved.")
    except Exception as e: print(f"Error: {e}")

def snap_heading(yaw):
    return round(yaw / 90) * 90 % 360

def decide_next_action():
    if current_lidar["F"] < 250: return "STOP", current_yaw, 0
    
    is_wall_f = walls_confirmed["F"]
    is_wall_l = walls_confirmed["L"]
    is_wall_r = walls_confirmed["R"]
    current_snap = snap_heading(current_yaw) 
    
    if current_lidar["F"] < 150 and current_lidar["L"] < 150 and current_lidar["R"] < 150:
        return "BACKWARD", current_snap, 0

    if not is_wall_r: return "ROTATE_RIGHT", (current_snap - 90) % 360, 1
    elif not is_wall_f: return "FORWARD", current_snap, 2
    elif not is_wall_l: return "ROTATE_LEFT", (current_snap + 90) % 360, 3
    else: return "U-TURN", (current_snap + 180) % 360, 4 

def send_auto_command(client):
    global proposed_action, target_heading, controller_state
    if controller_state != "EXECUTING": return

    vx, vy = 0, 0
    center_correction = 0.0
    if proposed_action in ["FORWARD", "BACKWARD"]:
        l_dist = current_lidar["L"]
        r_dist = current_lidar["R"]
        if l_dist < 1000 and r_dist < 1000:
             error = l_dist - r_dist
             center_correction = -1 * center_pid.compute(error)

    if proposed_action == "FORWARD": 
        vy = 0.6; vx = center_correction 
    elif proposed_action == "BACKWARD": 
        vy = -0.6; vx = center_correction
    elif "ROTATE" in proposed_action or "U-TURN" in proposed_action: 
        vy = 0; vx = 0 
        
    cmd_data = {"vx": vx, "vy": vy, "target_yaw": target_heading}
    client.publish(TOPIC_ROBOT_COMMAND, json.dumps(cmd_data))

def send_manual_command(client):
    cmd_data = {"vx": manual_vx, "vy": manual_vy, "wz": manual_wz}
    if manual_target_angle >= 0: cmd_data["target_yaw"] = manual_target_angle 
    client.publish(TOPIC_ROBOT_COMMAND, json.dumps(cmd_data))

# --- 6. UI Class ---
class Button:
    def __init__(self, x, y, w, h, text, callback, color=C_BTN_BG, text_size=16):
        self.rect = pygame.Rect(x, y, w, h); self.text = text; self.callback = callback; self.color = color; self.is_hovered = False; self.clicked_timer = 0
        self.font = pygame.font.SysFont("Arial", text_size, bold=True)
    def draw(self, screen):
        color = C_BTN_HOVER if self.is_hovered else self.color
        if self.clicked_timer > 0: color = (0, 200, 0); self.clicked_timer -= 1
        pygame.draw.rect(screen, color, self.rect); pygame.draw.rect(screen, (200, 200, 200), self.rect, 2)
        txt = self.font.render(self.text, True, C_BTN_TEXT); screen.blit(txt, txt.get_rect(center=self.rect.center))
    def handle_event(self, event):
        if event.type == pygame.MOUSEMOTION: self.is_hovered = self.rect.collidepoint(event.pos)
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if self.is_hovered and event.button == 1: self.clicked_timer = 10; self.callback()

# --- 7. Main UI ---
def main_ui():
    global running, controller_state, proposed_action, target_heading, logic_reason_idx, last_plotted_pos
    global manual_vx, manual_vy, manual_wz, manual_target_angle
    global h_pid_kp, h_pid_ki, h_pid_kd, lidar_offset_l, lidar_offset_r
    
    pygame.init()
    SCREEN_W, SCREEN_H = MAZE_WIDTH * CELL_SIZE + 400, MAZE_HEIGHT * CELL_SIZE + 60
    screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
    pygame.display.set_caption("Maze Master (Buttons Added)")
    clock = pygame.time.Clock()
    
    font_head = pygame.font.SysFont("Arial", 20, bold=True)
    font_text = pygame.font.SysFont("Arial", 16)
    font_logic = pygame.font.SysFont("Consolas", 14)
    font_coord = pygame.font.SysFont("Arial", 12)
    font_cmd = pygame.font.SysFont("Arial", 24, bold=True)

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    threading.Thread(target=mqtt_client_loop, args=(client,), daemon=True).start()
    
    # --- Button Definitions ---
    
    # 1. PID Tuning Buttons
    pid_y = 250
    btn_kp_dn = Button(SCREEN_W - 200, pid_y, 40, 25, "-", lambda: globals().update(h_pid_kp=max(0, h_pid_kp-0.005)), text_size=14)
    btn_kp_up = Button(SCREEN_W - 60,  pid_y, 40, 25, "+", lambda: globals().update(h_pid_kp=h_pid_kp+0.005), text_size=14)
    
    btn_ki_dn = Button(SCREEN_W - 200, pid_y+30, 40, 25, "-", lambda: globals().update(h_pid_ki=max(0, h_pid_ki-0.001)), text_size=14)
    btn_ki_up = Button(SCREEN_W - 60,  pid_y+30, 40, 25, "+", lambda: globals().update(h_pid_ki=h_pid_ki+0.001), text_size=14)
    
    btn_kd_dn = Button(SCREEN_W - 200, pid_y+60, 40, 25, "-", lambda: globals().update(h_pid_kd=max(0, h_pid_kd-0.005)), text_size=14)
    btn_kd_up = Button(SCREEN_W - 60,  pid_y+60, 40, 25, "+", lambda: globals().update(h_pid_kd=h_pid_kd+0.005), text_size=14)

    # 2. Lidar Calib Buttons
    cal_y = 185
    btn_l_dn = Button(SCREEN_W - 300, cal_y, 40, 25, "L-", lambda: globals().update(lidar_offset_l=lidar_offset_l-10), text_size=14)
    btn_l_up = Button(SCREEN_W - 250, cal_y, 40, 25, "L+", lambda: globals().update(lidar_offset_l=lidar_offset_l+10), text_size=14)
    btn_r_dn = Button(SCREEN_W - 150, cal_y, 40, 25, "R-", lambda: globals().update(lidar_offset_r=lidar_offset_r-10), text_size=14)
    btn_r_up = Button(SCREEN_W - 100, cal_y, 40, 25, "R+", lambda: globals().update(lidar_offset_r=lidar_offset_r+10), text_size=14)

    # 3. Map Buttons
    map_y = 540
    btn_plot = Button(MAZE_WIDTH * CELL_SIZE + 20, map_y, 100, 40, "PLOT (P)", plot_current_walls)
    btn_clear = Button(MAZE_WIDTH * CELL_SIZE + 140, map_y, 100, 40, "CLEAR (C)", clear_current_cell_walls, color=(150, 50, 50))
    btn_save = Button(MAZE_WIDTH * CELL_SIZE + 260, map_y, 100, 40, "SAVE (S)", save_map_to_csv, color=(0, 100, 0))
    
    all_buttons = [
        btn_kp_dn, btn_kp_up, btn_ki_dn, btn_ki_up, btn_kd_dn, btn_kd_up,
        btn_l_dn, btn_l_up, btn_r_dn, btn_r_up,
        btn_plot, btn_clear, btn_save
    ]

    start_exec_time = 0
    last_manual_send = 0
    last_pid_val = (h_pid_kp, h_pid_ki, h_pid_kd)

    logic_steps = [
        "0. STOP (< 250mm)",
        f"1. RIGHT OPEN (> {WALL_THRESHOLD}) -> TURN 90",
        f"2. FRONT OPEN (> {WALL_THRESHOLD}) -> FWD",
        f"3. LEFT OPEN (> {WALL_THRESHOLD}) -> TURN 270",
        "4. BLOCKED -> U-TURN"
    ]

    def stop_robot():
        client.publish(TOPIC_ROBOT_COMMAND, json.dumps({"vx": 0, "vy": 0, "wz": 0}))

    while running:
        update_wall_timers() 
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False
            
            # Click Events
            for btn in all_buttons: btn.handle_event(event)
            
            if event.type == pygame.KEYDOWN:
                # General
                if event.key == pygame.K_z: controller_state = "THINKING" 
                elif event.key == pygame.K_RETURN and controller_state == "WAITING_FOR_CONFIRM":
                    controller_state = "EXECUTING"; start_exec_time = time.time()
                elif event.key == pygame.K_SPACE: 
                    stop_robot(); controller_state = "IDLE"; manual_vx=0; manual_vy=0; manual_wz=0; manual_target_angle=-1.0
                elif event.key == pygame.K_p: plot_current_walls()
                elif event.key == pygame.K_c: clear_current_cell_walls()
                elif event.key == pygame.K_s: save_map_to_csv()
                
                # Manual
                if event.key == pygame.K_UP: manual_vy = 0.6; controller_state = "MANUAL"
                elif event.key == pygame.K_DOWN: manual_vy = -0.6; controller_state = "MANUAL"
                elif event.key == pygame.K_LEFT: manual_vx = -0.6; controller_state = "MANUAL"
                elif event.key == pygame.K_RIGHT: manual_vx = 0.6; controller_state = "MANUAL"
                elif event.key == pygame.K_a: manual_wz = 0.6; controller_state = "MANUAL"
                elif event.key == pygame.K_d: manual_wz = -0.6; controller_state = "MANUAL"
                
                elif event.key == pygame.K_1: manual_target_angle = 0.0; controller_state = "MANUAL"
                elif event.key == pygame.K_2: manual_target_angle = 90.0; controller_state = "MANUAL"
                elif event.key == pygame.K_3: manual_target_angle = 180.0; controller_state = "MANUAL"
                elif event.key == pygame.K_4: manual_target_angle = 270.0; controller_state = "MANUAL"
                elif event.key == pygame.K_0: manual_target_angle = -1.0; controller_state = "MANUAL"

            if event.type == pygame.KEYUP:
                if event.key in [pygame.K_UP, pygame.K_DOWN]: manual_vy = 0
                if event.key in [pygame.K_LEFT, pygame.K_RIGHT]: manual_vx = 0 
                if event.key in [pygame.K_a, pygame.K_d]: manual_wz = 0        

        # Send PID only if changed
        current_pid_val = (h_pid_kp, h_pid_ki, h_pid_kd)
        if current_pid_val != last_pid_val:
            send_pid_update(client)
            last_pid_val = current_pid_val

        if controller_state == "THINKING":
            proposed_action, target_heading, logic_reason_idx = decide_next_action()
            controller_state = "WAITING_FOR_CONFIRM"
        elif controller_state == "EXECUTING":
            send_auto_command(client)
            if time.time() - start_exec_time > 1.5: 
                stop_robot(); controller_state = "THINKING"; time.sleep(0.2) 
            if aruco_state in ["STOP", "CHECK"]:
                stop_robot(); controller_state = "THINKING"
        elif controller_state == "MANUAL":
            if time.time() - last_manual_send > 0.1:
                last_manual_send = time.time()
                send_manual_command(client)

        if aruco_state in ["STOP", "CHECK"]:
             current_pos = (robot_x, robot_y)
             if current_pos != last_plotted_pos: plot_current_walls()

        # Draw
        screen.fill(C_BG)
        pygame.draw.rect(screen, C_GRID_BG, (0, 0, MAZE_WIDTH * CELL_SIZE, SCREEN_H - 60))
        for y in range(MAZE_HEIGHT):
            for x in range(MAZE_WIDTH):
                r = (x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE)
                pygame.draw.rect(screen, C_GRID_LINE, r, 1)
                pygame.draw.circle(screen, C_GRID_POINT, (x * CELL_SIZE + 35, y * CELL_SIZE + 35), 3)
                screen.blit(font_coord.render(f"{x},{y}", True, C_COORD_TEXT), (x * CELL_SIZE + 3, y * CELL_SIZE + 3))
        
        for y in range(len(map_h_walls)):
            for x in range(len(map_h_walls[0])):
                if map_h_walls[y][x] == 1: pygame.draw.line(screen, C_WALL_SAVED, (x*CELL_SIZE, y*CELL_SIZE), ((x+1)*CELL_SIZE, y*CELL_SIZE), 5)
        for y in range(len(map_v_walls)):
            for x in range(len(map_v_walls[0])):
                if map_v_walls[y][x] == 1: pygame.draw.line(screen, C_WALL_SAVED, (x*CELL_SIZE, y*CELL_SIZE), (x*CELL_SIZE, (y+1)*CELL_SIZE), 5)

        cx, cy, s = robot_x * CELL_SIZE + 35, robot_y * CELL_SIZE + 35, 23
        pts = []
        if robot_dir == 0: pts = [(cx, cy-s), (cx-s, cy+s), (cx+s, cy+s)]
        elif robot_dir == 1: pts = [(cx+s, cy), (cx-s, cy-s), (cx-s, cy+s)]
        elif robot_dir == 2: pts = [(cx, cy+s), (cx-s, cy-s), (cx+s, cy-s)]
        elif robot_dir == 3: pts = [(cx-s, cy), (cx+s, cy-s), (cx+s, cy+s)]
        pygame.draw.polygon(screen, C_ROBOT, pts)
        
        dirs = ["Top", "Right", "Bottom", "Left"]
        f_idx, r_idx = robot_dir, (robot_dir + 1) % 4
        b_idx, l_idx = (robot_dir + 2) % 4, (robot_dir + 3) % 4
        def get_wall_color(key): return C_WALL_CONFIRMED if walls_confirmed[key] else (C_WALL_DETECTING if wall_start_times[key] > 0 else None)
        px, py = robot_x * CELL_SIZE, robot_y * CELL_SIZE
        c = get_wall_color("F"); 
        if c and f_idx==0: pygame.draw.line(screen, c, (px, py), (px+70, py), 3) 
        elif c and f_idx==1: pygame.draw.line(screen, c, (px+70, py), (px+70, py+70), 3) 
        elif c and f_idx==2: pygame.draw.line(screen, c, (px, py+70), (px+70, py+70), 3) 
        elif c and f_idx==3: pygame.draw.line(screen, c, (px, py), (px, py+70), 3) 
        c = get_wall_color("R"); 
        if c and r_idx==0: pygame.draw.line(screen, c, (px, py), (px+70, py), 3)
        elif c and r_idx==1: pygame.draw.line(screen, c, (px+70, py), (px+70, py+70), 3)
        elif c and r_idx==2: pygame.draw.line(screen, c, (px, py+70), (px+70, py+70), 3)
        elif c and r_idx==3: pygame.draw.line(screen, c, (px, py), (px, py+70), 3)
        c = get_wall_color("L"); 
        if c and l_idx==0: pygame.draw.line(screen, c, (px, py), (px+70, py), 3)
        elif c and l_idx==1: pygame.draw.line(screen, c, (px+70, py), (px+70, py+70), 3)
        elif c and l_idx==2: pygame.draw.line(screen, c, (px, py+70), (px+70, py+70), 3)
        elif c and l_idx==3: pygame.draw.line(screen, c, (px, py), (px, py+70), 3)

        px = MAZE_WIDTH * CELL_SIZE + 10
        screen.blit(font_head.render("SENSORS", True, C_TEXT), (px, 20))
        pygame.draw.rect(screen, (50, 50, 50), (px, 50, 380, 100))
        screen.blit(font_text.render(f"F:{current_lidar['F']:.0f} R:{current_lidar['R']:.0f} L:{current_lidar['L']:.0f}", True, C_TEXT), (px+10, 60))
        screen.blit(font_text.render(f"Yaw: {current_yaw:.1f} (Dir: {robot_dir})", True, (255, 255, 0)), (px+10, 85))
        screen.blit(font_text.render(f"ArUco: {aruco_state}", True, (0,255,255) if aruco_state in ["CHECK","STOP"] else C_TEXT), (px+10, 110))

        screen.blit(font_head.render("LIDAR CALIBRATION", True, C_TEXT), (px, 160))
        for btn in [btn_l_dn, btn_l_up, btn_r_dn, btn_r_up]: btn.draw(screen)
        off_txt = font_coord.render(f"Offset L: {lidar_offset_l:+.0f} | R: {lidar_offset_r:+.0f}", True, C_OFFSET_TEXT)
        screen.blit(off_txt, (px+130, 195))

        screen.blit(font_head.render("HEADING PID TUNING", True, (255, 200, 0)), (px, 225))
        pygame.draw.rect(screen, (50, 50, 50), (px, 250, 380, 100))
        
        # Draw PID Buttons and Values
        for btn in [btn_kp_dn, btn_kp_up, btn_ki_dn, btn_ki_up, btn_kd_dn, btn_kd_up]: btn.draw(screen)
        
        screen.blit(font_text.render(f"Kp: {h_pid_kp:.3f}", True, C_TEXT), (px+130, 255))
        screen.blit(font_text.render(f"Ki: {h_pid_ki:.3f}", True, C_TEXT), (px+130, 285))
        screen.blit(font_text.render(f"Kd: {h_pid_kd:.3f}", True, C_TEXT), (px+130, 315))

        screen.blit(font_head.render("AUTO LOGIC", True, C_TEXT), (px, 370))
        y = 400
        for i, step in enumerate(logic_steps):
            c = C_HIGHLIGHT if (controller_state in ["WAITING_FOR_CONFIRM", "EXECUTING"] and i == logic_reason_idx) else C_TEXT
            screen.blit(font_logic.render(step, True, c), (px, y)); y += 25
        
        for btn in [btn_plot, btn_clear, btn_save]: btn.draw(screen)
        
        pygame.draw.rect(screen, (10, 10, 10), (0, SCREEN_H - 60, SCREEN_W, 60))
        msg = "IDLE"
        color_msg = (200, 200, 200)
        if controller_state == "MANUAL": 
            msg = f"MANUAL: V({manual_vx},{manual_vy}) Lock: {manual_target_angle}"
            color_msg = C_MANUAL_MODE
        elif controller_state == "WAITING_FOR_CONFIRM": 
            msg = f"AUTO: {proposed_action} > {target_heading:.0f} (ENTER)"
            color_msg = (255, 255, 0)
        elif controller_state == "EXECUTING": 
            msg = f"AUTO EXEC: {proposed_action}..."
            color_msg = (0, 255, 0)
        elif controller_state == "THINKING": msg = "THINKING..."
            
        screen.blit(font_cmd.render(msg, True, color_msg), (20, SCREEN_H - 50))
        pygame.display.flip()
        clock.tick(30)

    pygame.quit()
    if client.is_connected(): stop_robot(); client.loop_stop()

if __name__ == "__main__":
    main_ui()
