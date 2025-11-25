import threading
import csv
import pygame
import time
import paho.mqtt.client as mqtt
from collections import deque

# --- 1. การตั้งค่า ---

# --- ตั้งค่า MQTT ---
MQTT_BROKER_IP = "broker.hivemq.com"
MQTT_PORT = 1883
TOPIC_ROBOT_COMMAND = "robot/command"

# --- ตั้งค่าไฟล์ CSV ---
FILE_GRID = 'maze_grid.csv'
FILE_H_WALLS = 'horizontal_walls.csv'
FILE_V_WALLS = 'vertical_walls.csv'

# --- ตั้งค่าแผนที่ ---
MAZE_WIDTH = 8
MAZE_HEIGHT = 8
CELL_SIZE = 80
UI_PANEL_WIDTH = 250 # พื้นที่ด้านขวาสำหรับแสดง Step

# --- 2. ตัวแปร Global ---
data_lock = threading.Lock() # (สำคัญ) ป้องกัน Error Thread
running = True 

maze_grid = []
horizontal_walls = []
vertical_walls = []

start_point = None
start_dir = 2       # 0:N, 1:E, 2:S, 3:W
end_point = None
solved_path = []

# --- ตัวแปรสำหรับ Step Execution ---
command_list = []   # ["FORWARD", "LEFT", ...]
current_step_index = 0 # ตอนนี้อยู่ที่ Step ไหน
is_step_mode = False   # กำลังอยู่ในโหมดรันหรือไม่
execution_status = "IDLE" # IDLE, WAITING, FINISHED

# --- 3. ฟังก์ชัน Helper: โหลด CSV ---
def load_csv(filename):
    data = []
    try:
        with open(filename, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                data.append([int(cell) for cell in row])
        print(f"โหลด '{filename}' สำเร็จ")
        return data
    except Exception as e:
        print(f"!!! Error loading '{filename}': {e}")
        return [[0]*MAZE_WIDTH for _ in range(MAZE_HEIGHT)] # Return empty if fail

# --- 4. Solver Logic (BFS) ---
def is_move_valid(x1, y1, x2, y2, h_walls, v_walls):
    if not (0 <= x2 < MAZE_WIDTH and 0 <= y2 < MAZE_HEIGHT): return False
    try:
        if x1 == x2: # เดินแนวตั้ง
            if y2 < y1: return h_walls[y1][x1] == 2 # ขึ้น
            else: return h_walls[y2][x1] == 2 # ลง
        elif y1 == y2: # เดินแนวนอน
            if x2 < x1: return v_walls[y1][x1] == 2 # ซ้าย
            else: return v_walls[y1][x2] == 2 # ขวา
    except: return False
    return False

def solve_bfs(start, end, h_walls, v_walls):
    queue = deque([(start, [start])])
    visited = set([start])
    while queue:
        (curr, path) = queue.popleft()
        if curr == end: return path
        cx, cy = curr
        # N, E, S, W
        neighbors = [(cx, cy-1), (cx+1, cy), (cx, cy+1), (cx-1, cy)]
        for nx, ny in neighbors:
            if (nx, ny) not in visited and is_move_valid(cx, cy, nx, ny, h_walls, v_walls):
                visited.add((nx, ny))
                queue.append(((nx, ny), path + [(nx, ny)]))
    return []

def generate_commands(path, start_dir):
    if not path or len(path) < 2: return []
    commands = []
    curr_d = start_dir
    for i in range(len(path) - 1):
        x1, y1 = path[i]
        x2, y2 = path[i+1]
        
        # หา Target Dir
        if y2 < y1: target_d = 0 # N
        elif x2 > x1: target_d = 1 # E
        elif y2 > y1: target_d = 2 # S
        else: target_d = 3 # W
        
        diff = target_d - curr_d
        # ปรับค่า Diff (-1=Left, 1=Right)
        if diff == 3: diff = -1
        elif diff == -3: diff = 1
        
        if diff == 0: pass
        elif diff == 1 or diff == -3: commands.append("RIGHT")
        elif diff == -1 or diff == 3: commands.append("LEFT")
        elif abs(diff) == 2: 
            commands.append("RIGHT")
            commands.append("RIGHT")
            
        commands.append("FORWARD")
        curr_d = target_d
    return commands

# --- 5. MQTT Helper ---
def send_command(client, cmd):
    if client:
        print(f">>> MQTT SEND: {cmd}")
        client.publish(TOPIC_ROBOT_COMMAND, cmd)

# --- 6. Main UI ---
def main_ui():
    global running, start_point, end_point, solved_path, command_list
    global start_dir, current_step_index, is_step_mode, execution_status
    
    # Load Data
    global maze_grid, horizontal_walls, vertical_walls
    maze_grid = load_csv(FILE_GRID)
    horizontal_walls = load_csv(FILE_H_WALLS)
    vertical_walls = load_csv(FILE_V_WALLS)

    pygame.init()
    
    # Screen Setup
    SCREEN_W = (MAZE_WIDTH * CELL_SIZE) + UI_PANEL_WIDTH
    SCREEN_H = (MAZE_HEIGHT * CELL_SIZE) + 60 # + Status Bar
    screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
    pygame.display.set_caption("Maze Solver : Step-by-Step Mode")
    
    # Fonts
    font_ui = pygame.font.SysFont("Arial", 18)
    font_cmd = pygame.font.SysFont("Consolas", 16)
    font_big = pygame.font.SysFont("Arial", 24, bold=True)

    # Colors
    C_BG = (240, 240, 240)
    C_PANEL = (50, 50, 60)
    C_TXT_PANEL = (200, 200, 200)
    C_HIGHLIGHT = (0, 255, 0)
    C_PENDING = (100, 100, 100)
    
    # MQTT
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    try:
        client.connect(MQTT_BROKER_IP, MQTT_PORT, 60)
        client.loop_start()
        print("MQTT Connected.")
    except:
        print("!!! MQTT Connect Fail (Offline Mode)")

    clock = pygame.time.Clock()

    while running:
        # --- Event Handling ---
        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False
            
            # 1. Mouse Click (เลือกจุด)
            if event.type == pygame.MOUSEBUTTONDOWN:
                mx, my = event.pos
                if mx < MAZE_WIDTH * CELL_SIZE and my < MAZE_HEIGHT * CELL_SIZE:
                    gx, gy = mx // CELL_SIZE, my // CELL_SIZE
                    
                    if not is_step_mode: # ห้ามแก้จุดตอนรัน
                        if start_point is None:
                            start_point = (gx, gy)
                        elif end_point is None and (gx, gy) != start_point:
                            end_point = (gx, gy)
                            # Auto Solve
                            solved_path = solve_bfs(start_point, end_point, horizontal_walls, vertical_walls)
                            command_list = generate_commands(solved_path, start_dir)
                            execution_status = "READY"
                        else:
                            # Reset
                            start_point = (gx, gy)
                            end_point = None
                            solved_path = []
                            command_list = []
                            is_step_mode = False
                            execution_status = "IDLE"

            # 2. Keyboard
            if event.type == pygame.KEYDOWN:
                
                # [G] Start / Stop
                if event.key == pygame.K_g and command_list:
                    if not is_step_mode:
                        is_step_mode = True
                        current_step_index = 0
                        execution_status = "WAITING" # รอ Spacebar
                        print("--- Enter Step Mode ---")
                    else:
                        # กด G ซ้ำเพื่อหยุด/Reset
                        is_step_mode = False
                        execution_status = "PAUSED"

                # [Spacebar] Execute Next Step
                if event.key == pygame.K_SPACE:
                    if is_step_mode and current_step_index < len(command_list):
                        # ส่งคำสั่งถัดไป
                        cmd = command_list[current_step_index]
                        send_command(client, cmd)
                        current_step_index += 1
                        
                        if current_step_index >= len(command_list):
                            execution_status = "FINISHED"
                            is_step_mode = False
                        else:
                            execution_status = "WAITING"

                # [Arrows] Manual Correction (ทำงานได้ตลอด แม้จะอยู่ใน Step Mode)
                # เอาไว้แก้ทรงหุ่นยนต์ ถ้ามันเดินเบี้ยว
                manual_cmd = None
                if event.key == pygame.K_UP: manual_cmd = "FORWARD"
                elif event.key == pygame.K_DOWN: manual_cmd = "BACKWARD"
                elif event.key == pygame.K_LEFT: manual_cmd = "LEFT"
                elif event.key == pygame.K_RIGHT: manual_cmd = "RIGHT"
                
                if manual_cmd:
                    print(f"[Manual Override] {manual_cmd}")
                    send_command(client, manual_cmd)
                
                # [Start Dir]
                if not is_step_mode and start_point and not end_point:
                    if event.key == pygame.K_UP: start_dir = 0
                    elif event.key == pygame.K_RIGHT: start_dir = 1
                    elif event.key == pygame.K_DOWN: start_dir = 2
                    elif event.key == pygame.K_LEFT: start_dir = 3
                    # Recalculate if needed (not critical here)

        # --- Drawing ---
        screen.fill(C_BG)
        
        # 1. Draw Maze (Zone ซ้าย)
        for y in range(MAZE_HEIGHT):
            for x in range(MAZE_WIDTH):
                rect = (x*CELL_SIZE, y*CELL_SIZE, CELL_SIZE, CELL_SIZE)
                color = (255, 255, 255)
                if maze_grid[y][x] != 2: color = (220, 220, 220)
                pygame.draw.rect(screen, color, rect)
                pygame.draw.rect(screen, (230, 230, 230), rect, 1)
        
        # Walls
        for y in range(MAZE_HEIGHT+1):
            for x in range(MAZE_WIDTH):
                if horizontal_walls[y][x] == 1:
                    pygame.draw.line(screen, (0,0,0), (x*CELL_SIZE, y*CELL_SIZE), ((x+1)*CELL_SIZE, y*CELL_SIZE), 4)
        for y in range(MAZE_HEIGHT):
            for x in range(MAZE_WIDTH+1):
                if vertical_walls[y][x] == 1:
                    pygame.draw.line(screen, (0,0,0), (x*CELL_SIZE, y*CELL_SIZE), (x*CELL_SIZE, (y+1)*CELL_SIZE), 4)

        # Path
        if solved_path:
            pts = [(x*CELL_SIZE+CELL_SIZE//2, y*CELL_SIZE+CELL_SIZE//2) for x,y in solved_path]
            pygame.draw.lines(screen, (0,0,255), False, pts, 3)
            
        # Start/End
        if start_point:
            sx, sy = start_point
            pygame.draw.rect(screen, (0,200,0), (sx*CELL_SIZE, sy*CELL_SIZE, CELL_SIZE, CELL_SIZE), 4)
            # Arrow
            cx, cy = sx*CELL_SIZE+CELL_SIZE//2, sy*CELL_SIZE+CELL_SIZE//2
            pygame.draw.circle(screen, (255,0,0), (cx, cy), 5) # Simple dot for now
            
        if end_point:
            ex, ey = end_point
            pygame.draw.rect(screen, (200,0,0), (ex*CELL_SIZE, ey*CELL_SIZE, CELL_SIZE, CELL_SIZE), 4)

        # 2. Draw UI Panel (Zone ขวา)
        panel_rect = (MAZE_WIDTH*CELL_SIZE, 0, UI_PANEL_WIDTH, SCREEN_H)
        pygame.draw.rect(screen, C_PANEL, panel_rect)
        
        # Title
        title_s = font_big.render("Command List", True, (255,255,255))
        screen.blit(title_s, (MAZE_WIDTH*CELL_SIZE + 20, 20))
        
        # Instructions
        help_y = SCREEN_H - 140
        help_lines = [
            "[G] Start Step Mode",
            "[Space] Execute Next",
            "[Arrows] Manual Fix",
            "[Click] Reset Map"
        ]
        for i, line in enumerate(help_lines):
            t = font_ui.render(line, True, (150, 150, 150))
            screen.blit(t, (MAZE_WIDTH*CELL_SIZE + 20, help_y + i*25))

        # Command List Scroll
        start_list_y = 70
        max_items = 15
        
        # คำนวณหน้าที่จะแสดง (Scroll ตาม Current Step)
        display_start_idx = 0
        if current_step_index > 5:
            display_start_idx = current_step_index - 5
            
        for i in range(display_start_idx, min(len(command_list), display_start_idx + max_items)):
            cmd = command_list[i]
            
            # Determine Color
            color = C_TXT_PANEL
            prefix = "   "
            
            if is_step_mode:
                if i < current_step_index:
                    color = (100, 100, 100) # ทำไปแล้ว (เทาเข้ม)
                    prefix = "[x]"
                elif i == current_step_index:
                    color = C_HIGHLIGHT     # กำลังจะทำ (เขียวสว่าง)
                    prefix = ">>>"
                else:
                    color = C_PENDING       # รอทำ (เทา)
                    prefix = "[ ]"
            
            text_str = f"{i+1}. {prefix} {cmd}"
            txt = font_cmd.render(text_str, True, color)
            screen.blit(txt, (MAZE_WIDTH*CELL_SIZE + 20, start_list_y + (i - display_start_idx)*25))

        # 3. Status Bar (Bottom)
        status_rect = (0, MAZE_HEIGHT*CELL_SIZE, MAZE_WIDTH*CELL_SIZE, 60)
        pygame.draw.rect(screen, (30, 30, 30), status_rect)
        
        status_msg = f"STATUS: {execution_status}"
        if is_step_mode:
            status_msg += f" | Step: {current_step_index}/{len(command_list)}"
            if execution_status == "WAITING":
                status_msg += " (Press Space)"
        
        st_txt = font_ui.render(status_msg, True, (255, 255, 255))
        screen.blit(st_txt, (20, MAZE_HEIGHT*CELL_SIZE + 20))

        pygame.display.flip()
        clock.tick(30)

    pygame.quit()
    if client: client.disconnect()

if __name__ == "__main__":
    main_ui()