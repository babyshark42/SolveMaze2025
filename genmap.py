import pygame
import csv
import sys

# --- 1. การตั้งค่า ---
MAZE_WIDTH = 8
MAZE_HEIGHT = 8
CELL_SIZE = 80      # ขนาดช่องแสดงผล
CLICK_TOLERANCE = 20 # ระยะห่างจากเส้นที่ยอมให้คลิกได้ (pixel)

# สี
C_BG = (255, 255, 255)      # พื้นหลังขาว
C_GRID_LINE = (220, 220, 220) # เส้น Grid จางๆ
C_WALL_EXIST = (0, 0, 0)    # กำแพงจริง (สีดำ)
C_WALL_HOVER = (200, 0, 0)  # (Optional)
C_TEXT = (0, 0, 0)

# --- 2. ตัวแปรเก็บข้อมูล (Data Model) ---
# 1 = มีกำแพง, 2 = ไม่มีกำแพง (ใช้ 2 เพื่อให้ตรงกับ Logic Solver เดิมของคุณ)
horizontal_walls = [[2 for _ in range(MAZE_WIDTH)] for _ in range(MAZE_HEIGHT + 1)]
vertical_walls = [[2 for _ in range(MAZE_WIDTH + 1)] for _ in range(MAZE_HEIGHT)]
maze_grid = [[2 for _ in range(MAZE_WIDTH)] for _ in range(MAZE_HEIGHT)] # Grid พื้นหลัง (ไม่ได้ใช้แก้ไข แต่ต้องมีเพื่อ save)

# --- 3. ฟังก์ชันจัดการข้อมูล ---

def reset_map(add_border=False):
    """ ล้างค่าทั้งหมดให้เป็น 2 (ว่าง) """
    global horizontal_walls, vertical_walls
    
    # Reset เป็น 2 (ว่าง)
    horizontal_walls = [[2 for _ in range(MAZE_WIDTH)] for _ in range(MAZE_HEIGHT + 1)]
    vertical_walls = [[2 for _ in range(MAZE_WIDTH + 1)] for _ in range(MAZE_HEIGHT)]
    
    if add_border:
        # สร้างกำแพงรอบนอก (เป็น 1)
        for x in range(MAZE_WIDTH):
            horizontal_walls[0][x] = 1            # ขอบบน
            horizontal_walls[MAZE_HEIGHT][x] = 1  # ขอบล่าง
        for y in range(MAZE_HEIGHT):
            vertical_walls[y][0] = 1           # ขอบซ้าย
            vertical_walls[y][MAZE_WIDTH] = 1  # ขอบขวา

def save_to_csv():
    """ บันทึกไฟล์ CSV 3 ไฟล์ """
    try:
        with open('maze_grid.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerows(maze_grid)
        
        with open('horizontal_walls.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerows(horizontal_walls)
            
        with open('vertical_walls.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerows(vertical_walls)
            
        print(">>> บันทึกไฟล์ CSV สำเร็จ! พร้อมรัน Solver <<<")
    except Exception as e:
        print(f"Error saving CSV: {e}")

def toggle_wall(mx, my):
    """ ตรวจสอบพิกัดเมาส์และสลับสถานะกำแพง """
    # แปลงพิกัดเมาส์เป็น Grid
    gx = mx // CELL_SIZE
    gy = my // CELL_SIZE
    
    # หาตำแหน่งสัมพัทธ์ในช่อง (0-80)
    ox = mx % CELL_SIZE
    oy = my % CELL_SIZE
    
    # 1. เช็คเส้นแนวตั้ง (Vertical) : อยู่ใกล้ขอบซ้ายหรือขวา?
    if ox < CLICK_TOLERANCE: # ใกล้ขอบซ้ายของช่องนี้
        if 0 <= gy < MAZE_HEIGHT and 0 <= gx <= MAZE_WIDTH:
            current = vertical_walls[gy][gx]
            vertical_walls[gy][gx] = 1 if current == 2 else 2
            return True
            
    elif ox > CELL_SIZE - CLICK_TOLERANCE: # ใกล้ขอบขวาของช่องนี้ (คือขอบซ้ายของช่องถัดไป gx+1)
        if 0 <= gy < MAZE_HEIGHT and 0 <= gx + 1 <= MAZE_WIDTH:
            current = vertical_walls[gy][gx + 1]
            vertical_walls[gy][gx + 1] = 1 if current == 2 else 2
            return True

    # 2. เช็คเส้นแนวนอน (Horizontal) : อยู่ใกล้ขอบบนหรือล่าง?
    if oy < CLICK_TOLERANCE: # ใกล้ขอบบน
        if 0 <= gy <= MAZE_HEIGHT and 0 <= gx < MAZE_WIDTH:
            current = horizontal_walls[gy][gx]
            horizontal_walls[gy][gx] = 1 if current == 2 else 2
            return True
            
    elif oy > CELL_SIZE - CLICK_TOLERANCE: # ใกล้ขอบล่าง
        if 0 <= gy + 1 <= MAZE_HEIGHT and 0 <= gx < MAZE_WIDTH:
            current = horizontal_walls[gy + 1][gx]
            horizontal_walls[gy + 1][gx] = 1 if current == 2 else 2
            return True
            
    return False

# --- 4. Main UI Loop ---

def main():
    pygame.init()
    
    SCREEN_WIDTH = MAZE_WIDTH * CELL_SIZE
    SCREEN_HEIGHT = MAZE_HEIGHT * CELL_SIZE + 50 # +50 สำหรับแถบข้อความ
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Maze Editor | Click=Toggle Wall | B=Border | C=Clear | S=Save")
    
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("Arial", 20)
    
    # เริ่มต้นสร้างขอบให้เลย เพื่อความสะดวก
    reset_map(add_border=True)
    
    running = True
    msg_text = "Ready. Click lines to edit."
    
    while running:
        # --- Event Handling ---
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            
            # คลิกเมาส์
            elif event.type == pygame.MOUSEBUTTONDOWN:
                mx, my = pygame.mouse.get_pos()
                if my < MAZE_HEIGHT * CELL_SIZE: # คลิกในพื้นที่ตาราง
                    if toggle_wall(mx, my):
                        msg_text = "Wall Updated."
                
            # กดคีย์บอร์ด
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_s:
                    save_to_csv()
                    msg_text = "Saved to CSV!"
                elif event.key == pygame.K_c:
                    reset_map(add_border=False)
                    msg_text = "Map Cleared."
                elif event.key == pygame.K_b:
                    reset_map(add_border=True)
                    msg_text = "Borders Added."

        # --- Drawing ---
        screen.fill(C_BG)
        
        # 1. วาดเส้น Grid จางๆ (Guide)
        for x in range(MAZE_WIDTH + 1):
            pygame.draw.line(screen, C_GRID_LINE, (x*CELL_SIZE, 0), (x*CELL_SIZE, MAZE_HEIGHT*CELL_SIZE), 1)
        for y in range(MAZE_HEIGHT + 1):
            pygame.draw.line(screen, C_GRID_LINE, (0, y*CELL_SIZE), (MAZE_WIDTH*CELL_SIZE, y*CELL_SIZE), 1)

        # 2. วาดกำแพงจริง (Horizontal Walls)
        for y in range(MAZE_HEIGHT + 1):
            for x in range(MAZE_WIDTH):
                if horizontal_walls[y][x] == 1:
                    start_pos = (x * CELL_SIZE, y * CELL_SIZE)
                    end_pos = ((x + 1) * CELL_SIZE, y * CELL_SIZE)
                    pygame.draw.line(screen, C_WALL_EXIST, start_pos, end_pos, 6) # หนา 6px

        # 3. วาดกำแพงจริง (Vertical Walls)
        for y in range(MAZE_HEIGHT):
            for x in range(MAZE_WIDTH + 1):
                if vertical_walls[y][x] == 1:
                    start_pos = (x * CELL_SIZE, y * CELL_SIZE)
                    end_pos = (x * CELL_SIZE, (y + 1) * CELL_SIZE)
                    pygame.draw.line(screen, C_WALL_EXIST, start_pos, end_pos, 6) # หนา 6px
        
        # 4. วาดแถบสถานะ
        pygame.draw.rect(screen, (240, 240, 240), (0, MAZE_HEIGHT * CELL_SIZE, SCREEN_WIDTH, 50))
        text_surface = font.render(f"[S]ave | [B]order | [C]lear | {msg_text}", True, C_TEXT)
        screen.blit(text_surface, (10, MAZE_HEIGHT * CELL_SIZE + 15))

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()