import pygame
import sys

# เริ่มต้น Pygame และ font system
pygame.init()

def get_hoop_position():
    # กำหนดตัวแปรสำหรับเก็บค่าตำแหน่ง
    x_value = ''
    z_value = ''
    input_active_x = True  # เริ่มต้นที่กรอกค่า X
    input_active_z = False

    # สร้างหน้าต่าง Pygame
    WIDTH, HEIGHT = 500, 400
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Basketball Hoop Position Input")

    # โหลดรูปพื้นหลัง
    background = pygame.image.load("Background_bas.png")
    background = pygame.transform.scale(background, (WIDTH, HEIGHT))

    # โหลดรูปสัญลักษณ์ NBA
    nba_logo = pygame.image.load("NBA.png")  # เปลี่ยนชื่อไฟล์โลโก้ NBA ตามชื่อไฟล์ของคุณ
    nba_logo = pygame.transform.scale(nba_logo, (30, 50))  # ปรับขนาดโลโก้ NBA

    # กำหนดสี
    WHITE = (255, 255, 255)
    BLACK = (0, 0, 0)
    GREEN = (0, 255, 0)
    GRAY = (200, 200, 200)
    ORANGE = (255, 140, 0)

    # โหลดรูปไอคอนลูกบาสเกตบอล
    basketball_icon = pygame.image.load("Basketball_icon.png")
    basketball_icon = pygame.transform.scale(basketball_icon, (30, 30))

    # กำหนดฟอนต์
    title_font = pygame.font.Font(None, 30)
    input_font = pygame.font.Font(None, 26)
    button_font = pygame.font.Font(None, 30)

    # กำหนดระยะเลื่อนลง
    y_offset = 60

    # กำหนดตำแหน่ง UI
    input_box_x = pygame.Rect(200, 100 + y_offset, 140, 32)
    input_box_z = pygame.Rect(200, 160 + y_offset, 140, 32)
    button_rect = pygame.Rect(150, 220 + y_offset, 200, 50)

    # ฟังก์ชันแสดงข้อความ
    def draw_text(text, font, color, surface, x, y):
        label = font.render(text, True, color)
        surface.blit(label, (x, y))

    # ฟังก์ชันยืนยันค่าที่ป้อน
    def confirm_values():
        try:
            x_position = float(x_value)
            z_position = float(z_value)
            pygame.quit()
            return x_position, z_position
        except ValueError:
            print("Invalid input. Please enter valid numeric values.")
            return None, None

    # ฟังก์ชันหลักในการรับข้อมูลจาก Pygame
    while True:
        screen.blit(background, (0, 0))  # วาดรูปพื้นหลัง

        # วาดโลโก้ NBA ที่มุมขวาบน
        screen.blit(nba_logo, (WIDTH - 60, 10))  # ตำแหน่ง x = ขวาสุด - ขนาดโลโก้, y = 10

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

            # การคลิกช่องกรอกข้อความ
            if event.type == pygame.MOUSEBUTTONDOWN:
                input_active_x = input_box_x.collidepoint(event.pos)
                input_active_z = input_box_z.collidepoint(event.pos)
                # การคลิกปุ่มยืนยัน
                if button_rect.collidepoint(event.pos):
                    x, z = confirm_values()
                    if x is not None and z is not None:
                        return x, z

            # การพิมพ์ในช่องกรอกข้อความ
            if event.type == pygame.KEYDOWN:
                if input_active_x:
                    if event.key == pygame.K_BACKSPACE:
                        x_value = x_value[:-1]
                    elif event.unicode.isdigit() or event.unicode == '.':
                        # รับเฉพาะตัวเลขและจุดทศนิยม
                        if event.unicode == '.' and '.' in x_value:
                            continue  # ป้องกันการพิมพ์จุดมากกว่าหนึ่งจุด
                        x_value += event.unicode

                if input_active_z:
                    if event.key == pygame.K_BACKSPACE:
                        z_value = z_value[:-1]
                    elif event.unicode.isdigit() or event.unicode == '.':
                        # รับเฉพาะตัวเลขและจุดทศนิยม
                        if event.unicode == '.' and '.' in z_value:
                            continue  # ป้องกันการพิมพ์จุดมากกว่าหนึ่งจุด
                        z_value += event.unicode

                # กด Enter หรือ Enter บน Numpad เพื่อยืนยัน
                if event.key in [pygame.K_RETURN, pygame.K_KP_ENTER]:  # เพิ่มเงื่อนไขสำหรับ Numpad Enter
                    x, z = confirm_values()
                    if x is not None and z is not None:
                        return x, z

                # กดปุ่มลูกศรขึ้นและลงเพื่อสลับช่องกรอก
                if event.key == pygame.K_UP:
                    input_active_x = True
                    input_active_z = False
                elif event.key == pygame.K_DOWN:
                    input_active_x = False
                    input_active_z = True

        # วาดข้อความหัวข้อ
        pygame.draw.rect(screen, ORANGE, (100, 20 + y_offset, 300, 50))
        draw_text("Set Basketball Hoop Position", title_font, BLACK, screen, 105, 30 + y_offset)

        # วาดไอคอนและข้อความช่องกรอก
        screen.blit(basketball_icon, (45, 100 + y_offset))  # ไอคอนลูกบาสหน้าข้อความ X-Position
        draw_text("X-Position (m):", input_font, BLACK, screen, 85, 105 + y_offset)  # ปรับข้อความให้เลื่อนหลังไอคอน

        screen.blit(basketball_icon, (45, 160 + y_offset))  # ไอคอนลูกบาสหน้าข้อความ Z-Position
        draw_text("Z-Position (m):", input_font, BLACK, screen, 85, 165 + y_offset)  # ปรับข้อความให้เลื่อนหลังไอคอน

        # วาดช่องกรอกข้อความ
        pygame.draw.rect(screen, GRAY, input_box_x)
        pygame.draw.rect(screen, GRAY, input_box_z)

        # แสดงค่าที่พิมพ์ลงไปในช่องกรอกข้อความ
        draw_text(x_value, input_font, BLACK, screen, input_box_x.x + 5, input_box_x.y + 5)
        draw_text(z_value, input_font, BLACK, screen, input_box_z.x + 5, input_box_z.y + 5)

        # วาดปุ่มยืนยัน
        pygame.draw.rect(screen, GREEN, button_rect)
        draw_text("Enter", button_font, BLACK, screen, button_rect.x + 70, button_rect.y + 10)

        # อัพเดทหน้าจอ
        pygame.display.flip()

        # ตั้งเวลาเฟรม
        pygame.time.Clock().tick(30)

# เรียกใช้งานฟังก์ชัน
if __name__ == "__main__":
    x, z = get_hoop_position()
    print(f"X: {x}, Z: {z}")
