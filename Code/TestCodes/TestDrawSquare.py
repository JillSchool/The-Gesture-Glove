import pygame
import serial

# Serial settings
serial_port = 'COM3'  
baud_rate = 115200
ser = serial.Serial(serial_port, baud_rate, timeout=0.01)

# Pygame setup
pygame.init()
screen = pygame.display.set_mode((800, 600))
pygame.display.set_caption("MPU6500 Accelerometer Visualization")
clock = pygame.time.Clock()

# Square settings
square_size = 20
center_x = 400
center_y = 300
accel_scale = 50  

# Font setup
font = pygame.font.SysFont('Arial', 24)

# Main loop
running = True
while running:
    screen.fill((0, 0, 0))

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').strip()
        print(f"Ontvangen data: {line}")  

        if "AccX" in line and "AccY" in line:
            try:
                parts = line.split("\t")

                # Acceleration
                accX = float(parts[0].split(":")[1])
                accY = float(parts[1].split(":")[1])

                # Poaition
                square_x = center_x + accX * accel_scale
                square_y = center_y - accY * accel_scale  

                #Screen bounds
                square_x = max(0, min(square_x, 800 - square_size))
                square_y = max(0, min(square_y, 600 - square_size))

                #Square
                pygame.draw.rect(screen, (0, 255, 0), (square_x, square_y, square_size, square_size))

                acc_text = font.render(f"AccX: {accX:.2f}  AccY: {accY:.2f}", True, (255, 255, 255))
                screen.blit(acc_text, (10, 10))

            except Exception as e:
                print(f"Fout bij parsen: {e}")

    pygame.display.flip()
    clock.tick(60)

pygame.quit()
