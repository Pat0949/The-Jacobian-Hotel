import pygame
import time

# Initialize pygame joystick module
pygame.init()
pygame.joystick.init()

# Get joystick
if pygame.joystick.get_count() == 0:
    print("No joystick detected.")
    exit()

joy = pygame.joystick.Joystick(0)
joy.init()

print("Your joystick has:")
print(f" - {joy.get_numbuttons()} buttons")
print(f" - {joy.get_numaxes()} axes")
time.sleep(2)

# Main loop
try:
    while True:
        pygame.event.pump()  # Update internal state

        print("--------------")
        for i in range(joy.get_numbuttons()):
            print(f"Button[{i+1}]: {joy.get_button(i)}")
        for i in range(joy.get_numaxes()):
            print(f"Axis[{i+1}]: {joy.get_axis(i):.3f}")
        print("--------------")

        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nExiting...")
    pygame.quit()
