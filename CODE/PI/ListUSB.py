#ListUSB.py
# Author: Michael Rechtin 2025
import pygame

def list_connected_joysticks():
    # Initialize Pygame's joystick module
    pygame.init()
    pygame.joystick.init()

    # Get the count of joysticks
    joystick_count = pygame.joystick.get_count()
    print(f"Number of joysticks connected: {joystick_count}")

    # Iterate through each joystick and print its name
    if joystick_count > 0:
        for i in range(joystick_count):
            joystick = pygame.joystick.Joystick(i)
            joystick.init()
            print(f"Joystick {i}: {joystick.get_name()}")
    else:
        print("No joysticks connected.")

    # Quit the Pygame joystick module
    pygame.joystick.quit()

if __name__ == "__main__":
    list_connected_joysticks()
