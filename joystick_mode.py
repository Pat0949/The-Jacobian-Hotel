import pygame
import numpy as np

def manual_irb(scene, dt=0.1, ray=None):
    # -- Setup Joystick --
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("No joystick detected.")
        exit()

    joy = pygame.joystick.Joystick(0)
    joy.init()

    # -- Setup Robot --
    env = scene["env"]
    water = scene["water"]
    robot = scene["IRB_robot"]
    glass = scene["glass"]

    env.set_camera_pose([0.7, 2.9, 1.9], [0.2, 0, 0.2])

    water_pose = water.T.copy()
    water_pose[2, 3] = 0.02

    #glass_pose = glass.T.copy()
    #glass_pose[2, 3] = 1

    print("\n------------------------------------------------------------------------------------")
    print("                                     Joystick Controls")
    print(f" L-Stick: body move   R-Stick: end effector move   D-pad: up, down, left, right")
    print("                                    Press B to reset position")
    print("                                   Collision status: None")
    print("                                 Press Control + C to close")
    print("------------------------------------------------------------------------------------")

    # -- Simulation Parameters --
    Kv = 0.3  # linear velocity gain
    Kw = 0.8  # angular velocity gain
    lambda_dls = 0.1  # DLS damping factor

    joystick_deadzone_threshold = 0.6  # deadzone threshold for joystick axes
    button_gain = 0.5  # button velocity gain (since buttons are binary)

    in_collision = False

    # -- Main Control Loop --
    try:
        while True:
            pygame.event.pump()

            # Read and print raw axis values
            axes_raw = [joy.get_axis(i) for i in range(joy.get_numaxes())]
            # Apply deadzone threshold
            axes = [0 if abs(a) < joystick_deadzone_threshold else a for a in axes_raw]
            buttons = [joy.get_button(i) for i in range(joy.get_numbuttons())]

            # Joystick to velocity mapping
            vx = Kv * -axes[1]  # Left stick left/right
            vy = Kv * -axes[0]  # Left stick up/down
            vz = Kv * button_gain * (buttons[11] - buttons[12])  # up down buttons

            wx = Kw * axes[2]  # Right stick left/right
            wy = Kw * axes[3]  # Right stick up/down
            wz = Kw * (buttons[13] - buttons[14])  # left right buttons

            dx = np.array([vx, vy, vz, wx, wy, wz])  # 6D velocity

            # Damped Least Squares inverse Jacobian
            J = robot.jacob0(robot.q)
            JTJ = J.T @ J
            lambda_I = (lambda_dls**2) * np.eye(J.shape[1])
            Jinv_dls = np.linalg.inv(JTJ + lambda_I) @ J.T

            dq = Jinv_dls @ dx
            robot.q = robot.q + dq * dt

            # check for a collision before the ray is lifted away from the floor
            collided = ray.hit_floor()

            if ray is not None:
                ray.keep_above_floor()
                ray.update()

            if collided and not in_collision:
                print("------------------------------------------------------------------------------------\n")
                print("\n                                  Floor detected!\n")
                print("\n------------------------------------------------------------------------------------")
            if not collided and in_collision:
                print("\n------------------------------------------------------------------------------------")
                print("                                     Joystick Controls")
                print(f" L-Stick: body move   R-Stick: end effector move   D-pad: up, down, left, right")
                print("                                    Press B to reset position")
                print("                                   Collision status: None")
                print("                                 Press Control + C to close")
                print("------------------------------------------------------------------------------------")
            in_collision = collided

            # Animate robot
            env.step(dt)

            reset_button = 1
            
            if buttons[reset_button]:
                robot.q = np.zeros(6)
                print("Resetting robot to zero configuration.")

    except KeyboardInterrupt:
        print(" Exiting...")
    finally:
        pygame.quit()