import pygame
import numpy as np

def manual_irb(scene, dt=0.1, attached_object=None, object_offset=None, ray=None):
    # -- Setup Joystick --
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("No joystick detected.")
        exit()

    joy = pygame.joystick.Joystick(0)
    joy.init()

    print(f"Joystick: {joy.get_name()}")
    print(f" - {joy.get_numbuttons()} buttons")
    print(f" - {joy.get_numaxes()} axes")

    # -- Setup Robot --
    env = scene["env"]
    robot = scene["IRB_robot"]

    # -- Simulation Parameters --
    Kv = 0.3  # linear velocity gain
    Kw = 0.8  # angular velocity gain
    lambda_dls = 0.1  # DLS damping factor

    joystick_deadzone_threshold = 0.6  # deadzone threshold for joystick axes
    button_gain = 0.5  # button velocity gain (since buttons are binary)

    # -- Main Control Loop --
    try:
        while True:
            pygame.event.pump()

            # Read and print raw axis values
            axes_raw = [joy.get_axis(i) for i in range(joy.get_numaxes())]
            # Apply deadzone threshold
            axes = [0 if abs(a) < joystick_deadzone_threshold else a for a in axes_raw]
            buttons = [joy.get_button(i) for i in range(joy.get_numbuttons())]

            # Read triggers separately because they rest at -1 and reach 1 when fully pressed
            left_trigger = (axes_raw[5] + 1) / 2 if len(axes_raw) > 5 else 0
            right_trigger = (axes_raw[4] + 1) / 2 if len(axes_raw) > 4 else 0

            # Apply the same deadzone idea to the rescaled values
            left_trigger = 0 if left_trigger < joystick_deadzone_threshold else left_trigger
            right_trigger = 0 if right_trigger < joystick_deadzone_threshold else right_trigger

            # Joystick to velocity mapping
            vx = Kv * -axes[1]  # Left stick left/right
            vy = Kv * -axes[0]  # Left stick up/down
            vz = Kv * button_gain * (buttons[11] - buttons[12])  # up down buttons

            wx = Kw * axes[2]  # Right stick left/right
            wy = Kw * axes[3]  # Right stick up/down
            wz = Kw * (right_trigger - left_trigger)  # left right triggers

            dx = np.array([vx, vy, vz, wx, wy, wz])  # 6D velocity

            # Damped Least Squares inverse Jacobian
            J = robot.jacob0(robot.q)
            JTJ = J.T @ J
            lambda_I = (lambda_dls**2) * np.eye(J.shape[1])
            Jinv_dls = np.linalg.inv(JTJ + lambda_I) @ J.T

            dq = Jinv_dls @ dx
            robot.q = robot.q + dq * dt

            ray.keep_above_floor()
            ray.update()

            if attached_object is not None and object_offset is not None:
                robot_pose = robot.fkine(robot.q)
                attached_object.T = robot_pose * object_offset

            if ray is not None:
                ray.update()

            # Animate robot
            env.step(dt)

            reset_button = 1
            glass_button = 2
            vacuum_button = 3
            
            if buttons[reset_button]:
                robot.q = np.zeros(6)
                print("Resetting robot to zero configuration.")

    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        pygame.quit()
