import pygame
import swift
import numpy as np
from spatialmath.base import *

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

    env.set_camera_pose([0.7, 2.9, 1.9], [0.2, 0, 0.2])

    water_pose = water.T.copy()
    water_pose[2, 3] = 0.02

    # -- Simulation Parameters --
    Kv = 0.3  # linear velocity gain
    Kw = 0.8  # angular velocity gain
    lambda_dls = 0.1  # DLS damping factor

    joystick_deadzone_threshold = 0.6  # deadzone threshold for joystick axes
    button_gain = 0.5  # button velocity gain (since buttons are binary)

    in_collision = False
    estop_active = False
    robot_active = False

    button_estop = swift.Button(cb=lambda value: estop_callback(value), desc='Toggle Emergency Stop')
    env.add(button_estop)

    # Tech Pendant
    xyz, rpy = calculate_dof(scene["IRB_robot"])

    # Creating Labels to show XYZ, Roll, Pitch, Yaw
    # XYZ
    text_x, text_y, text_z = f"X: {xyz[0]}", f"Y: {xyz[1]}", f"Z: {xyz[2]}"
    label_x, label_y, label_z = swift.Label(text_x), swift.Label(text_y), swift.Label(text_z)
    # RPY
    text_roll, text_pitch, text_yaw = f"Roll (φ): {rpy[0]}", f"Pitch (θ): {rpy[1]}", f"Yaw (ψ): {rpy[2]}"
    label_roll, label_pitch, label_yaw = swift.Label(text_roll), swift.Label(text_pitch), swift.Label(text_yaw)
    # Create list of labels
    labels = [label_x, label_y, label_z, label_roll, label_pitch, label_yaw]

    header = ("JoyStick Movements and Controls")
    smessage = ("PRESS A TO ACTIVATE THE CLEANER!")
    controls_L = ("L-Stick: body move")
    controls_R = ("R-Stick: end effector move")
    controls_D = ("D-pad: up, down, left, right")
    e_stop = ("Press B to Emergency Stop")
    emessage1 = ("EMERGENCY STOP BUTTON ACTIVATED\n")
    emessage2 = ("(Press B to turn off the emergency stop button)")
    label_header, label_e_stop, label_c_L, label_c_R, label_c_D, label_smessage, label_emessage1, label_emessage2= swift.Label(header), swift.Label(e_stop), swift.Label(controls_L), swift.Label(controls_R), swift.Label(controls_D), swift.Label(smessage), swift.Label(emessage1), swift.Label(emessage2)

    # Labels
    # every label is added once and only its text is changed afterwards
    all_labels = [label_smessage, label_emessage1, label_emessage2,
                  label_header, label_c_L, label_c_R, label_c_D, label_e_stop,
                  label_x, label_y, label_z, label_roll, label_pitch, label_yaw]

    for lbl in all_labels:
        env.add(lbl)

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

            if estop_active == False and robot_active:
                robot.q = robot.q + dq * dt

            collided = False
            if ray is not None:
                collided = ray.hit_floor()
                ray.keep_above_floor()
                ray.update()

            if collided and not in_collision:
                print("Floor detected!")

            in_collision = collided

            if estop_active == False and robot_active:
                # calculate robots current position
                xyz, rpy = calculate_dof(robot)
                label_x.desc = f"X: {xyz[0]}"
                label_y.desc = f"Y: {xyz[1]}"
                label_z.desc = f"Z: {xyz[2]}"
                label_roll.desc = f"Roll: {rpy[0]}"
                label_pitch.desc = f"Pitch: {rpy[1]}"
                label_yaw.desc = f"Yaw: {rpy[2]}"

            # blank everything then fill in labels for the current state
            for lbl in all_labels:
                lbl.desc = ""

            if estop_active == False and robot_active == False:
                label_smessage.desc = smessage
            elif estop_active == False and robot_active == True:
                label_header.desc = header
                label_c_L.desc = controls_L
                label_c_R.desc = controls_R
                label_c_D.desc = controls_D
                label_e_stop.desc = e_stop
            elif estop_active == True:
                label_emessage1.desc = emessage1
                label_emessage2.desc = emessage2

            if buttons[1] and not estop_active:
                print("Emergency Stop On")
                estop_active = True
                robot_active = False
            elif buttons[1] and estop_active:
                print("Emergency Stop Off")
                estop_active = False

            if buttons[0] and not estop_active:
                print("Robot Activated")
                robot_active = True

            # callback for swift estop
            def estop_callback(value):
                nonlocal estop_active, robot_active
                estop_active = not estop_active
                robot_active = False

            # Animate robot
            env.step(dt)

    except KeyboardInterrupt:
        print(" Exiting...")
    finally:
        pygame.quit()

def calculate_dof(robot):

    ee_tr = robot.fkine(robot.q).A
    xyz = np.round(ee_tr[0:3, 3], 3)
    rpy = np.round(tr2rpy(ee_tr, unit="deg"), 2)

    return xyz, rpy