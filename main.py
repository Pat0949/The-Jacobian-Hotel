#this main runs everything

import swift
import spatialgeometry as sg
import threading
from spatialmath import SE3
from raysensor_collision_detection import EndEffectorRay
from joystick_mode import manual_irb, calculate_dof
from scene import build_scene
from bar_movements import pick_place, make_cocktail



#workaround for the no update_pyb error
if not hasattr(sg.Shape, "_update_pyb"):
    sg.Shape._update_pyb = lambda self: None

DrinkHandover = SE3(1.0, -0.8, 0.95)

def main():
    env = swift.Swift()
    env.launch(realtime=True)
    scene = build_scene(env)
    env.set_camera_pose([0.8, 0.37, 1.7], [0.8, -1, 1.5])
    env.step()

    cocktail_done = False

    customerOrder = input("What is your order? (beer, wine or cocktail) ").strip().lower()
    if customerOrder == "cocktail":
        make_cocktail(env, scene)
        cocktail_done = True
    else:
        pick_place(env, scene["ur3e"], scene[customerOrder], DrinkHandover)

    if cocktail_done == True:
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

        # Labels
        env.add(label_x)
        env.add(label_y)
        env.add(label_z)
        env.add(label_roll)
        env.add(label_pitch)
        env.add(label_yaw)  

        input_thread = threading.Thread(target=scene["IRB_robot"])
        input_thread.start()  

        # Joystick mode
        ray = EndEffectorRay(scene["IRB_robot"], scene["floor"], env)
        manual_irb(scene, l_x=label_x, l_y=label_y, l_z=label_z, l_roll=label_roll, l_pitch=label_pitch, l_yaw=label_yaw, ray=ray)

if __name__ == "__main__":
    main()