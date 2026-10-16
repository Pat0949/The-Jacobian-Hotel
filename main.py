#this main runs everything

import swift
import spatialgeometry as sg
from spatialmath import SE3
import numpy as np
from math import pi


from robot_movement import solve_ik, move_arm
from scene import build_scene
from ur3e_pick import pick_Place

#workaround for the no update_pyb error


if not hasattr(sg.Shape, "_update_pyb"):
    sg.Shape._update_pyb = lambda self: None

DrinkHandover = SE3(1.0, -0.8, 0.95)


def main():
    env = swift.Swift()
    env.launch(realtime=True)
    scene = build_scene(env)
    env.step()

    customerOrder = input("What is your oder? (beer, spirit or wine) ").strip().lower()
    pick_Place(scene["ur3e"], env, scene[customerOrder], DrinkHandover)

    q_home = np.zeros(6)
    target_pose = SE3(1.2, 1.3, 0.8) * SE3.Rx(pi)
    q_target = solve_ik(scene["IRB_robot"], target_pose, q_home)

    move_arm(scene["IRB_robot"], q_home, q_target, env, steps=80)
    move_arm(scene["IRB_robot"], q_target, q_home, env, steps=80)

    input("Done. Press Enter to close\n")


if __name__ == "__main__":
    main()