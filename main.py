#this main runs everything

import swift
import spatialgeometry as sg
from spatialmath import SE3
import numpy as np
from math import pi
from raysensor import EndEffectorRay
from joystickcontrols import manual_irb
from robot_movement import solve_ik, move_arm
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
    env.step()

    customerOrder = input("What is your oder? (beer, wine or cocktail) ").strip().lower()
    if customerOrder == "cocktail":
        make_cocktail(env, scene)
    else:
        pick_place(env, scene["ur3e"], scene[customerOrder], DrinkHandover)

    q_home = np.zeros(6)
    target_pose = SE3(1.2, 1.3, 0.8) * SE3.Rx(pi)
    q_target = solve_ik(scene["IRB_robot"], target_pose, q_home)

    ray = EndEffectorRay(scene["IRB_robot"], scene["floor"], env)

    manual_irb(scene, ray=ray)

    input("Done. Press Enter to close\n")


if __name__ == "__main__":
    main() 