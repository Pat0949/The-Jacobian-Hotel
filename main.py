#this main runs everything

import swift
import spatialgeometry as sg
from spatialmath import SE3
import numpy as np
from math import pi
from raysensor_collision_detection import EndEffectorRay
from joystickcontrols import manual_irb
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

    ray = EndEffectorRay(scene["IRB_robot"], scene["floor"], env)
    manual_irb(scene, ray=ray)

if __name__ == "__main__":
    main()