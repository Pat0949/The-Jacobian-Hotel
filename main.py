#this main runs everything

import swift
import spatialgeometry as sg
from spatialmath import SE3

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

    input("Press Enter to pick up the spirit bottle\n")
    pick_Place(scene["ur3e"], env, scene["spiritBottle"], DrinkHandover)

    input("Done. Press Enter to close\n")


if __name__ == "__main__":
    main()