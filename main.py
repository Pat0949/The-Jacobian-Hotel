#this main runs everything
import matplotlib
matplotlib.use("Agg")
import warnings
warnings.filterwarnings("ignore", category=UserWarning)
import swift
import spatialgeometry as sg
from spatialmath import SE3
from raysensor_collision_detection import EndEffectorRay
from joystick_mode import manual_irb
from scene import build_scene
from bar_movements import pick_place, make_cocktail


#workaround for the no update_pyb error
if not hasattr(sg.Shape, "_update_pyb"):
    sg.Shape._update_pyb = lambda self: None

DrinkHandover = SE3(1.0, -0.8, 0.95)

cocktail_done = False

def main():
    env = swift.Swift()
    env.launch(realtime=True)
    scene = build_scene(env)
    env.set_camera_pose([0.8, 0.37, 1.7], [0.8, -1, 1.5])
    env.step()

    customerOrder = input("What is your order? (beer, wine or cocktail) ").strip().lower()
    if customerOrder == "cocktail":
        make_cocktail(env, scene)
        cocktail_done = True
    else:
        pick_place(env, scene["ur3e"], scene[customerOrder], DrinkHandover)

    if cocktail_done == True:
        print("--------------------------------------------------------------------------------------------\n")
        print("                                        Oh No!")
        print("                               You Spilt Your cocktail!")
        print("                                 Time to Clean it Up!\n")
        input("                               (Press Enter to continue)")
        print("\n--------------------------------------------------------------------------------------------")
        ray = EndEffectorRay(scene["IRB_robgot"], scene["floor"], env)
        manual_irb(scene, ray=ray)

if __name__ == "__main__":
    main()