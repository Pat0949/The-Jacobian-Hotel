import swift
from spatialmath import SE3
from spatialgeometry import Sphere
from math import pi
import numpy as np
from roboticstoolbox.backends.swift import Swift
from ir_support_extra_parts.parts import part_mesh



def build_scene(env):

    # bar table
    tablePose = SE3(0.6, 0, 0)
    table = part_mesh("StandingBarTable")
    table.scale = [1, 1, 0.8]
    table.T = tablePose
    env.add(table)

    return {
            "env": env,
    }
    
if __name__ == "__main__":
    env = swift.Swift()
    env.launch(realtime=True)
    build_scene(env)
    input("enter to close it")
#ddddd

#pleaase please please