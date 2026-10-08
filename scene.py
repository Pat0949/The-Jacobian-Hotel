import swift
from spatialmath import SE3
from spatialgeometry import Sphere
from math import pi
import numpy as np
from roboticstoolbox.backends.swift import Swift
from ir_support_extra_parts.parts import part_mesh



def build_scene(env):

    # bar table
    tablePositions = [(1, 3), (2.5, 0), (0, 0)]
    for x, y in tablePositions:
        tablePose = SE3(x, y, 0)
        table = part_mesh("StandingBarTable")
        table.scale = [1, 1, 0.8]
        table.T = tablePose
        env.add(table)

    # bar stools
    chairsPositions = [(0, 0.4), (0.4, 0), (0, -0.4), (1, 3.4), (1.4, 3), (1, 2.6), (2.9, 0), (2.1, 0)]
    for x, y in chairsPositions:
        chairs = part_mesh("SimpleTable")
        chairs.scale = (0.6,0.45,0.9)
        chairs.T = SE3(x, y, 0)
        env.add(chairs)

    # bar table
    barTable = part_mesh("BarTable")
    barTable.scale = (1.5,1,1)
    barTable.T = SE3(1,-2,0)
    env.add(barTable)

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