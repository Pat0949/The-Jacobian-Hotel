import swift
from spatialmath import SE3
import spatialgeometry as sg
from math import pi
import numpy as np
from ir_support.robots import UR3e
from roboticstoolbox.backends.swift import Swift
from ir_support_extra_parts.parts import part_mesh



def build_scene(env):

    # ur3 robot (sitting behind the bar)
    ur3e = UR3e()
    ur3eHomeQ = np.array([0, -pi/2, 0, 0, 0, 0])
    ur3e.q = ur3eHomeQ
    ur3eLimits = [
        (-2*pi, 2*pi),   # joint 1 - base
        (-2*pi/3, 0),   # joint 2 - shoulder
        (-pi, pi),       # joint 3 - elbow
        (-2*pi/3, 2*pi/3),   # joint 4 - wrist 1
        (-2*pi, 2*pi),   # joint 5 - wrist 2
        (-2*pi, 2*pi),   # joint 6 - wrist 3
    ]
    for link, (lo, hi) in zip(ur3e.links, ur3eLimits):
        link.qlim = [lo, hi]
    
    ur3e.base = SE3(1, -1.2, 0.91)
    ur3e.add_to_env(env)

    # ur3e chair
    ur3eChair = part_mesh("SimpleTable")
    ur3eChair.scale = (0.9,0.675,1.5)
    ur3eChair.T = SE3(1,-1.2,0.005)
    env.add(ur3eChair)

    # bar table
    tablePositions = [(1, 5), (2.5, 2), (-0.5, 2.5)]
    for x, y in tablePositions:
        tablePose = SE3(x, y, 0)
        table = part_mesh("StandingBarTable")
        table.scale = [1, 1, 0.8]
        table.T = tablePose
        env.add(table)

    # bar stools
    chairsPositions = [(-0.5, 2.9), (-0.1, 2.5), (-0.5, 2.1), (1, 5.4), (1.4, 5), (1, 4.6), (2.9, 2), (2.1, 2)]
    for x, y in chairsPositions:
        chairs = part_mesh("SimpleTable")
        chairs.scale = (0.6,0.45,0.9)
        chairs.T = SE3(x, y, 0)
        env.add(chairs)

    # bar table
    barTable = part_mesh("BarTable")
    barTable.scale = (2,0.8,0.8)
    barTable.T = SE3(1,-0.45,0.005)
    env.add(barTable)

    floor = sg.Cuboid(
        scale=[6, 9, 0.001],                 
        pose=SE3(1, 2, 0.005),             
        color=(0.18, 0.17, 0.16, 1.0)
    )
    env.add(floor)

    cabinet = sg.Cuboid(
            scale=[3, 0.7, 0.7],                 
            pose=SE3(1, -2, 0.355),            
            color=(0.30, 0.20, 0.12, 1),
        )
    env.add(cabinet)

    # shelf
    barTable = part_mesh("WallShelf")
    barTable.scale = (4,2.5,2)
    barTable.T = SE3(1,-2,0.705)
    env.add(barTable)

    wall = sg.Cuboid(
            scale=[6, 0.005, 2.2],                 
            pose=SE3(1, -2.5, 1.1),            
            color=(0.85, 0.77, 0.62, 0.6)
        )
    env.add(wall)

    # wine bottle
    wineBottle = part_mesh("WineBottle")
    wineBottle.scale = (1,1,1)
    wineBottle.T = SE3(0.75, -1.85, 0.95)
    env.add(wineBottle)

    spiritBottle = part_mesh("SpiritBottle")
    spiritBottle.scale = (1,1,1)
    spiritBottle.T = SE3(1, -1.85, 0.95)
    env.add(spiritBottle)

    beerBottle = part_mesh("BeerBottle")
    beerBottle.scale = (1.25,1,1)
    beerBottle.T = SE3(1.2, -1.85, 0.95)
    env.add(beerBottle)


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