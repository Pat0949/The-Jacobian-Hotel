import swift
import os
from spatialmath import SE3
import spatialgeometry as sg
from math import pi
import numpy as np
from ir_support.robots import UR3e
from ir_support_extra_parts.parts import part_mesh
from tx2_60 import TX2_60, MESH_DIR

from IRB4600_Visual import IRB4600

def build_scene(env):

    # ur3 robot (sitting behind the bar)
    ur3e = UR3e()
    ur3eHomeQ = np.array([0, -pi/2, 0, 0, 0, 0])
    ur3e.q = ur3eHomeQ
    ur3eLimits = [
        (-2*pi, 2*pi),   
        (-pi, 0),        
        (-pi, pi),       
        (-pi, pi),       
        (-2*pi, 2*pi),   
        (-2*pi, 2*pi),   
    ]
    for link, (lo, hi) in zip(ur3e.links, ur3eLimits):
        link.qlim = [lo, hi]
    
    ur3e.base = SE3(1.2, -1.2, 0.91)
    ur3e.add_to_env(env)

    # ur3e and tx2 chair
    robotChairPositions = [(1.2,-1.25), (0.2,-1.25)]
    for x, y in robotChairPositions:
        robotChair = part_mesh("SimpleTable")
        robotChair.T = SE3(x, y, 0.005)
        robotChair.scale = (0.9,0.675,1.5)
        env.add(robotChair)

    # tables
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
    barTable.scale = (2,0.5,0.8)
    barTable.T = SE3(1,-0.7,0.005)
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
    barTable.T = SE3(1,-1.9,0.705)
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
    wineBottle.T = SE3(0.75, -1.7, 0.95)
    env.add(wineBottle)

    spiritBottle = part_mesh("SpiritBottle")
    spiritBottle.scale = (1,1,1)
    spiritBottle.T = SE3(1, -1.7, 0.95)
    env.add(spiritBottle)

    beerBottle = part_mesh("BeerBottle")
    beerBottle.scale = (1.25,1,1)
    beerBottle.T = SE3(1.2, -1.7, 0.95)
    env.add(beerBottle)

    #cocktailShaker = part_mesh("???")
    #cocktailShaker.scale = (1,1,1)
    #cocktailShaker.T = SE3(0.7,-0.8,0.95)
    cocktailShaker = sg.Cylinder(
        radius=0.04,
        length=0.20,            
        pose=SE3(0.8, -0.8, 1.05),
        color=(0.80, 0.80, 0.80, 1)
    )
    env.add(cocktailShaker)

    
    cocktailGlass = part_mesh("WineCup")
    cocktailGlass.scale = (1.25,1,1)
    cocktailGlass.T = SE3(0.5, -0.6, 0.95)
    env.add(cocktailGlass)


    water = sg.Cylinder(
            radius=0.6,
            length=0.02,
            pose=SE3(0.15, 0.1, 0.02),
            color=(0.2, 0.6, 1.0, 0.7)
        )
    env.add(water)

    Drunkard = part_mesh("personMaleOld")
    Drunkard.T = SE3(1.2,0.45,-0.05) * SE3.Rz(-pi/6) * SE3.Rx(-pi/16)
    env.add(Drunkard)


    #Place and add IRB4600 Robot
    IRB4600_robot = IRB4600(SE3(-1.2, 0.5, 0))
    env.add(IRB4600_robot.base_mesh)
    env.add(IRB4600_robot)

    # place and add tx2_60 robot
    tx2 = TX2_60()
    tx2.base = SE3(0.3, -1.22, 0.9)
    tx2.q = tx2.qz
    env.add(tx2)
    env.add(sg.Mesh(os.path.join(MESH_DIR, "base_link.stl"), pose=tx2.base, color=[0.2, 0.2, 0.2]))

    #caution sign
    cautionSign = part_mesh("CautionSign")
    cautionSign.scale = (1,1,1)
    cautionSign.T = SE3(0, 1.4, 0.05)
    env.add(cautionSign)

    #safety barrier1
    safetyBarrier = part_mesh("barrier1.5x0.2x1m")
    safetyBarrier.scale = (1,1,1)
    safetyBarrier.T = SE3(3.3, -1.5, 0.05) * SE3.Rz(pi/2)
    env.add(safetyBarrier)

    #safety barrier2
    safetyBarrier2 = part_mesh("barrier1.5x0.2x1m")
    safetyBarrier2.scale = (1,1,1)
    safetyBarrier2.T = SE3(-1.3, -1.5, 0.05) * SE3.Rz(pi/2)
    env.add(safetyBarrier2)

    eStop = part_mesh("emergencyStopButton")
    eStop.scale = (0.5,0.5,0.5)
    eStop.T = SE3(1.8,-0.7,0.92)
    env.add(eStop)
    


    return {
            "env": env,
            "ur3e": ur3e,
            "beer": beerBottle,
            "spirit": spiritBottle,
            "wine": wineBottle,
            "floor": floor,
            "water": water,
            "IRB_robot": IRB4600_robot,
            "tx2": tx2,
            "shaker": cocktailShaker,
            "glass": cocktailGlass
    }
    
if __name__ == "__main__":
    env = swift.Swift()
    env.launch(realtime=True)
    build_scene(env)
    input("enter to close it")
#ddddd

#pleaase please please
