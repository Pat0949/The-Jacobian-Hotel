"""
    joint_1 base_link->link_1 (0, 0, 0.375)      z
    joint_2 link_1->link_2    (0, 0, 0)          y
    joint_3 link_2->link_3    (0, 0.02, 0.29)    y
    joint_4 link_3->link_4    (0, 0, 0)          z
    joint_5 link_4->link_5    (0, 0, 0.31)       y
    joint_6 link_5->link_6    (0, 0, 0.07)       z
"""
 
import os
import numpy as np
import spatialgeometry as sg
 
try:
    from roboticstoolbox import ELink as Link, ERobot as Robot
except ImportError:
    from roboticstoolbox import Link, Robot
from roboticstoolbox import ET
 
MESH_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "meshes")
deg = np.pi / 180
 
 
def mesh(filename, color):
    return [sg.Mesh(filename=os.path.join(MESH_DIR, filename), color=color)]
 
 
base_link = Link(
    ET.tx(0),
    name="base_link",
    geometry=mesh("base_link.stl", [0.2, 0.2, 0.2]),
)
 
link_1 = Link(
    ET.tz(0.375) * ET.Rz(),
    parent=base_link,
    name="link_1",
    qlim=[-180 * deg, 180 * deg],
    geometry=mesh("link_1.stl", [0.9, 0.8, 0.1]),
)
 
link_2 = Link(
    ET.Ry(),
    parent=link_1,
    name="link_2",
    qlim=[-127.5 * deg, 127.5 * deg],
    geometry=mesh("link_2.stl", [0.9, 0.8, 0.1]),
)
 
link_3 = Link(
    ET.ty(0.02) * ET.tz(0.29) * ET.Ry(),
    parent=link_2,
    name="link_3",
    qlim=[-142.5 * deg, 142.5 * deg],
    geometry=mesh("link_3.stl", [0.9, 0.8, 0.1]),
)
 
link_4 = Link(
    ET.Rz(),
    parent=link_3,
    name="link_4",
    qlim=[-270 * deg, 270 * deg],
    geometry=mesh("link_4.stl", [0.7, 0.7, 0.7]),
)
 
link_5 = Link(
    ET.tz(0.31) * ET.Ry(),
    parent=link_4,
    name="link_5",
    qlim=[-121 * deg, 132.5 * deg],
    geometry=mesh("link_5.stl", [0.7, 0.7, 0.7]),
)
 
link_6 = Link(
    ET.tz(0.07) * ET.Rz(),
    parent=link_5,
    name="link_6",
    qlim=[-270 * deg, 270 * deg],
    geometry=mesh("link_6.stl", [0.7, 0.7, 0.7]),
)
 
robot = Robot(
    [base_link, link_1, link_2, link_3, link_4, link_5, link_6],
    name="Staubli_TX2_60_visual",
)
 
 
if __name__ == "__main__":
    print(robot)
 
    # NOTE: unlike tx2_60.py's DH model, this model's joint zero positions
    # come straight from the URDF, so they don't line up with the old
    # model's q values. This pose was found by numerically maximising
    # horizontal reach (verified independently, converges to 670.3mm vs
    # the datasheet's 670mm) -- it's the equivalent "arm fully extended"
    # pose for this model.
    q_extended = np.array([0, -90 * deg, 0, 0, 0, 0])
    T = robot.fkine(q_extended)
    horiz_reach = np.hypot(T.t[0], T.t[1])
    print(f"\nEnd-effector pose at extended pose: \n{T}")
    print(f"Horizontal reach: {horiz_reach * 1000:.1f} mm "
          f"(datasheet quotes 670 mm)")
 
    from roboticstoolbox.backends.swift import Swift
 
    env = Swift()
    env.launch(realtime=True)
    env.add(robot)
    robot.q = np.array([0, -60 * deg, 60 * deg, 0, 30 * deg, 0])
    env.step()
    env.hold()
 