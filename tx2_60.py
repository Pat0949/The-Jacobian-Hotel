import os
import shutil
import tempfile
import numpy as np
import swift
import spatialgeometry as sg
from spatialmath import SE3
from roboticstoolbox import DHRobot, RevoluteDH

# Workaround for my update_pyb error
if not hasattr(sg.Shape, "_update_pyb"):
    sg.Shape._update_pyb = lambda self: None

deg = np.pi / 180
YELLOW = [0.9, 0.8, 0.1]
GREY = [0.7, 0.7, 0.7]

# Liam if it doesnt run and has an issue finding the mesh files uncomment these 3 lines
#PROJECT_MESHES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "meshes")
#MESH_DIR = os.path.join(tempfile.gettempdir(), "tx2_60_meshes")
#shutil.copytree(PROJECT_MESHES, MESH_DIR, dirs_exist_ok=True)
MESH_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "meshes")

# d, a, alpha, offset, joint limits, colour
DH = [
    (0.375, 0.0,   -np.pi / 2,  0.0,       [-180 * deg, 180 * deg],     YELLOW),
    (0.020, 0.290,  0.0,       -np.pi / 2, [-127.5 * deg, 127.5 * deg], YELLOW),
    (0.0,   0.0,    np.pi / 2,  np.pi / 2, [-142.5 * deg, 142.5 * deg], YELLOW),
    (0.310, 0.0,   -np.pi / 2,  0.0,       [-270 * deg, 270 * deg],     GREY),
    (0.0,   0.0,    np.pi / 2,  0.0,       [-121 * deg, 132.5 * deg],   GREY),
    (0.070, 0.0,    0.0,        0.0,       [-270 * deg, 270 * deg],     GREY),
]

# joint origins within the world cooridinate frame
MESH_ORIGINS = [
    (0, 0.00, 0.375),   # link_1
    (0, 0.00, 0.375),   # link_2
    (0, 0.02, 0.665),   # link_3
    (0, 0.02, 0.665),   # link_4
    (0, 0.02, 0.975),   # link_5
    (0, 0.02, 1.045),   # link_6
]


class TX2_60(DHRobot): #needed to reposition the joints since all their origins lie at the joint.
    def __init__(self): #so we have an offset of the mesh origins
        links = []
        T = SE3()   # DH frame at the zero pose for each link going up
        for i, (d, a, al, off, qlim, colour) in enumerate(DH):
            T = T * RevoluteDH(d=d, a=a, alpha=al, offset=off).A(0)
            mesh = sg.Mesh(os.path.join(MESH_DIR, f"link_{i + 1}.stl"),
                           pose=T.inv() * SE3(*MESH_ORIGINS[i]), color=colour)
            links.append(RevoluteDH(d=d, a=a, alpha=al, offset=off, qlim=qlim, geometry=[mesh]))

        super().__init__(links, name="Staubli_TX2_60", manufacturer="Staubli")
        self.qz = np.zeros(6)   # straight up


if __name__ == "__main__":
    robot = TX2_60()

    env = swift.Swift()
    env.launch(realtime=True)

    robot.q = robot.qz
    env.add(robot)
    env.add(sg.Mesh(os.path.join(MESH_DIR, "base_link.stl"), pose=robot.base, color=[0.2, 0.2, 0.2]))
    env.step()
    env.hold()