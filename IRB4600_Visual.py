import os
import numpy as np
from roboticstoolbox import DHLink, DHRobot
import spatialgeometry as sg

from swift import Swift
from spatialmath import SE3

MESH_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ARB_Meshes")
MESH_NAMES = ["base_link.dae", "link_1.dae", "link_2.dae", "link_3.dae", "link_4.dae", "link_5.dae", "link_6.dae"]

LINK_ZEROS = [
    (0.0, 0.0,   0.495),
    (0.175, 0.0, 0.495),
    (0.175, 0.0, 1.395),
    (0.175, 0.0, 1.570),
    (1.135, 0.0, 1.570),
    (1.270, 0.0, 1.570)
]

def make_irb4600():

    link1 = DHLink(theta_offset=0.0,        d=0.495,    a=0.175,    alpha=-np.pi / 2,   offset=0.0,         qlim=[-3.141, 3.141])
    link2 = DHLink(theta_offset=-np.pi / 2, d=0.0,      a=0.900,    alpha=0.0,          offset=-np.pi/2,    qlim=[-3.141, 3.141])
    link3 = DHLink(theta_offset=0.0,        d=0.0,      a=0.175,    alpha=-np.pi / 2,   offset=0.0,         qlim=[-3.141, 3.141])
    link4 = DHLink(theta_offset=0.0,        d=0.960,    a=0.0,      alpha=np.pi / 2,    offset=0.0,         qlim=[-3.141, 3.141])
    link5 = DHLink(theta_offset=0.0,        d=0.0,      a=0.0,      alpha=-np.pi / 2,   offset=0.0,         qlim=[-3.141, 3.141])
    link6 = DHLink(theta_offset=0.0,        d=0.135,    a=0.0,      alpha=0.0,          offset=0.0,         qlim=[-3.141, 3.141])

    return DHRobot([link1, link2, link3, link4, link5, link6], name="IRB4600")


def load_meshes(robot):

    base_mesh = sg.Mesh(os.path.join(MESH_DIR, MESH_NAMES[0]))
    base_mesh.T = robot.base.A

    dh_zero = robot.fkine_all(np.zeros(6))

    for i in range(6):
        mesh = sg.Mesh(os.path.join(MESH_DIR, MESH_NAMES[i+1]))

        mesh_at_zero = np.eye(4)
        mesh_at_zero[:3, 3] = LINK_ZEROS[i]

        mesh.T = np.linalg.inv(dh_zero[i+1].A) @ mesh_at_zero

        robot.links[i].geometry = [mesh]

    robot.base_mesh = base_mesh
    return base_mesh


def main():
    env = Swift()
    env.launch(realtime=True)

    robot = make_irb4600()
    robot.q = np.zeros(6)

    load_meshes(robot)

    robot.base = SE3(0, 1, 0)
    robot.base_mesh.T = robot.base.A
    
    env.add(robot.base_mesh)
    env.add(robot)
    env.hold()

if __name__ == "__main__":
    main()