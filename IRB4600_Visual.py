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

class IRB4600(DHRobot):

    def __init__(self, base=SE3()):

        link1 = DHLink(theta_offset=0.0,        d=0.495,    a=0.175,    alpha=-np.pi / 2,   offset=0.0,         qlim=[-3.141, 3.141])
        link2 = DHLink(theta_offset=-np.pi / 2, d=0.0,      a=0.900,    alpha=0.0,          offset=-np.pi/2,    qlim=[-3.141, 3.141])
        link3 = DHLink(theta_offset=0.0,        d=0.0,      a=0.175,    alpha=-np.pi / 2,   offset=0.0,         qlim=[-3.141, 3.141])
        link4 = DHLink(theta_offset=0.0,        d=0.960,    a=0.0,      alpha=np.pi / 2,    offset=0.0,         qlim=[-3.141, 3.141])
        link5 = DHLink(theta_offset=0.0,        d=0.0,      a=0.0,      alpha=-np.pi / 2,   offset=0.0,         qlim=[-3.141, 3.141])
        link6 = DHLink(theta_offset=0.0,        d=0.135,    a=0.0,      alpha=0.0,          offset=0.0,         qlim=[-3.141, 3.141])

        super().__init__([link1, link2, link3, link4, link5, link6], name="IRB4600")
        self.q = np.zeros(6)
        self.load_meshes()

        self.base = base
        self.base_mesh.T = self.base.A


    def load_meshes(self):

        base_mesh = sg.Mesh(os.path.join(MESH_DIR, MESH_NAMES[0]), pose=self.base)
        base_mesh.T = self.base.A

        dh_zero = self.fkine_all(np.zeros(6))

        for i in range(6):
            mesh_at_zero = SE3(*LINK_ZEROS[i])

            mesh_at_zero = np.eye(4)
            mesh_at_zero[:3, 3] = LINK_ZEROS[i]

            mesh = sg.Mesh(os.path.join(MESH_DIR, MESH_NAMES[i+1]), pose=dh_zero[i+1].inv() * mesh_at_zero)

            self.links[i].geometry = [mesh]

        self.base_mesh = base_mesh

    def add_to_env(self, env):
        env.add(self.base_mesh)
        env.add(self)
