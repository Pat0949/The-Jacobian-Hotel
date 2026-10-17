import numpy as np
import spatialgeometry as sg
from spatialmath.base import skew


class EndEffectorRay:

    def __init__(self, robot, floor, env, max_range=0.5):
        self.robot = robot
        self.max_range = max_range

        # height of the top of the floor
        self.floor_z = floor.T[2, 3] + floor.scale[2] / 2

        # one red beam that always stays the same length
        self.beam = sg.Cylinder(radius=0.05, length=self.max_range, color=(1.0, 1.0, 1.0, 0.6))
        env.add(self.beam)
        self.update()

    def hit_floor(self):
        # the ray starts at the end effector and points along its z axis
        tr = self.robot.fkine(self.robot.q).A
        end_p = tr[:3, 3] + self.max_range * tr[:3, 2]

        # true when the end of the ray is at or below the floor
        return end_p[2] <= self.floor_z

    def update(self):
        # move the beam so it starts at the end effector
        tr = self.robot.fkine(self.robot.q).A
        beam_pose = tr.copy()
        beam_pose[:3, 3] = tr[:3, 3] + 0.5 * self.max_range * tr[:3, 2]
        self.beam.T = beam_pose

    def keep_above_floor(self):
        tr = self.robot.fkine(self.robot.q).A
        to_end = self.max_range * tr[:3, 2]
        end_z = (tr[:3, 3] + to_end)[2]

        # if the end of the ray is at or below the floor lift the end of the ray back up
        if end_z <= self.floor_z:
            lift = self.floor_z - end_z + 0.001

            # jacobian for the end of the ray and not just the end effector
            J = self.robot.jacob0(self.robot.q)
            J_end = J[:3, :] - skew(to_end) @ J[3:, :]
            dq = np.linalg.pinv(J_end) @ np.array([0.0, 0.0, lift])

            # keep every joint inside its limits
            self.robot.q = np.clip(self.robot.q + dq, self.robot.qlim[0], self.robot.qlim[1])