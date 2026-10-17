import numpy as np
import spatialgeometry as sg


class EndEffectorRay:

    def __init__(self, robot, floor, env, max_range=0.5):
        self.robot = robot
        self.max_range = max_range

        # height of the top of the floor
        self.floor_z = floor.T[2, 3] + floor.scale[2] / 2

        # one red beam that always stays the same length
        self.beam = sg.Cylinder(radius=0.005, length=self.max_range, color=(1.0, 0.0, 0.0, 1.0))
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