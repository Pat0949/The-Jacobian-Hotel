import swift
import spatialgeometry as sg
from spatialmath import SE3
import numpy as np
from math import pi

import scene
from robot_movement import solve_ik, move_arm

def move_IRB():
    env = swift.Swift()
    q_home = np.zeros(6)
    target_pose = SE3(1.2, 1.3, 0.8) * SE3.Rx(pi)
    q_target = solve_ik(scene["IRB_robot"], target_pose, q_home)

    move_arm(scene["IRB_robot"], q_home, q_target, env, steps=80)
    move_arm(scene["IRB_robot"], q_target, q_home, env, steps=80)