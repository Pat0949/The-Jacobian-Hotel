from spatialmath import SE3
from roboticstoolbox import jtraj

POSITION_ONLY = [1, 1, 1, 0, 0, 0]


def move_to(robot, env, position, carry=None, grasp_height=0.0, steps=60):
    sol = robot.ikine_LM(SE3.Trans(position), q0=robot.q, mask=POSITION_ONLY)
    if not sol.success:
        print("Could not reach", position)
        return False
    for q in jtraj(robot.q, sol.q, steps).q:
        robot.q = q
        if carry is not None:
            carry.T = SE3.Trans(robot.fkine(q).t) * SE3(0, 0, -grasp_height)
        env.step(0.02)
    return True



def pick_Place(robot, env, bottle, place_pose, grasp_height=0.22):
    grasp = SE3(bottle.T).t + [0, 0, grasp_height]   
    place = place_pose.t + [0, 0, grasp_height]       

    home = robot.q.copy()
    move_to(robot, env, grasp)                                             # go to the bottle
    move_to(robot, env, place, carry=bottle, grasp_height=grasp_height)   # carry it to the counter
    bottle.T = place_pose                                                  # set it down upright
    for q in jtraj(robot.q, home, 60).q:                                   # back home
        robot.q = q
        env.step(0.02)