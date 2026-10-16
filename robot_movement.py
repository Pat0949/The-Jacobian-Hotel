from roboticstoolbox import jtraj

def solve_ik(robot, target_pose, JA0):

    # Levenberg-Marquardt algorithm to get from q0 (where ever the arm currently is) to target pose
    result = robot.ik_LM(target_pose, q0 = JA0)

    # returns all the necesary joint angles the .ik_LM found
    return result[0]

def move_arm(robot, q_start, q_end, env, steps=80, attached_object=None, object_offset=None):

    #create a matrix of all 80 points between start and end pose
    traj = jtraj(q_start, q_end, steps).q

    #for each 80 poses move from previous pose to the next
    for q in traj:
        robot.q = q

        #if attached object, forward kinematics and offset it
        if attached_object is not None:
            robot_pose = robot.fkine(q)
            attached_object.T = (robot_pose * object_offset).A
            
        #pause time to redraw the scene
        env.step(0.02)

    return traj

