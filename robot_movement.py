from roboticstoolbox import jtraj

def solve_ik(robot, target_pose, JA0):

    # Levenberg-Marquardt algorithm to get from q0 (where ever the arm currently is) to target pose
    result = robot.ik_LM(target_pose, q0 = JA0)

    # returns all the necesary joint angles the .ik_LM found
    return result[0]

def move_arm(robot, q_start, q_end, env, steps=80, attached_object=None, object_offset=None, ray=None):

    traj = jtraj(q_start, q_end, steps).q
    last_q = q_start

    for q in traj:
        robot.q = q

        if ray is not None and ray.hit_floor():
            print("The end of the ray has hit the floor. Stopping the arm.")
            robot.q = last_q
            ray.update()
            env.step(0.02)
            break
        last_q = q

        if attached_object is not None:
            robot_pose = robot.fkine(q)
            attached_object.T = (robot_pose * object_offset).A

        if ray is not None:
            ray.update()

        env.step(0.02)

    return traj
