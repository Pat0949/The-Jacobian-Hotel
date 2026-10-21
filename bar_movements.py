import numpy as np
from scipy import linalg
from spatialmath import SE3
from spatialmath.base import transl
from roboticstoolbox import jtraj, trapezoidal
from ir_support import EllipsoidRobot

GRASP = 0.22       # bottle base to the gripper [m]
CLEARANCE = 0.12   # ellipsoid padding around the TX2 [m]

def get_algebraic_dist(points, center_point, radii):
    return np.sum(((points - center_point) / radii) ** 2, axis=1)

def get_link_poses(robot, q):
    return robot.fkine_all(q).A

def ellipsoid_for_robot_links(robot, q, holding):
    if not hasattr(robot, "ellipsoid_robot"):
        robot.ellipsoid_robot = EllipsoidRobot(robot, default_height=CLEARANCE, default_width=CLEARANCE)
    robot.ellipsoid_robot.ellipsoid_for_robot_links(q)
    ellipsoids = [robot.ellipsoid_robot.get_ellipsoid_transform_and_radii(i) for i in range(len(robot.ellipsoid_robot.ellipsoid_matrices))]
    transforms, radii_list = [T for T, _ in ellipsoids], [np.maximum(radii, CLEARANCE) for _, radii in ellipsoids]
    if holding:
        transforms.append(transl(*(robot.fkine(q).t - [0, 0, GRASP / 2])))
        radii_list.append([CLEARANCE, CLEARANCE, GRASP / 2 + CLEARANCE])
    return transforms, radii_list

def robot_points(robot, q, holding):
    p = [T[:3, 3] for T in get_link_poses(robot, q)]
    points = [a + (b - a) * s for a, b in zip(p[:-1], p[1:]) for s in np.linspace(0, 1, 10)]
    return np.array(points + ([p[-1] - [0, 0, s] for s in np.linspace(0, GRASP, 8)] if holding else []))

def check_points(transforms, radii_list, points):
    for T, radii in zip(transforms, radii_list):
        points_and_ones = linalg.inv(T) @ np.hstack((points, np.ones([points.shape[0], 1]))).T
        algebra_dist = get_algebraic_dist(np.transpose(points_and_ones)[:, :3], [0, 0, 0], radii)
        if np.size(np.where(algebra_dist < 1), 1) > 0:
            return True
    return False

def solve_ik_xyz(robot, xyz, q0):
    result = robot.ik_LM(SE3.Trans(xyz), q0=q0, mask=[1, 1, 1, 0, 0, 0])
    if not result[1]:
        print(f"{robot.name} could not reach {np.round(xyz, 3)}")
    return result[0] if result[1] else q0

def rmrc(robot, q0, end_p, steps=50, delta_t=0.05):
    epsilon, W = 0.01, np.diag([1, 1, 1, 0.1, 0.1, 0.1])
    T = robot.fkine(q0).A
    s = trapezoidal(0, 1, steps).q
    x, Rd = np.outer(T[:3, 3], 1 - s) + np.outer(end_p, s), T[:3, :3]
    q_matrix = [np.array(q0, dtype=float)]
    for i in range(steps - 1):
        T = robot.fkine(q_matrix[i]).A
        S = (Rd - T[:3, :3]) @ T[:3, :3].T / delta_t
        xdot = W @ np.hstack(((x[:, i + 1] - T[:3, 3]) / delta_t, [S[2, 1], S[0, 2], S[1, 0]]))
        J = robot.jacob0(q_matrix[i])
        m = np.sqrt(linalg.det(J @ J.T))
        m_lambda = (1 - m / epsilon) * 0.05 if m < epsilon else 0
        qdot = np.clip(linalg.inv(J.T @ J + m_lambda * np.eye(robot.n)) @ J.T @ xdot, -np.pi, np.pi)
        q_next = q_matrix[i] + delta_t * qdot
        qdot[(q_next < robot.qlim[0]) | (q_next > robot.qlim[1])] = 0
        q_matrix.append(q_matrix[i] + delta_t * qdot)
    return np.array(q_matrix)

def hold(robot, obj, offset, tilt=0.0):
    obj.T = SE3.Trans(robot.fkine(robot.q).t) * SE3.Ry(tilt) * SE3(0, 0, -offset)

def tip(env, robot, obj, offset, angle):
    for f in list(np.linspace(0, 1, 40)) + [1] * 50 + list(np.linspace(1, 0, 40)):
        hold(robot, obj, offset, f * angle)
        env.step(0.02)

def move_arms(env, robots, targets, holding, steps=50, avoid=None, line=False):
    trajs = [jtraj(r.q, t, steps).q if len(t) == 6 else rmrc(r, r.q, t, steps) if line
             else jtraj(r.q, solve_ik_xyz(r, t, r.q), steps).q for r, t in zip(robots, targets)]
    k = [0] * len(robots)
    for _ in range(4 * steps):
        for i, (r, traj) in enumerate(zip(robots, trajs)):
            if k[i] < steps - 1 and not (avoid and r is avoid[0] and check_points(
                    *ellipsoid_for_robot_links(r, traj[k[i] + 1], holding[id(r)]), robot_points(avoid[1], avoid[1].q, holding[id(avoid[1])]))):
                k[i] += 1
                r.q = traj[k[i]]
            if holding[id(r)]:
                hold(r, *holding[id(r)])
        env.step(0.02)
        if min(k) == steps - 1:
            return
    print("Move stopped early: path stayed blocked")

def pick_place(env, robot, bottle, place):
    home, holding = robot.q.copy(), {id(robot): None}
    move_arms(env, [robot], [SE3(bottle.T).t + [0, 0, GRASP]], holding)
    holding[id(robot)] = (bottle, GRASP)
    move_arms(env, [robot], [place.t + [0, 0, GRASP]], holding)
    bottle.T, holding[id(robot)] = place, None
    move_arms(env, [robot], [home], holding)

def make_cocktail(env, scene):
    ur3e, tx2, shaker, glass = scene["ur3e"], scene["tx2"], scene["shaker"], scene["glass"]
    robots, bottles, sides = [ur3e, tx2], [scene["beer"], scene["wine"]], [+1, -1]
    holding = {id(ur3e): None, id(tx2): None}
    both = lambda targets, line=False: move_arms(env, robots, targets, holding, avoid=(tx2, ur3e), line=line)
    solo = lambda target, steps=40, line=False: move_arms(env, robots, [ur3e.q, target], holding, steps, (tx2, ur3e), line)
    homes, starts = [r.q.copy() for r in robots], [SE3(b.T) for b in bottles]
    necks = [s.t + [0, 0, GRASP] for s in starts]
    lifted = [n + [0, 0, 0.10] for n in necks]
    tilt, top, mouth = np.radians(110), SE3(shaker.T).t + [0, 0, 0.10], 0.08
    pours = [top + [s * (mouth * np.sin(tilt) + 0.04), 0, 0.03 - mouth * np.cos(tilt)] for s in sides]
    # 1. grab bottles and take them to the shaker
    both(necks)
    holding.update({id(r): (b, GRASP) for r, b in zip(robots, bottles)})
    both(lifted, line=True)
    both(pours)
    # 2. pour one at a time
    for r, b, s in zip(robots, bottles, sides):
        tip(env, r, b, GRASP, -s * tilt)
    # 3. put bottles back and go home
    both(lifted)
    both(necks, line=True)
    for b, s in zip(bottles, starts):
        b.T = s
    holding.update({id(r): None for r in robots})
    both(homes)
    # 4. TX2 shakes the shaker and pours it into the glass
    start = SE3(shaker.T)
    grip = start.t + [0, 0, 0.08]
    up = grip + [0, 0, 0.15]
    solo(grip)
    holding[id(tx2)] = (shaker, 0.08)
    solo(up, line=True)
    for _ in range(2):
        solo(up + [0, 0, 0.15], steps=12, line=True)
        solo(up, steps=12, line=True)
    solo(SE3(glass.T).t + [0, 0, 0.45])
    tip(env, tx2, shaker, 0.08, -tilt)
    solo(up)
    solo(grip, line=True)
    shaker.T, holding[id(tx2)] = start, None
    solo(homes[1])