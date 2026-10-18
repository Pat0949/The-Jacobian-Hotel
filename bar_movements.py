import numpy as np
from scipy import linalg
from spatialmath import SE3
from roboticstoolbox import jtraj

GRASP = 0.22       # bottle base to the gripper (neck) [m]
MOUTH = 0.08       # gripper to the bottle's mouth [m]
CLEARANCE = 0.10   # ellipsoid radius around the TX2's links and held object [m]

def get_algebraic_dist(points, center_point, radii):
    return np.sum(((points - center_point) / radii) ** 2, axis=1)

def get_ellipsoids(robot, q, holding):
    ellipsoids = []
    joints = [T.t for T in robot.fkine_all(q)]
    for a, b in zip(joints[:-1], joints[1:]):
        length = np.linalg.norm(b - a)
        if length < 0.01:
            continue
        x = (b - a) / length
        y = np.cross([0, 0, 1] if abs(x[2]) < 0.9 else [1, 0, 0], x)
        y /= np.linalg.norm(y)
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = np.column_stack([x, y, np.cross(x, y)]), (a + b) / 2
        ellipsoids.append((T, [length / 2 + CLEARANCE, CLEARANCE, CLEARANCE]))
    if holding:
        T = np.eye(4)
        T[:3, 3] = joints[-1] - [0, 0, GRASP / 2]
        ellipsoids.append((T, [CLEARANCE, CLEARANCE, GRASP / 2 + CLEARANCE]))
    return ellipsoids

def robot_points(robot, q, holding):
    joints = [T.t for T in robot.fkine_all(q)]
    pts = [a + (b - a) * s for a, b in zip(joints[:-1], joints[1:]) for s in np.linspace(0, 1, 10)]
    if holding:
        pts += [joints[-1] - [0, 0, s] for s in np.linspace(0, GRASP, 8)]
    return np.array(pts)

def collides(robot, q, holding, points):
    for T, radii in get_ellipsoids(robot, q, holding):
        points_and_ones = linalg.inv(T) @ np.hstack((points, np.ones([points.shape[0], 1]))).T
        algebra_dist = get_algebraic_dist(np.transpose(points_and_ones)[:, :3], [0, 0, 0], radii)
        if np.size(np.where(algebra_dist < 1), 1) > 0:
            return True
    return False

def solve_ik_xyz(robot, xyz, q0):
    result = robot.ik_LM(SE3.Trans(xyz), q0=q0, mask=[1, 1, 1, 0, 0, 0])
    if not result[1]:
        print(f"{robot.name} could not reach {np.round(xyz, 3)}")
        return q0
    return result[0]

def hold(robot, obj, offset, tilt=0.0):
    obj.T = SE3.Trans(robot.fkine(robot.q).t) * SE3.Ry(tilt) * SE3(0, 0, -offset)

def tip(env, robot, obj, offset, angle):
    for f in list(np.linspace(0, 1, 40)) + [1] * 50 + list(np.linspace(1, 0, 40)):
        hold(robot, obj, offset, f * angle)
        env.step(0.02)

def move_arms(env, robots, targets, holding, steps=50, avoid=None):
    trajs = [jtraj(r.q, t if len(t) == 6 else solve_ik_xyz(r, t, r.q), steps).q for r, t in zip(robots, targets)]
    k = [0] * len(robots)
    for _ in range(4 * steps):
        for i, (r, traj) in enumerate(zip(robots, trajs)):
            if k[i] < steps - 1:
                if avoid and r is avoid[0]:
                    other = avoid[1]
                    if collides(r, traj[k[i] + 1], holding[id(r)], robot_points(other, other.q, holding[id(other)])):
                        continue
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
    def both(targets):
        move_arms(env, robots, targets, holding, avoid=(tx2, ur3e))
    def solo(target, steps=40):
        move_arms(env, robots, [ur3e.q, target], holding, steps, (tx2, ur3e))
    homes = [r.q.copy() for r in robots]
    starts = [SE3(b.T) for b in bottles]
    necks = [s.t + [0, 0, GRASP] for s in starts]
    lifted = [n + [0, 0, 0.10] for n in necks]
    tilt = np.radians(110)
    top = SE3(shaker.T).t + [0, 0, 0.10]
    pours = [top + [s * (MOUTH * np.sin(tilt) + 0.02), 0, 0.03 - MOUTH * np.cos(tilt)] for s in sides]
    # 1. grab bottles and take them to the shaker
    both(necks)
    for r, b in zip(robots, bottles):
        holding[id(r)] = (b, GRASP)
    both(lifted)
    both(pours)
    # 2. pour one at a time
    for r, b, s in zip(robots, bottles, sides):
        tip(env, r, b, GRASP, -s * tilt)
    # 3. put bottles back and go home
    both(lifted)
    both(necks)
    for r, s in zip(robots, starts):
        holding[id(r)][0].T = s
        holding[id(r)] = None
    both(homes)
    # 4. TX2 shakes the shaker and pours it into the glass
    start = SE3(shaker.T)
    grip = start.t + [0, 0, 0.08]
    up = grip + [0, 0, 0.15]
    solo(grip)
    holding[id(tx2)] = (shaker, 0.08)
    solo(up)
    for _ in range(2):
        solo(up + [0, 0, 0.10], steps=12)
        solo(up, steps=12)
    solo(SE3(glass.T).t + [0, 0, 0.45])
    tip(env, tx2, shaker, 0.08, tilt)
    solo(grip)
    shaker.T = start
    holding[id(tx2)] = None
    solo(homes[1])