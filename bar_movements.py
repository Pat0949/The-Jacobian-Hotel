
import numpy as np
from scipy import linalg
from spatialmath import SE3
from roboticstoolbox import jtraj

GRASP = 0.22        # bottle base to where the gripper holds the neck [m]
LIFT = 0.10         # lift off the shelf [m]
POUR_SIDE = 0.08    # each robot pours from this far either side of the shaker [m]
POUR_ABOVE = 0.08   # gripper height above the shaker top while pouring [m]
CLEARANCE = 0.10    # ellipsoid radius around each TX2 link and bottle [m]


#collision checking

def get_algebraic_dist(points, center_point, radii):
    """Algebraic distance of points from an ellipsoid (from Lab 6): < 1 means inside"""
    return np.sum(((points - center_point) / radii) ** 2, axis=1)


def get_ellipsoids(robot, q, carrying_bottle=False):
    """Transform T and radii of an ellipsoid around each link (like EllipsoidRobot), plus the carried bottle"""
    ellipsoids = []
    joints = [T.t for T in robot.fkine_all(q)]
    for a, b in zip(joints[:-1], joints[1:]):
        length = np.linalg.norm(b - a)
        if length < 0.01:                                    # skip zero-length links
            continue
        x = (b - a) / length                                 # long axis along the link
        helper = [0, 0, 1] if abs(x[2]) < 0.9 else [1, 0, 0]
        y = np.cross(helper, x)
        y /= np.linalg.norm(y)
        z = np.cross(x, y)

        T = np.eye(4)
        T[:3, :3] = np.column_stack([x, y, z])               # ellipsoid orientation
        T[:3, 3] = (a + b) / 2                               # ellipsoid centre: middle of the link
        ellipsoids.append((T, [length / 2 + CLEARANCE, CLEARANCE, CLEARANCE]))

    if carrying_bottle:
        neck = robot.fkine(q).t
        T = np.eye(4)
        T[:3, 3] = neck + [0, 0, (0.10 - GRASP) / 2]          # centre of the upright bottle
        ellipsoids.append((T, [CLEARANCE, CLEARANCE, (GRASP + 0.10) / 2 + CLEARANCE / 2]))
    return ellipsoids


def link_points(robot, q, n=10):
    """Points spread along every link: the robot as a point cloud (like the cube points in Lab 6)"""
    joints = [T.t for T in robot.fkine_all(q)]
    return np.array([a + (b - a) * s for a, b in zip(joints[:-1], joints[1:]) for s in np.linspace(0, 1, n)])


def bottle_points(robot, q, n=8):
    """Points along an upright bottle held at the gripper"""
    neck = robot.fkine(q).t
    return np.array([neck + [0, 0, s] for s in np.linspace(-GRASP, 0.10, n)])


def body_points(robot, q, bottle=None):
    """The robot's links, plus its bottle if it's carrying one"""
    pts = link_points(robot, q)
    return pts if bottle is None else np.vstack([pts, bottle_points(robot, q)])


def collides(robot, q, carrying_bottle, other_points):
    """Lab 6 Q2.9-2.10: transform the points into each ellipsoid's frame and check the algebraic distance"""
    for T, radii in get_ellipsoids(robot, q, carrying_bottle):
        points_and_ones = linalg.inv(T) @ np.hstack((other_points, np.ones([other_points.shape[0], 1]))).T
        updated_points = np.transpose(points_and_ones)[:, :3]
        algebra_dist = get_algebraic_dist(updated_points, [0, 0, 0], radii)
        points_inside_indices = np.where(algebra_dist < 1)
        if np.size(points_inside_indices, 1) > 0:
            return True
    return False


# movement 

def ik(robot, xyz):
    sol = robot.ikine_LM(SE3.Trans(xyz), q0=robot.q, mask=[1, 1, 1, 0, 0, 0])
    if not sol.success:
        print(f"{robot.name} could not reach {np.round(xyz, 3)}")
        return robot.q
    return sol.q


def hold(robot, bottle, tilt=0.0):
    # Bottle hangs from the gripper by its neck, tipped about y by 'tilt'
    bottle.T = SE3.Trans(robot.fkine(robot.q).t) * SE3.Ry(tilt) * SE3(0, 0, -GRASP)


def move(env, moves, steps=50, avoid=None):
    # Move several robots at once. Each move is (robot, target, bottle or None),
    # where target is an x, y, z position or a full set of joint angles.
    # avoid=(robot, other): 'robot' pauses whenever its next step (arm or bottle) would collide with 'other'
    paths = [jtraj(r.q, ik(r, t) if len(t) == 3 else t, steps).q for r, t, _ in moves]
    carried = {id(r): b for r, _, b in moves}
    idx = [0] * len(moves)

    for _ in range(steps * 4):                                # extra frames allow for waiting
        for m, ((r, _, b), path) in enumerate(zip(moves, paths)):
            if idx[m] < steps - 1:
                if avoid and r is avoid[0]:
                    mover, other = avoid
                    other_points = body_points(other, other.q, carried.get(id(other)))
                    if collides(mover, path[idx[m] + 1], carried.get(id(mover)) is not None, other_points):
                        continue                              # would collide: wait this frame
                idx[m] += 1
                r.q = path[idx[m]]
            if b is not None:
                hold(r, b)
        env.step(0.02)
        if all(i == steps - 1 for i in idx):
            return
    print("Move stopped early: path stayed blocked")


# tasks

def pick_place(env, robot, bottle, place):
    # One robot carries one bottle to 'place'
    home = robot.q.copy()
    move(env, [(robot, SE3(bottle.T).t + [0, 0, GRASP], None)])
    move(env, [(robot, place.t + [0, 0, GRASP], bottle)])
    bottle.T = place
    move(env, [(robot, home, None)])


def make_cocktail(env, scene):
    robots = [scene["ur3e"], scene["tx2"]]
    bottles = [scene["beer"], scene["wine"]]
    sides = [+1, -1]                                         # which side of the shaker each pours from
    shaker_top = SE3(scene["shaker"].T).t + [0, 0, 0.10]

    homes = [r.q.copy() for r in robots]
    starts = [SE3(b.T) for b in bottles]
    necks = [s.t + [0, 0, GRASP] for s in starts]
    lifted = [n + [0, 0, LIFT] for n in necks]
    pours = [shaker_top + [side * POUR_SIDE, 0, POUR_ABOVE] for side in sides]

    def both(targets, carrying):
        # Move both robots together; the TX2 gives way to the UR3e
        move(env, list(zip(robots, targets, bottles if carrying else [None, None])),
             avoid=(scene["tx2"], scene["ur3e"]))

    both(lifted, False)           # reach over the bottles
    both(necks, False)            # grab
    both(lifted, True)            # lift
    both(pours, True)             # carry to the shaker

    for r, b, side in zip(robots, bottles, sides):           # pour one at a time so the bottles don't clash
        for s in list(np.linspace(0, 1, 40)) + [1] * 50 + list(np.linspace(1, 0, 40)):
            hold(r, b, tilt=-side * s * np.radians(110))
            env.step(0.02)

    both(lifted, True)            # back over the shelf
    both(necks, True)             # lower
    for b, s in zip(bottles, starts):
        b.T = s                   # release, upright
    both(lifted, False)
    both(homes, False)            # home