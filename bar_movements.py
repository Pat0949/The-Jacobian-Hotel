import numpy as np
from spatialmath import SE3
from roboticstoolbox import jtraj

GRASP = 0.22        # bottle base to where the gripper holds the neck [m]
LIFT = 0.10         # lift off the shelf [m]
POUR_SIDE = 0.10    # each robot pours from this far either side of the shaker [m]
POUR_ABOVE = 0.08   # gripper height above the shaker top while pouring [m]


def ik(robot, xyz):
    sol = robot.ikine_LM(SE3.Trans(xyz), q0=robot.q, mask=[1, 1, 1, 0, 0, 0])
    if not sol.success:
        print(f"{robot.name} could not reach {np.round(xyz, 3)}")
        return robot.q
    return sol.q


def hold(robot, bottle, tilt=0.0):
    # Bottle hangs from the gripper by its neck, tipped about y by 'tilt'
    bottle.T = SE3.Trans(robot.fkine(robot.q).t) * SE3.Ry(tilt) * SE3(0, 0, -GRASP)


def move(env, moves, steps=50):
    # Move several robots at once. Each move is (robot, target, bottle or None),
    # where target is an x, y, z position or a full set of joint angles
    paths = [jtraj(r.q, ik(r, t) if len(t) == 3 else t, steps).q for r, t, _ in moves]
    for k in range(steps):
        for (r, _, b), path in zip(moves, paths):
            r.q = path[k]
            if b is not None:
                hold(r, b)
        env.step(0.02)


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
        move(env, list(zip(robots, targets, bottles if carrying else [None, None])))

    both(lifted, False)           # reach over the bottles
    both(necks, False)            # grab
    both(lifted, True)            # lift
    both(pours, True)             # carry to the shaker

    for s in list(np.linspace(0, 1, 40)) + [1] * 50 + list(np.linspace(1, 0, 40)):    # pour
        for r, b, side in zip(robots, bottles, sides):
            hold(r, b, tilt=-side * s * np.radians(110))
        env.step(0.02)

    both(lifted, True)            # back over the shelf
    both(necks, True)             # lower
    for b, s in zip(bottles, starts):
        b.T = s                   # release, upright
    both(lifted, False)
    both(homes, False)            # home