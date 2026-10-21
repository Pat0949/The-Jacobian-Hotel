import numpy as np
from ir_support import EllipsoidRobot
from spatialmath import SE3
from tx2_60 import TX2_60

tx2 = TX2_60()
tx2.base = SE3(0.4, -1.25, 0.9)
ellipsoid_robot = EllipsoidRobot(tx2, default_height=0.12, default_width=0.12)
ellipsoid_robot.ellipsoid_for_robot_links(tx2.q)
for i in range(len(ellipsoid_robot.ellipsoid_matrices)):
    T, radii = ellipsoid_robot.get_ellipsoid_transform_and_radii(i)
    print(i, np.round(T[:3, 3], 3), np.round(radii, 3))