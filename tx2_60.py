import os
import shutil
import tempfile
import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3
import spatialgeometry as sg

# Absolute path to the real meshes/ folder next to this script (this is
# what lives in the git repo and what you submit).
_PROJECT_MESH_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "meshes")


def _stage_meshes_for_swift(mesh_dir):
    """
    Swift's local file server can fail to load meshes from a path that
    contains spaces (e.g. ".../Industrial Robotics/...") because the
    URL-encoded space isn't always decoded correctly before the file is
    opened. To work around this, copy the .stl files into the system temp
    folder (which has no spaces in the path for this user) once per run,
    and point the visual geometry at that copy instead. The real meshes/
    folder in the repo is untouched -- this only affects the live preview.
    """
    staged_dir = os.path.join(tempfile.gettempdir(), "tx2_60_meshes")
    os.makedirs(staged_dir, exist_ok=True)
    for fname in os.listdir(mesh_dir):
        if fname.lower().endswith(".stl"):
            shutil.copy2(os.path.join(mesh_dir, fname),
                         os.path.join(staged_dir, fname))
    return staged_dir


MESH_DIR = _stage_meshes_for_swift(_PROJECT_MESH_DIR)


class TX2_60(DHRobot):
    """
    Stäubli TX2-60 6-DOF industrial robot arm.

    DH parameters

        Joint | d      | a     | alpha    | range
        ------+--------+-------+----------+-------------------
          1   | 0.375  | 0.0   | -pi/2    | +-180 deg
          2   | 0.020  | 0.290 |  0       | +-127.5 deg
          3   | 0.0    | 0.0   |  pi/2    | +-142.5 deg
          4   | 0.310  | 0.0   | -pi/2    | +-270 deg
          5   | 0.0    | 0.0   |  pi/2    | +132.5/-121 deg
          6   | 0.070  | 0.0   |  0       | +-270 deg

    """

    def __init__(self):
        deg = np.pi / 180

        links = [
            RevoluteDH(d=0.375, a=0.0,   alpha=-np.pi / 2,
                       qlim=[-180 * deg, 180 * deg],
                       geometry=[sg.Mesh(filename=os.path.join(MESH_DIR, "link_1.stl"),
                                          color=[0.9, 0.8, 0.1])]),
            RevoluteDH(d=0.020, a=0.290, alpha=0.0,
                       qlim=[-127.5 * deg, 127.5 * deg],
                       geometry=[sg.Mesh(filename=os.path.join(MESH_DIR, "link_2.stl"),
                                          color=[0.9, 0.8, 0.1])]),
            RevoluteDH(d=0.0,   a=0.0,   alpha=np.pi / 2,
                       qlim=[-142.5 * deg, 142.5 * deg],
                       geometry=[sg.Mesh(filename=os.path.join(MESH_DIR, "link_3.stl"),
                                          color=[0.9, 0.8, 0.1])]),
            RevoluteDH(d=0.310, a=0.0,   alpha=-np.pi / 2,
                       qlim=[-270 * deg, 270 * deg],
                       geometry=[sg.Mesh(filename=os.path.join(MESH_DIR, "link_4.stl"),
                                          color=[0.7, 0.7, 0.7])]),
            RevoluteDH(d=0.0,   a=0.0,   alpha=np.pi / 2,
                       qlim=[-121 * deg, 132.5 * deg],
                       geometry=[sg.Mesh(filename=os.path.join(MESH_DIR, "link_5.stl"),
                                          color=[0.7, 0.7, 0.7])]),
            RevoluteDH(d=0.070, a=0.0,   alpha=0.0,
                       qlim=[-270 * deg, 270 * deg],
                       geometry=[sg.Mesh(filename=os.path.join(MESH_DIR, "link_6.stl"),
                                          color=[0.7, 0.7, 0.7])]),
        ]

        super().__init__(
            links,
            name="Staubli_TX2_60",
            manufacturer="Staubli",
        )

        self.qz = np.zeros(6)
        self.addconfiguration("qz", self.qz)

        self.q_extended = np.array([0, 0, 90 * deg, 0, 0, 0])
        self.addconfiguration("q_extended", self.q_extended)

        self.qr = np.array([0, -30 * deg, 90 * deg, 0, 60 * deg, 0])
        self.addconfiguration("qr", self.qr)


if __name__ == "__main__":
    robot = TX2_60()
    print(robot)

    T = robot.fkine(robot.q_extended)
    horiz_reach = np.hypot(T.t[0], T.t[1])
    print(f"\nEnd-effector pose at 'q_extended': \n{T}")
    print(f"Horizontal reach: {horiz_reach * 1000:.1f} mm "
          f"(datasheet quotes 670 mm)")

    from roboticstoolbox.backends.swift import Swift

    env = Swift()
    env.launch(realtime=True)
    env.add(robot)
    robot.q = robot.qr
    env.step()
    env.hold()