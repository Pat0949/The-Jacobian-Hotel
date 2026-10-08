import os
from math import pi
from pathlib import Path
 
import numpy as np
import swift
from spatialmath import SE3
from roboticstoolbox import ERobot, jtraj
from xacrodoc import packages
 
 
def find_repo_root(start):
    # Walk up the folders until one containing abb_irb4600_support is found
    for folder in [start, *start.parents]:
        if (folder / "abb_irb4600_support").is_dir():
            return folder
    raise FileNotFoundError("Could not find the abb_irb4600_support folder above this script")
 
 
REPO_ROOT = find_repo_root(Path(__file__).resolve().parent)
PACKAGE_PATH = REPO_ROOT / "abb_irb4600_support"
XACRO_PATH = PACKAGE_PATH / "urdf" / "irb4600_60_205.xacro"
 
# Help the xacro find its meshes
packages.update_package_cache({"abb_irb4600_support": str(PACKAGE_PATH)})
 
 
class IRB4600(ERobot):
    def __init__(self):
        links, name, urdf_string, urdf_filepath = self.URDF_read(str(XACRO_PATH))
        super().__init__(
            links,
            name=name,
            manufacturer="ABB",
            urdf_string=urdf_string,
            urdf_filepath=urdf_filepath,
        )
 
 
def main():
    # Check the files exist before doing anything else
    print("Xacro file exists", os.path.isfile(XACRO_PATH))
    print("Package folder exists", os.path.isdir(PACKAGE_PATH))
 
    # Build the robot
    robot = IRB4600()
    robot.base = SE3(0, 0, 0)
    robot.q = np.zeros(robot.n)
    print("Robot loaded with", robot.n, "joints")
    print(robot)
 
    # Launch Swift and add the robot
    env = swift.Swift()
    env.launch(realtime=True)
    env.add(robot)
    env.step()
    input("Robot loaded. Press Enter to start moving it\n")
 
    # A few simple joint moves
    q_home = np.zeros(robot.n)
    q_move1 = np.array([0, -pi / 4, pi / 4, 0, 0, 0])
    q_move2 = np.array([pi / 4, 0, -pi / 2, pi / 4, 0, 0])
 
    for q_start, q_end in [(q_home, q_move1), (q_move1, q_move2), (q_move2, q_home)]:
        traj = jtraj(q_start, q_end, 80).q
        for q in traj:
            robot.q = q
            env.step(0.02)
 
    input("Done. Press Enter to exit\n")
    env.close()
 
 
if __name__ == "__main__":
    main()