import swift
from spatialmath import SE3
from ir_support_extra_parts.parts import part_mesh

env = swift.Swift()
env.launch(realtime=True)

table = part_mesh("RobotTable")
#table.scale = [1.6, 1.6, 0.4]
table.T = SE3(0.3, 0, 0)
env.add(table)

env.step()