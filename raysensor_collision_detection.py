import numpy as np
from itertools import combinations
import spatialgeometry as sg
from spatialmath.base import skew
from ir_support import RectangularPrism, line_plane_intersection
 
class EndEffectorRay:
 
    def __init__(self, robot, floor, env, max_range=0.5):

        self.robot = robot
        self.max_range = max_range
 
        # height of the top of the floor
        self.floor_z = floor.T[2, 3] + floor.scale[2] / 2
        lwh = floor.scale.tolist()
        centre = floor.T[:3, 3].tolist()
        self.vertices, self.faces, self.face_normals = RectangularPrism(lwh[0], lwh[1], lwh[2], center=centre).get_data()

        #radius of ray
        r = 0.05
 
        # ray
        self.beam = sg.Cylinder(radius=r, length=self.max_range, color=(1.0, 1.0, 1.0, 0.6))
        env.add(self.beam)
 
        # four lines along the ray as wide as the beam
        self.line_offsets = np.array([[r, r, 0], [-r, r, 0], [-r, -r, 0], [r, -r, 0]])
 
        self.update()
 
    def get_ray_lines(self):
        tr = self.robot.fkine(self.robot.q).A
        lines = []
        for offset in self.line_offsets:
            start_p = tr[:3, 3] + tr[:3, :3] @ offset
            end_p = start_p + self.max_range * tr[:3, 2]
            lines.append((start_p, end_p))
        return lines
    
    def hit_floor(self):
        for start_p, end_p in self.get_ray_lines():
            if is_collision(start_p, end_p, self.faces, self.vertices, self.face_normals):
                return True
        return False
 
    def update(self):
        tr = self.robot.fkine(self.robot.q).A
        beam_pose = tr.copy()
        beam_pose[:3, 3] = tr[:3, 3] + 0.5 * self.max_range * tr[:3, 2]
        self.beam.T = beam_pose
 
    def keep_above_floor(self):
        tr = self.robot.fkine(self.robot.q).A
        lines = self.get_ray_lines()
 
        # if any of the 4 lines has hit the floor lift the ray back up
        if self.hit_floor():
            # the lowest line end is the one furthest below the floor
            ends_z = [end_p[2] for start_p, end_p in lines]
            lowest = int(np.argmin(ends_z))
            lift = max(self.floor_z - ends_z[lowest] + 0.001, 0.0)
 
            # vector from the end effector to the end of that line
            to_end = tr[:3, :3] @ self.line_offsets[lowest] + self.max_range * tr[:3, 2]
 
            # jacobian for the end of that line and not just the end effector
            J = self.robot.jacob0(self.robot.q)
            J_end = J[:3, :] - skew(to_end) @ J[3:, :]
            dq = np.linalg.pinv(J_end) @ np.array([0.0, 0.0, lift])
 
            # keep every joint inside its limits
            self.robot.q = np.clip(self.robot.q + dq, self.robot.qlim[0], self.robot.qlim[1])
 
def is_intersection_point_inside_triangle(intersect_p, triangle_verts):
    u = triangle_verts[1, :] - triangle_verts[0, :]
    v = triangle_verts[2, :] - triangle_verts[0, :]
 
    uu = np.dot(u, u)
    uv = np.dot(u, v)
    vv = np.dot(v, v)
 
    w = intersect_p - triangle_verts[0, :]
    wu = np.dot(w, u)
    wv = np.dot(w, v)
 
    D = uv * uv - uu * vv
 
    # Get and test parametric coords (s and t)
    s = (uv * wv - vv * wu) / D
    if s < 0.0 or s > 1.0:  # intersect_p is outside the triangle
        return 0
 
    t = (uv * wu - uu * wv) / D
    if t < 0.0 or (s + t) > 1.0:  # intersect_p is outside the triangle
        return False
 
    return True  # intersect_p is in the triangle
 
 
def is_collision(start_p, end_p, faces, vertex, face_normals):
    # Go through each triangle face
    for j, face in enumerate(faces):
        vert_on_plane = vertex[face][0]
        intersect_p, check = line_plane_intersection(face_normals[j],
                                                    vert_on_plane,
                                                    start_p,
                                                    end_p)
        # list of all triangle combinations in a face
        triangle_list = np.array(list(combinations(face, 3)), dtype=int)
        if check == 1:
            for triangle in triangle_list:
                if is_intersection_point_inside_triangle(intersect_p, vertex[triangle]):
                    return True
    return False