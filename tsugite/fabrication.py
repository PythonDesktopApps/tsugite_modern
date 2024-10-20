import math
import os

import numpy as np

from utils import *


# noinspection PyAttributeOutsideInit
class MillVertex:
    def __init__(self, pt: ArrayLike,
                 is_traversing: bool = False,
                 is_arc: bool = False,
                 arc_ctr: ArrayLike = np.array([0, 0, 0])) -> None:

        self.pt = np.array(pt)
        self.x = pt[0]
        self.y = pt[1]
        self.z = pt[2]
        self.is_arc = is_arc
        self.arc_ctr = np.array(arc_ctr)
        self.is_traversing = is_traversing  # gcode_mode G0 (max milling_speed) (otherwise G1)

    def scale_and_swap(self, ax, direction: Direction, ratio, real_tim_dims, coords: list[int], d: int) -> None:
        # swap
        xyz = [ratio * self.x, ratio * self.y, ratio * self.z]
        if ax == 2: xyz[1] = -xyz[1]
        xyz = xyz[coords[0]], xyz[coords[1]], xyz[coords[2]]
        self.x, self.y, self.z = xyz[0], xyz[1], xyz[2]
        # move z down, flip if component b
        self.z = -(2 * direction - 1) * self.z - 0.5 * real_tim_dims[ax]
        self.y = -(2 * direction - 1) * self.y
        self.pt = np.array([self.x, self.y, self.z])
        self.pos = np.array([self.x, self.y, self.z], dtype=np.float64)
        self.xstr = str(round(self.x, d))
        self.ystr = str(round(self.y, d))
        self.zstr = str(round(self.z, d))
        ##
        if self.is_arc:
            self.arc_ctr = [ratio * self.arc_ctr[0], ratio * self.arc_ctr[1],
                            ratio * self.arc_ctr[2]]  # ratio*self.arc_ctr
            if ax == 2: self.arc_ctr[1] = -self.arc_ctr[1]
            self.arc_ctr = [self.arc_ctr[coords[0]], self.arc_ctr[coords[1]], self.arc_ctr[coords[2]]]
            self.arc_ctr[2] = -(2 * direction - 1) * self.arc_ctr[2] - 0.5 * real_tim_dims[ax]
            self.arc_ctr[1] = -(2 * direction - 1) * self.arc_ctr[1]
            self.arc_ctr = np.array(self.arc_ctr)

    def rotate(self, ang: ArrayLike, d: int) -> None:
        self.pt = np.array([self.x, self.y, self.z])
        self.pt = rotate_vector_around_axis(self.pt, [0, 0, 1], ang)
        self.x = self.pt[0]
        self.y = self.pt[1]
        self.z = self.pt[2]
        self.pos = np.array([self.x, self.y, self.z], dtype=np.float64)
        self.xstr = str(round(self.x, d))
        self.ystr = str(round(self.y, d))
        self.zstr = str(round(self.z, d))
        ##
        if self.is_arc:
            self.arc_ctr = rotate_vector_around_axis(self.arc_ctr, [0, 0, 1], ang)
            self.arc_ctr = np.array(self.arc_ctr)


def angle_between(vector_1: ArrayLike, vector_2: ArrayLike, normal_vector: list = []) -> DegreeArray:
    unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
    unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
    dot_product = np.dot(unit_vector_1, unit_vector_2)
    angle = np.arccos(dot_product)
    cross = np.cross(unit_vector_1, unit_vector_2)
    if len(normal_vector) > 0 and np.dot(normal_vector, cross) < 0: angle = -angle
    return angle


def rotate_vector_around_axis(vec: ArrayLike = [3, 5, 0],
                              axis: list = [4, 4, 1],
                              theta: float = 1.2) -> Any:  # np.dot can return a scalar or an array
    axis = np.asarray(axis)
    axis = axis / math.sqrt(np.dot(axis, axis))
    a = math.cos(theta / 2.0)
    b, c, d = -axis * math.sin(theta / 2.0)
    aa, bb, cc, dd = a * a, b * b, c * c, d * d
    bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
    mat = np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                    [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                    [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])
    rotated_vec = np.dot(mat, vec)
    return rotated_vec


def is_connected_arc(mv0: MillVertex, mv1: MillVertex) -> bool:
    conn_arc = False
    if mv0.is_arc and mv1.is_arc:
        if mv0.arc_ctr[0] == mv1.arc_ctr[0]:
            if mv0.arc_ctr[1] == mv1.arc_ctr[1]:
                conn_arc = True
    return conn_arc


def arc_points(st: ArrayLike, en: ArrayLike, ctr0: ArrayLike, ctr1: ArrayLike, ax: int, astep: ArrayLike) -> list:
    pts = []
    # numpy arrays
    st = np.array(st)
    en = np.array(en)
    ctr0 = np.array(ctr0)
    ctr1 = np.array(ctr1)
    # calculate steps and count and produce in between points
    v0 = st - ctr0
    v1 = en - ctr1
    cnt = int(0.5 + angle_between(v0, v1) / astep)
    astep = angle_between(v0, v1) / cnt
    zstep = (en[ax] - st[ax]) / cnt
    ax_vec = np.cross(v0, v1)
    for i in range(1, cnt + 1):
        rvec = rotate_vector_around_axis(v0, ax_vec, astep * i)
        zvec = [0, 0, zstep * i]
        pts.append(ctr0 + rvec + zvec)
    return pts


class RegionVertex:
    def __init__(self, ind: list[int], abs_ind: list[int], neighbors: ArrayLike,
                 neighbor_values: list[list[Optional[int]]],
                 dia: bool = False,
                 minus_one_neighbor: bool = False) -> None:
        self.ind = ind
        self.i = ind[0]
        self.j = ind[1]
        self.neighbors = neighbors
        self.flat_neighbors = self.neighbors.flatten()
        self.region_count = np.sum(self.flat_neighbors == 0)
        self.block_count = np.sum(self.flat_neighbors == 1)
        self.free_count = np.sum(self.flat_neighbors == 2)
        ##
        self.minus_one_neighbor = minus_one_neighbor
        ##
        self.dia = dia
        ##
        self.neighbor_values = np.array(neighbor_values)
        self.flat_neighbor_values = self.neighbor_values.flatten()


# noinspection PyChainedComparisons
class RoughPixel:
    def __init__(self, ind: ArrayLike, mat: ArrayLike, pad_loc: tuple, dim, n) -> None:
        self.ind = ind
        self.ind_abs = ind.copy()
        self.ind_abs[0] -= pad_loc[0][0]
        self.ind_abs[1] -= pad_loc[1][0]
        self.outside = False
        if self.ind_abs[0] < 0 or self.ind_abs[0] >= dim:
            self.outside = True
        elif self.ind_abs[1] < 0 or self.ind_abs[1] >= dim:
            self.outside = True
        self.neighbors = []
        # Region or free=0
        # Blocked=1
        for ax in range(2):
            temp = []
            for direction in range(-1, 2, 2):
                nind = self.ind.copy()
                nind[ax] += direction
                neighbor_type = 0
                if nind[0] >= 0 and nind[0] < mat.shape[0] and nind[1] >= 0 and nind[1] < mat.shape[1]:
                    val = mat[tuple(nind)]
                    if val == n: neighbor_type = 1
                temp.append(neighbor_type)
            self.neighbors.append(temp)
        self.flat_neighbors = [x for sublist in self.neighbors for x in sublist]


class Fabrication:
    def __init__(self, joint_type,                                          # parent is JointType
                 tolerances: float = 0.15,
                 bit_diameter: float = 6.00,
                 fab_ext: FabricationExt = "gcode",
                 align_ax: int = 0,
                 arc_interp: bool = True,
                 fab_speed: int = 400,
                 spindle_speed: int = 6000) -> None:
        self.joint_type = joint_type
        self.real_diam = bit_diameter  # milling bit radius in mm
        self.tolerances = tolerances  # 0.10 #tolerance in mm
        self.radius = 0.5 * self.real_diam - self.tolerances
        self.diameter = 2 * self.radius
        # does v means voxel, vector, vertical
        self.vdiam = self.diameter / self.joint_type.ratio
        self.vradius = self.radius / self.joint_type.ratio
        self.vtolerances = self.tolerances / self.joint_type.ratio

        self.depth = 1.5  # milling depth in mm
        self.alignment_axis = align_ax
        self.export_ext = fab_ext
        self.arc_interp = arc_interp
        self.milling_speed = fab_speed
        self.spindle_speed = spindle_speed

    def export_gcode(self, filename_tsu: FilePath = os.getcwd() + os.sep + "joint.tsu") -> None:
        # make sure that the z axis of the gcode is facing up
        fax = self.joint_type.sliding_axis
        coords = [0, 1]
        coords.insert(fax, 2)
        #
        d = 3  # =precision / no of decimals to write
        names = ["A", "B", "C", "D", "E", "F"]
        for n in range(self.joint_type.timber_count):
            fdir = self.joint_type.mesh.fab_directions[n]
            comp_ax = self.joint_type.fixed_sides.sides[n][0].ax
            comp_dir = self.joint_type.fixed_sides.sides[n][0].direction  # component direction
            comp_vec = self.joint_type.pos_vecs[comp_ax]
            if comp_dir == 0 and comp_ax != self.joint_type.sliding_axis: comp_vec = -comp_vec
            comp_vec = np.array([comp_vec[coords[0]], comp_vec[coords[1]], comp_vec[coords[2]]])
            comp_vec = comp_vec / np.linalg.norm(comp_vec)  # unitize
            zax = np.array([0, 0, 1])
            aax = [0, 0, 0]
            aax[int(self.alignment_axis / 2)] = 2 * (self.alignment_axis % 2) - 1
            # aax = rotate_vector_around_axis(aax, axis=zax, theta=math.radians(self.extra_rot_angle))
            rot_ang = angle_between(aax, comp_vec, normal_vector=zax)
            if fdir == 0: rot_ang = -rot_ang
            #
            file_name = filename_tsu[:-4] + "_" + names[n] + "." + self.export_ext
            file = open(file_name, "w")
            if self.export_ext == "gcode" or self.export_ext == "nc":
                # initialization .goce and .nc
                file.write("%\n")
                file.write("G90 (Absolute [G91 is increm_depth])\n")
                file.write("G17 (set XY plane for circle path)\n")
                file.write("G94 (set unit/minute)\n")
                file.write("G21 (set unit[mm])\n")
                spistr = str(int(self.spindle_speed))
                file.write("S" + spistr + " (Spindle " + spistr + "rpm)\n")
                file.write("M3 (spindle start)\n")
                file.write("G54\n")
                spestr = str(int(self.milling_speed))
                file.write("F" + spestr + " (Feed " + spestr + "mm/min)\n")
            elif self.export_ext == "sbp":
                file.write("'%\n")
                file.write("SA\n")
                file.write("MS,6.67,6.67\n\n")
                file.write("TR 6000\n\n")
                file.write("SO 1,1\n")
            else:
                print("Unknown extension:", self.export_ext)

            # content
            for i, mv in enumerate(self.joint_type.gcode_verts[n]):
                mv.scale_and_swap(fax, fdir, self.joint_type.ratio, self.joint_type.real_timber_dims, coords, d)
                if comp_ax != fax: mv.rotate(rot_ang, d)
                if i > 0: pmv = self.joint_type.gcode_verts[n][i - 1]
                # check segment angle
                arc = False
                clockwise = False
                if i > 0 and is_connected_arc(mv, pmv):
                    arc = True
                    vec1 = mv.pt - mv.arc_ctr
                    vec1 = vec1 / np.linalg.norm(vec1)
                    zvec = np.array([0, 0, 1])
                    xvec = np.cross(vec1, zvec)
                    vec2 = pmv.pt - mv.arc_ctr
                    vec2 = vec2 / np.linalg.norm(vec2)
                    diff_ang = angle_between(xvec, vec2)
                    if diff_ang > 0.5 * math.pi: clockwise = True

                # write to file
                if self.export_ext == "gcode" or self.export_ext == "nc":
                    if arc and self.arc_interp:
                        if clockwise:
                            file.write("G2")
                        else:
                            file.write("G3")
                        file.write(" R" + str(round(self.diameter, d)) + " X" + mv.xstr + " Y" + mv.ystr)
                        if mv.z != pmv.z: file.write(" Z" + mv.zstr)
                        file.write("\n")
                    elif arc and not self.arc_interp:
                        pts = arc_points(pmv.pt, mv.pt, pmv.arc_ctr, mv.arc_ctr, 2, math.radians(1))
                        for pt in pts:
                            file.write("G1")
                            file.write(" X" + str(round(pt[0], 3)) + " Y" + str(round(pt[1], 3)))
                            if mv.z != pmv.z: file.write(" Z" + str(round(pt[2], 3)))
                            file.write("\n")
                    elif i == 0 or mv.x != pmv.x or mv.y != pmv.y or mv.z != pmv.z:
                        if mv.is_traversing:
                            file.write("G0")
                        else:
                            file.write("G1")
                        if i == 0 or mv.x != pmv.x: file.write(" X" + mv.xstr)
                        if i == 0 or mv.y != pmv.y: file.write(" Y" + mv.ystr)
                        if i == 0 or mv.z != pmv.z: file.write(" Z" + mv.zstr)
                        file.write("\n")
                elif self.export_ext == "sbp":
                    if arc and mv.z == pmv.z:
                        file.write("CG," + str(round(2 * self.diameter, d)) + "," + mv.xstr + "," + mv.ystr + ",,,T,")
                        if clockwise:
                            file.write("1\n")
                        else:
                            file.write("-1\n")
                    elif arc and mv.z != pmv.z:
                        pts = arc_points(pmv.pt, mv.pt, pmv.arc_ctr, mv.arc_ctr, 2, math.radians(1))
                        for pt in pts:
                            file.write("M3," + str(round(pt[0], 3)) + "," + str(round(pt[1], 3)) + "," + str(
                                round(pt[2], 3)) + "\n")
                    elif i == 0 or mv.x != pmv.x or mv.y != pmv.y or mv.z != pmv.z:
                        if mv.is_traversing:
                            file.write("J3,")
                        else:
                            file.write("M3,")
                        if i == 0 or mv.x != pmv.x:
                            file.write(mv.xstr + ",")
                        else:
                            file.write(" ,")
                        if i == 0 or mv.y != pmv.y:
                            file.write(mv.ystr + ",")
                        else:
                            file.write(" ,")
                        if i == 0 or mv.z != pmv.z:
                            file.write(mv.zstr + "\n")
                        else:
                            file.write(" \n")
            # end
            if self.export_ext == "gcode" or self.export_ext == "nc":
                file.write("M5 (Spindle stop)\n")
                file.write("M2 (end of program)\n")
                file.write("M30 (delete sd file)\n")
                file.write("%\n")
            elif self.export_ext == "sbp":
                file.write("SO 1,0\n")
                file.write("END\n")
                file.write("'%\n")

            print("Exported", file_name)
            file.close()
