import math
import copy
import os
import random

import numpy as np
import PyQt5.QtWidgets as qtw
import OpenGL.GL as gl

from buffer import Buffer
from evaluation import Evaluation
from fabrication import *
from geometries import Geometries, get_index
from fixed_sides import FixedSides
from utils import *


# noinspection PyDefaultArgument,PyAttributeOutsideInit,PyBroadException,PyChainedComparisons
class JointType:
    def __init__(self, _glWidget,  # parent is GLWidget
                 fs=[],
                 sliding_axis=2,
                 voxel_res=3,
                 angle=0.0,
                 timber_dims=[44.0, 44.0, 44.0],  # xdim, ydim, zdim
                 milling_speed=400,
                 spindle_speed=6000,
                 tolerances=0.15,
                 milling_diam=6.00,
                 alignment_axis=0,
                 fab_ext="gcode",
                 increm_depth=False,
                 height_fields=[],
                 arc_interp=True):

        # glWidget is parent for JointType
        self._glWidget = _glWidget
        self.sliding_axis = sliding_axis
        self.fixed_sides = FixedSides(self)
        self.timber_count = len(self.fixed_sides.sides)  # number of components
        self.voxel_res = voxel_res
        self.suggestions_on = True
        self.component_size = 0.275
        self.real_timber_dims = np.array(timber_dims)
        self.component_length = 0.5 * self.component_size
        self.ratio = np.average(self.real_timber_dims) / self.component_size
        self.voxel_sizes = np.copy(self.real_timber_dims) / (self.ratio * self.voxel_res)
        self.fab = Fabrication(self, tolerances=tolerances, bit_diameter=milling_diam, fab_ext=fab_ext,
                               align_ax=alignment_axis, arc_interp=arc_interp, spindle_speed=spindle_speed,
                               fab_speed=milling_speed)
        self.vertex_num = 8
        self.angle = angle
        self.buffer = Buffer(self)  # initiating the buffer
        self.fixed_sides.update_unblocked()
        self.verts = self.create_and_buffer_vertices(milling_path=False)  # create and buffer verts
        self.mesh = Geometries(self, height_fields=height_fields)
        self.suggestions = []
        self.gallery_figures = []
        self.update_suggestions()
        self.combine_and_buffer_indices()
        self.gallery_start_index = -20
        self.increm_depth = increm_depth

    def create_and_buffer_vertices(self, milling_path=False):
        self.joint_verts = []
        self.eval_verts = []
        self.milling_verts = []
        self.gcode_verts = []

        for ax in range(3):
            self.joint_verts.append(self.create_joint_vertices(ax))

        if milling_path:
            for n in range(self.timber_count):
                mvs, gvs = self.milling_path_vertices(n)
                self.milling_verts.append(mvs)
                self.gcode_verts.append(gvs)

        arrow_verts = self.get_arrow_vertices()

        # Combine
        joint_vertices = np.concatenate(self.joint_verts)

        if milling_path and len(self.milling_verts[0]) > 0:
            milling_vertices = np.concatenate(self.milling_verts)
            self.verts = np.concatenate([joint_vertices, arrow_verts, milling_vertices])
        else:
            self.verts = np.concatenate([joint_vertices, arrow_verts])

        self.verts_num = int(len(self.joint_verts[0]) / 8)
        self.arrow_verts_num = int(len(arrow_verts) / 8)

        if milling_path and len(self.milling_verts[0]) > 0:
            self.m_start = []
            mst = 3 * self.verts_num + self.arrow_verts_num
            for n in range(self.timber_count):
                self.m_start.append(mst)
                mst += int(len(self.milling_verts[n]) / 8)

        return self.buffer.buffer_vertices()

    def create_joint_vertices(self, ax):
        verts = []
        r = g = b = 0.0

        # Create vectors - one for each of the 3 axis
        vec_x = np.array([1.0, 0, 0]) * self.voxel_sizes[0]
        vec_y = np.array([0, 1.0, 0]) * self.voxel_sizes[1]
        vec_z = np.array([0, 0, 1.0]) * self.voxel_sizes[2]
        self.pos_vecs = [vec_x, vec_y, vec_z]

        # If it is possible to rotate the geometry, rotate position vectors
        if self.rot:
            non_sliding_axis = [0, 1, 2]
            non_sliding_axis.remove(self.sliding_axis)
            for i, ax in enumerate(non_sliding_axis):
                theta = math.radians(0.5 * self.angle)
                if i % 2 == 1: theta = -theta
                self.pos_vecs[ax] = rotate_vector_around_axis(self.pos_vecs[ax], self.pos_vecs[self.sliding_axis], theta)
                self.pos_vecs[ax] = self.pos_vecs[ax] / math.cos(math.radians(abs(self.angle)))

        # Add all verts of the voxel_res*voxel_res*voxel_res voxel cube
        for i in range(self.voxel_res + 1):
            for j in range(self.voxel_res + 1):
                for k in range(self.voxel_res + 1):
                    # position coordinates
                    ivec = (i - 0.5 * self.voxel_res) * self.pos_vecs[0]
                    jvec = (j - 0.5 * self.voxel_res) * self.pos_vecs[1]
                    kvec = (k - 0.5 * self.voxel_res) * self.pos_vecs[2]
                    pos = ivec + jvec + kvec
                    x, y, z = pos

                    # texture coordinates
                    tex_coords = [i, j, k]
                    tex_coords.pop(ax)
                    tx = tex_coords[0] / self.voxel_res
                    ty = tex_coords[1] / self.voxel_res

                    # extend list of verts
                    verts.extend([x, y, z, r, g, b, tx, ty])

        # Calculate extra length for angled components
        extra_len = 0
        if self.angle != 0.0 and self.rot:
            extra_len = 0.1 * self.component_size * math.tan(math.radians(abs(self.angle)))

        # Add component base verts
        for ax in range(3):
            if ax == self.sliding_axis:
                extra_l = 0

            else:
                extra_l = extra_len
            for direction in range(-1, 2, 2):
                for step in range(3):
                    if step == 0:
                        step = 1
                    else:
                        step += 0.5 + extra_len
                    axvec = direction * step * (self.component_size + extra_l) * self.pos_vecs[ax] / np.linalg.norm(
                        self.pos_vecs[ax])
                    for x in range(2):
                        for y in range(2):
                            other_vecs = copy.deepcopy(self.pos_vecs)
                            other_vecs.pop(ax)
                            if ax != self.sliding_axis and self.rot and step != 0.5:
                                # cvec = copy.deep(self.pos_vecs[ax])
                                xvec = (x - 0.5) * self.voxel_res * other_vecs[0]  # +cvec
                                yvec = (y - 0.5) * self.voxel_res * other_vecs[1]  # -cvec
                            else:
                                xvec = (x - 0.5) * self.voxel_res * other_vecs[0]
                                yvec = (y - 0.5) * self.voxel_res * other_vecs[1]
                            pos = axvec + xvec + yvec
                            # texture coordinates
                            tex_coords = [x, y]
                            tx = tex_coords[0]
                            ty = tex_coords[1]
                            # extend list of verts
                            verts.extend([pos[0], pos[1], pos[2], r, g, b, tx, ty])
        # Format
        verts = np.array(verts, dtype=np.float32)  # converts to correct format
        return verts

    def combine_and_buffer_indices(self, milling_path=False):
        self.update_suggestions()
        self.mesh.create_indices(milling_path=milling_path)
        glo_off = len(self.mesh.indices)  # global offset
        for i in range(len(self.suggestions)):
            self.suggestions[i].create_indices(glo_off=glo_off, milling_path=False)
            glo_off += len(self.suggestions[i].indices)
        for i in range(len(self.gallery_figures)):
            self.gallery_figures[i].create_indices(glo_off=glo_off, milling_path=False)
            glo_off += len(self.gallery_figures[i].indices)
        indices = []
        indices.extend(self.mesh.indices)
        for mesh in self.suggestions: indices.extend(mesh.indices)
        for mesh in self.gallery_figures: indices.extend(mesh.indices)
        self.indices = np.array(indices, dtype=np.uint32)
        Buffer.buffer_indices(self.buffer)

    def update_sliding_direction(self, sliding_axis) -> tuple[bool, str]:
        blocked = False
        for i, sides in enumerate(self.fixed_sides.sides):
            for side in sides:
                if side.ax == sliding_axis:
                    if side.direction == 0 and i == 0: continue
                    if side.direction == 1 and i == self.timber_count - 1: continue
                    blocked = True
        if blocked:
            return False, "This sliding direction is blocked"
        else:
            self.sliding_axis = sliding_axis
            self.fixed_sides.update_unblocked()
            self.create_and_buffer_vertices(milling_path=False)
            self.mesh.voxel_matrix_from_height_fields()
            for mesh in self.suggestions: mesh.voxel_matrix_from_height_fields()
            self.combine_and_buffer_indices()
            return True, ''

    def update_dimension(self, add):
        self.voxel_res += add
        self.voxel_sizes = np.copy(self.real_timber_dims) / (self.ratio * self.voxel_res)
        self.create_and_buffer_vertices(milling_path=False)
        self.mesh.randomize_height_fields()

    def update_angle(self, angle):
        self.angle = angle
        self.create_and_buffer_vertices(milling_path=False)

    def update_timber_width_and_height(self, inds, val, milling_path=False):
        for i in inds:
            self.real_timber_dims[i] = val

        self.ratio = np.average(self.real_timber_dims) / self.component_size
        self.voxel_sizes = np.copy(self.real_timber_dims) / (self.ratio * self.voxel_res)
        self.fab.vdiam = self.fab.diameter / self.ratio
        self.fab.vradius = self.fab.radius / self.ratio
        self.fab.vtolerances = self.fab.tolerances / self.ratio
        self.create_and_buffer_vertices(milling_path)

    def update_number_of_components(self, new_num_of_components):
        if new_num_of_components != self.timber_count:
            # Increasing number of components
            if new_num_of_components > self.timber_count:
                if len(self.fixed_sides.unblocked) >= (new_num_of_components - self.timber_count):
                    for i in range(new_num_of_components - self.timber_count):
                        random_i = random.randint(0, len(self.fixed_sides.unblocked) - 1)
                        if self.fixed_sides.sides[-1][0].ax == self.sliding_axis:  # last component is aligned with the sliding axis
                            self.fixed_sides.sides.insert(-1, [self.fixed_sides.unblocked[random_i]])
                        else:
                            self.fixed_sides.sides.append([self.fixed_sides.unblocked[random_i]])
                        # also consider if it is aligned and should be the first one in line... rare though...
                        self.fixed_sides.update_unblocked()
                    self.timber_count = new_num_of_components

            # Decreasing number of components
            elif new_num_of_components < self.timber_count:
                for i in range(self.timber_count - new_num_of_components):
                    self.fixed_sides.sides.pop()
                self.timber_count = new_num_of_components

            # Rebuffer
            self.fixed_sides.update_unblocked()
            self.create_and_buffer_vertices(milling_path=False)
            self.mesh.randomize_height_fields()

    def update_component_position(self, new_sides, n):
        self.fixed_sides.sides[n] = new_sides
        self.fixed_sides.update_unblocked()
        self.create_and_buffer_vertices(milling_path=False)
        self.mesh.voxel_matrix_from_height_fields()
        self.combine_and_buffer_indices()

    def reset(self, fs=None, sliding_axis=2, voxel_res=3, angle=90., timber_dims=[44.0, 44.0, 44.0], increm=False,
              alignment_axis=0, milling_diam=6.0, fab_tolerances=0.15, arc_interp=True, fab_rot_angle=0.0,
              fab_ext="gcode", height_fields: ArrayLike = np.array([]), milling_speed=400, spindle_speed=600):
        self.fixed_sides = FixedSides(self, fs=fs)
        self.timber_count = len(self.fixed_sides.sides)
        self.sliding_axis = sliding_axis
        self.voxel_res = voxel_res
        self.angle = angle
        self.real_timber_dims = np.array(timber_dims)
        self.ratio = np.average(self.real_timber_dims) / self.component_size
        self.voxel_sizes = np.copy(self.real_timber_dims) / (self.ratio * self.voxel_res)
        self.fab.tolerances = fab_tolerances
        self.fab.real_diam = milling_diam
        self.fab.radius = 0.5 * self.fab.real_diam - self.fab.tolerances
        self.fab.diameter = 2 * self.fab.radius
        self.fab.vdiam = self.fab.diameter / self.ratio
        self.fab.vradius = self.fab.radius / self.ratio
        self.fab.vtolerances = self.fab.tolerances / self.ratio
        self.fab.milling_speed = milling_speed
        self.fab.spindle_speed = spindle_speed
        self.fab.extra_rot_angle = fab_rot_angle
        self.fab.export_ext = fab_ext
        self.fab.alignment_axis = alignment_axis
        self.fab.arc_interp = arc_interp
        self.increm_depth = increm
        self.mesh = Geometries(self, height_fields=height_fields)
        self.fixed_sides.update_unblocked()
        self.create_and_buffer_vertices(milling_path=False)
        self.combine_and_buffer_indices()

    def update_suggestions(self):
        self.suggestions = []  # clear list of suggestions
        if self.suggestions_on:
            sugg_hfs = []
            if not self.mesh.eval.valid:
                sugg_hfs = self.produce_suggestions(self.mesh.height_fields)
                for i in range(len(sugg_hfs)): self.suggestions.append(Geometries(self,
                                                                                  main_mesh=False, height_fields=sugg_hfs[i]))

    def init_gallery(self, start_index):
        self.gallery_start_index = start_index
        # do not reset again because they are already in the init
        # self.gallery_figures = []
        # self.suggestions = []

        # Folder
        location = os.path.abspath(os.getcwd())
        location = location.split(os.sep)
        location.pop()
        location = os.sep.join(location)
        location += os.sep + "search_results" + os.sep + "noc_" + str(self.timber_count) + os.sep + "res_" + str(
            self.voxel_res) + os.sep + "fs_"

        for i in range(len(self.fixed_sides.sides)):
            for fs in self.fixed_sides.sides[i]:
                location += str(fs.ax) + str(fs.direction)
            if i != len(self.fixed_sides.sides) - 1:
                location += "_"

        location += os.sep + "allvalid"
        maxi = len(os.listdir(location)) - 1

        for i in range(20):
            if (i + start_index) > maxi: break
            try:
                hfs = np.load(location + os.sep + "height_fields_" + str(start_index + i) + ".npy")
                self.gallery_figures.append(Geometries(self, main_mesh=False, height_fields=hfs))
            except:
                abc = 0

    def save(self, filename="joint.tsu"):

        """
        Meaning of abbreviations:
        sliding_axis                    (0-2)   (the sliding axis, not all possible sliding directions) (refer to Figure 3d of the paper)
        timber_count                    (2-6)   (refer to Figure 3e of the paper)
        voxel_res                       (2-5)   (2-->[2,2,2], 3-->[3,3,3] and so on. Non-uniform voxel_res such as [2,3,4] is not possible currently) (refer to Figure 3f of the paper)
        angle: angle of intersection    (refer to Figure 27a of the paper)
        timber_xdim:
        timber_ydim:
        timber_zdim: (mm)               (TDX, TDY, and TDZ does not have to be equal. Refer for Figure 27b of the paper)
        milling_diam:
        tolerances
        milling_speed
        spindle_speed
        increm_depth                     (T/F)   Option for the layering of the milling path to avoid "downcuts"
        arc_interp                       (T/F)   Milling path true arcs or divided into many points (depending on milling machine)
        alignment_axis                   Axis to align the timber element with during fabrication
        export_ext:("gcode"/"sbp"/"nc")  File format for the milling machine. Roland machine: nc. Shopbot machine: sbp
        fixed_sides                      Fixed sides of the cube are connected to the timber (non-fixed_sides sides are free/open)
        height_fields                    Voxel geometry described by height fields of size res*res

        These abbreviations are only for the file
        """

        # Initiate
        file = open(filename, "w")

        # note that
        # Joint properties
        file.write("sliding_axis " + str(self.sliding_axis) + "\n")
        file.write("timber_count " + str(self.timber_count) + "\n")
        file.write("voxel_res " + str(self.voxel_res) + "\n")
        file.write("angle " + str(self.angle) + "\n")
        file.write("timber_xdim " + str(self.real_timber_dims[0]) + "\n")
        file.write("timber_ydim " + str(self.real_timber_dims[1]) + "\n")
        file.write("timber_zdim " + str(self.real_timber_dims[2]) + "\n")
        file.write("milling_diam " + str(self.fab.real_diam) + "\n")
        file.write("tolerances " + str(self.fab.tolerances) + "\n")
        file.write("milling_speed " + str(self.fab.milling_speed) + "\n")
        file.write("spindle_speed " + str(self.fab.spindle_speed) + "\n")
        file.write("increm_depth " + str(self.increm_depth) + "\n")
        file.write("arc_interp " + str(self.fab.arc_interp) + "\n")
        file.write("alignment_axis " + str(self.fab.alignment_axis) + "\n")
        file.write("export_ext " + self.fab.export_ext + "\n")

        # Fixed sides
        file.write("fixed_sides ")
        for n in range(len(self.fixed_sides.sides)):
            for i in range(len(self.fixed_sides.sides[n])):
                file.write(str(int(self.fixed_sides.sides[n][i].ax)) + ",")
                file.write(str(int(self.fixed_sides.sides[n][i].direction)))
                if i != len(self.fixed_sides.sides[n]) - 1: file.write(".")
            if n != len(self.fixed_sides.sides) - 1: file.write(":")

        # Joint geometry
        file.write("\nheight_fields \n")
        for n in range(len(self.mesh.height_fields)):
            for i in range(len(self.mesh.height_fields[n])):
                for j in range(len(self.mesh.height_fields[n][i])):
                    file.write(str(int(self.mesh.height_fields[n][i][j])))
                    if j != len(self.mesh.height_fields[n][i]) - 1: file.write(",")
                if i != len(self.mesh.height_fields[n]) - 1: file.write(":")
            if n != len(self.mesh.height_fields) - 1: file.write("\n")

        # Finalize
        print("Saved", filename)
        file.close()

    def open(self, filename="joint.tsu"):

        # Open
        file = open(filename, "r")

        # Default values
        sliding_axis = self.sliding_axis
        timber_count = self.timber_count
        voxel_res = self.voxel_res
        angle = self.angle
        dx, dy, dz = self.real_timber_dims
        diam = self.fab.real_diam
        tolerances = self.fab.tolerances
        milling_speed = self.fab.milling_speed
        spindle_speed = self.fab.spindle_speed
        increm_depth = self.increm_depth
        alignment_axis = self.fab.alignment_axis
        export_ext = self.fab.export_ext
        fixed_sides = self.fixed_sides.sides
        arc_interp = self.fab.arc_interp

        # Read
        hfs = []
        hfi = 999
        for i, line in enumerate(file.readlines()):
            items = line.split()
            if items[0] == "sliding_axis":
                sliding_axis = int(items[1])
            elif items[0] == "timber_count":
                timber_count = int(items[1])
            elif items[0] == "voxel_res":
                voxel_res = int(items[1])
            elif items[0] == "angle":
                angle = float(items[1])
            elif items[0] == "timber_xdim":
                dx = float(items[1])
            elif items[0] == "timber_ydim":
                dy = float(items[1])
            elif items[0] == "timber_zdim":
                dz = float(items[1])
            elif items[0] == "milling_diam":
                diam = float(items[1])
            elif items[0] == "tolerances":
                tolerances = float(items[1])
            elif items[0] == "milling_speed":
                milling_speed = float(items[1])
            elif items[0] == "spindle_speed":
                spindle_speed = float(items[1])
            elif items[0] == "increm_depth":
                if items[1] == "True":
                    increm_depth = True
                else:
                    increm_depth = False
            elif items[0] == "arc_interp":
                if items[1] == "True":
                    arc_interp = True
                else:
                    arc_interp = False
            elif items[0] == "alignment_axis":
                alignment_axis = float(items[1])
            elif items[0] == "export_ext":
                export_ext = items[1]
            elif items[0] == "fixed_sides":
                fixed_sides = FixedSides(self, side_str=items[1]).sides
            elif items[0] == "height_fields":
                hfi = i
            elif i > hfi:
                hf = []
                for row in line.split(":"):
                    temp = []
                    for item in row.split(","): temp.append(int(float(item)))
                    hf.append(temp)
                hfs.append(hf)
        hfs = np.array(hfs)

        # Reinitiate
        self.reset(fs=fixed_sides, sliding_axis=sliding_axis, voxel_res=voxel_res, angle=angle, timber_dims=[dx, dy, dz], milling_diam=diam,
                   fab_tolerances=tolerances, alignment_axis=alignment_axis, arc_interp=arc_interp, increm=increm_depth,
                   fab_ext=export_ext, height_fields=hfs, milling_speed=milling_speed, spindle_speed=spindle_speed)

    def get_arrow_vertices(self) -> ArrayLike:
        vertices = []
        r = g = b = 0.0
        tx = ty = 0.0
        vertices.extend([0, 0, 0, r, g, b, tx, ty])  # origin
        for ax in range(3):
            for direction in range(-1, 2, 2):
                # arrow base
                xyz = direction * self.pos_vecs[ax] * self.voxel_res * 0.4
                vertices.extend([xyz[0], xyz[1], xyz[2], r, g, b, tx, ty])  # end of line
                # arrow head
                for i in range(-1, 2, 2):
                    for j in range(-1, 2, 2):
                        other_axes = [0, 1, 2]
                        other_axes.pop(ax)
                        pos = direction * self.pos_vecs[ax] * self.voxel_res * 0.3
                        pos += i * self.pos_vecs[other_axes[0]] * self.voxel_res * 0.025
                        pos += j * self.pos_vecs[other_axes[1]] * self.voxel_res * 0.025
                        vertices.extend([pos[0], pos[1], pos[2], r, g, b, tx, ty])  # arrow head indices
        # Format
        vertices = np.array(vertices, dtype=np.float32)  # converts to correct format
        return vertices

    def produce_suggestions(self, hfs: list) -> list:
        valid_suggestions = []
        for i in range(len(hfs)):
            for j in range(self.voxel_res):
                for k in range(self.voxel_res):
                    for add in range(-1, 2, 2):
                        sugg_hfs = copy.deepcopy(hfs)
                        sugg_hfs[i][j][k] += add
                        val = sugg_hfs[i][j][k]

                        if val >= 0 and val <= self.voxel_res:
                            sugg_voxmat = mat_from_fields(sugg_hfs, self.sliding_axis)
                            sugg_eval = Evaluation(sugg_voxmat, self, main_mesh=False)
                            if sugg_eval.valid:
                                valid_suggestions.append(sugg_hfs)
                                if len(valid_suggestions) == 4: break
        return valid_suggestions

    def layer_mat_from_cube(self, lay_num: int, n: int) -> ArrayLike:
        mat = np.ndarray(shape=(self.voxel_res, self.voxel_res), dtype=int)
        fdir = self.mesh.fab_directions[n]
        for i in range(self.voxel_res):
            for j in range(self.voxel_res):
                ind = [i, j]
                zval = (self.voxel_res - 1) * (1 - fdir) + (2 * fdir - 1) * lay_num
                ind.insert(self.sliding_axis, zval)
                mat[i][j] = int(self.mesh.voxel_matrix[tuple(ind)])
        return mat

    def pad_layer_mat_with_fixed_sides(self, mat, n):
        pad_loc = [[0, 0], [0, 0]]
        pad_val = [[-1, -1], [-1, -1]]
        for n2 in range(len(self.fixed_sides.sides)):
            for oside in self.fixed_sides.sides[n2]:
                if oside.ax == self.sliding_axis: continue
                axes = [0, 0, 0]
                axes[oside.ax] = 1
                axes.pop(self.sliding_axis)
                oax = axes.index(1)
                pad_loc[oax][oside.direction] = 1
                pad_val[oax][oside.direction] = n2
        # If it is an angled joint, pad so that the edge of a joint located on an edge will be trimmed well
        # if abs(self.angle-90)>1 and len(self.fixed_sides.sides[n])==1 and self.fixed_sides.sides[n][0].ax!=self.sliding_axis:
        #    print("get here")
        #    ax = self.fixed_sides.sides[n][0].ax
        #    direction = self.fixed_sides.sides[n][0].direction
        #    odir = 1-direction
        #    axes = [0,0,0]
        #    axes[ax] = 1
        #    axes.pop(self.sliding_axis)
        #    oax = axes.index(1)
        #    pad_loc[oax][odir] = 1
        #    pad_val[oax][odir] = 9
        # Perform the padding
        pad_loc = tuple(map(tuple, pad_loc))
        pad_val = tuple(map(tuple, pad_val))
        mat = np.pad(mat, pad_loc, 'constant', constant_values=pad_val)
        # take care of -1 corners # does this still work after adding former step??????????????
        # This could be shorter for sure...
        for fixed_sides_1 in self.fixed_sides.sides:
            for fixed_sides_2 in self.fixed_sides.sides:
                for side1 in fixed_sides_1:
                    if side1.ax == self.sliding_axis: continue
                    axes = [0, 0, 0]
                    axes[side1.ax] = 1
                    axes.pop(self.sliding_axis)
                    ax1 = axes.index(1)
                    for side2 in fixed_sides_2:
                        if side2.ax == self.sliding_axis: continue
                        axes = [0, 0, 0]
                        axes[side2.ax] = 1
                        axes.pop(self.sliding_axis)
                        ax2 = axes.index(1)
                        if ax1 == ax2: continue
                        ind = [0, 0]
                        ind[ax1] = side1.direction * (mat.shape[ax1] - 1)
                        ind[ax2] = side2.direction * (mat.shape[ax2] - 1)
                        mat[tuple(ind)] = -1
        return mat, pad_loc

    def milling_path_vertices(self, n):
        vertices = []
        milling_vertices = []

        min_vox_size = np.min(self.voxel_sizes)
        # Check that the milling bit is not too large for the voxel size
        if np.min(self.voxel_sizes) < self.fab.vdiam: print("Could not generate milling path. The milling bit is too large.")

        # Calculate depth constants
        no_z = int(self.ratio * self.voxel_sizes[self.sliding_axis] / self.fab.depth)
        dep = self.voxel_sizes[self.sliding_axis] / no_z

        # Defines axes and vectors
        fdir = self.mesh.fab_directions[n]
        axes = [0, 1, 2]
        axes.pop(self.sliding_axis)
        dir_ax = axes[0]  # primary milling direction axis
        off_ax = axes[1]  # milling offset axis
        ### new for oblique angles ### neighbor vectors
        le = self.fab.vradius / math.cos(abs(math.radians(-self.angle)))
        dir_vec = le * self.pos_vecs[axes[0]] / np.linalg.norm(self.pos_vecs[axes[0]])
        off_vec = le * self.pos_vecs[axes[1]] / np.linalg.norm(self.pos_vecs[axes[1]])
        neighbor_vectors = []
        neighbor_vectors_a = []
        neighbor_vectors_b = []
        for x in range(-1, 2, 2):
            temp = []
            tempa = []
            tempb = []
            for y in range(-1, 2, 2):
                temp.append(x * dir_vec + y * off_vec)
                tempa.append(x * dir_vec)
                tempb.append(y * off_vec)
            neighbor_vectors.append(temp)
            neighbor_vectors_a.append(tempa)
            neighbor_vectors_b.append(tempb)
        neighbor_vectors = np.array(neighbor_vectors)
        neighbor_vectors_a = np.array(neighbor_vectors_a)
        neighbor_vectors_b = np.array(neighbor_vectors_b)

        # Browse layers
        for lay_num in range(self.voxel_res):

            # Create a 2D matrix of current layer
            lay_mat = self.layer_mat_from_cube(lay_num, n)  # OK

            # Pad 2d matrix with fixed_sides sides
            lay_mat, pad_loc = self.pad_layer_mat_with_fixed_sides(lay_mat, n)  # OK
            org_lay_mat = copy.deepcopy(lay_mat)  # OK

            # Get/browse regions
            for reg_num in range(self.voxel_res * self.voxel_res):

                # Get indices of a region
                inds = np.argwhere((lay_mat != -1) & (lay_mat != n))  # OK
                if len(inds) == 0: break  # OK
                reg_inds = get_diff_neighbors(lay_mat, [inds[0]], n)  # OK

                # If oblique joint, create path to trim edge
                edge_path = []
                if abs(self.angle) > 1: edge_path = self.edge_milling_path(lay_num, n)
                if len(edge_path) > 0:
                    verts, mverts = self.get_layered_vertices(edge_path, n, lay_num, no_z, dep)
                    vertices.extend(verts)
                    milling_vertices.extend(mverts)

                # Anaylize which voxels needs to be roughly cut initially
                # 1. Add all open voxels in the region
                rough_inds = []
                for ind in reg_inds:
                    rough_inds.append(RoughPixel(ind, lay_mat, pad_loc, self.voxel_res, n))  # should be same...
                # 2. Produce rough milling paths
                rough_paths = self.rough_milling_path(rough_inds, lay_num, n)
                for rough_path in rough_paths:
                    if len(rough_path) > 0:
                        verts, mverts = self.get_layered_vertices(rough_path, n, lay_num, no_z, dep)
                        vertices.extend(verts)
                        milling_vertices.extend(mverts)

                # Overwrite detected regin in original matrix
                for reg_ind in reg_inds: lay_mat[tuple(reg_ind)] = n  # OK

                # Make a list of all edge verts of the outline of the region
                reg_verts = get_region_outline_vertices(reg_inds, lay_mat, org_lay_mat, pad_loc, n)  # OK

                # Order the verts to create an outline
                for isl_num in range(10):
                    reg_ord_verts = []
                    if len(reg_verts) == 0: break

                    # Make sure first item in region verts is on blocked/free corner, or blocked
                    reg_verts = set_starting_vert(reg_verts)  # OK

                    # Get a sequence of ordered verts
                    reg_ord_verts, reg_verts, closed = get_sublist_of_ordered_verts(reg_verts)  # OK

                    # Make outline of ordered verts (for dedugging only!!!!!!!)
                    # if len(reg_ord_verts)>1: outline = get_outline(joint_self,reg_ord_verts,lay_num,n)

                    # Offset verts according to boundary condition (and remove if redundant)
                    outline, corner_artifacts = self.offset_verts(neighbor_vectors, neighbor_vectors_a, neighbor_vectors_b,
                                                             reg_ord_verts, lay_num,
                                                             n)  # <----needs to be updated for oblique angles!!!!!<---

                    # Get z height and extend verts to global list
                    if len(reg_ord_verts) > 1 and len(outline) > 0:
                        if closed: outline.append(MillVertex(outline[0].pt))
                        verts, mverts = self.get_layered_vertices(outline, n, lay_num, no_z, dep)
                        vertices.extend(verts)
                        milling_vertices.extend(mverts)

                    if len(corner_artifacts) > 0:
                        for artifact in corner_artifacts:
                            verts, mverts = self.get_layered_vertices(artifact, n, lay_num, no_z, dep)
                            vertices.extend(verts)
                            milling_vertices.extend(mverts)

        # Add end point
        end_verts, end_mverts = self.get_milling_end_points(n, milling_vertices[-1].pt[self.sliding_axis])
        vertices.extend(end_verts)
        milling_vertices.extend(end_mverts)

        # Format and return
        vertices = np.array(vertices, dtype=np.float32)

        return vertices, milling_vertices

    def rough_milling_path(self, rough_pixs, lay_num, n):
        mvertices = []

        # Defines axes
        ax = self.sliding_axis  # mill bit axis
        direction = self.mesh.fab_directions[n]
        axes = [0, 1, 2]
        axes.pop(ax)
        dir_ax = axes[0]  # primary milling direction axis
        off_ax = axes[1]  # milling offset axis

        # Define fabrication parameters

        no_lanes = 2 + math.ceil(((self.real_timber_dims[axes[1]] / self.voxel_res) - 2 * self.fab.diameter) / self.fab.diameter)
        lane_width = (self.voxel_sizes[axes[1]] - self.fab.vdiam) / (no_lanes - 1)
        ratio = np.linalg.norm(self.pos_vecs[axes[1]]) / self.voxel_sizes[axes[1]]
        v_vrad = self.fab.vradius * ratio
        lane_width = lane_width * ratio

        # create offset direction vectors
        dir_vec = normalize(self.pos_vecs[axes[0]])
        off_vec = normalize(self.pos_vecs[axes[1]])

        # get top ones to cut out
        for pix in rough_pixs:
            mverts = []
            if pix.outside: continue
            if no_lanes <= 2:
                if pix.neighbors[0][0] == 1 and pix.neighbors[0][1] == 1:
                    continue
                elif pix.neighbors[1][0] == 1 and pix.neighbors[1][1] == 1:
                    continue
            pix_end = pix

            # check that there is no previous same
            nind = pix.ind_abs.copy()
            nind[dir_ax] -= 1
            found = False
            for pix2 in rough_pixs:
                if pix2.outside: continue
                if pix2.ind_abs[0] == nind[0] and pix2.ind_abs[1] == nind[1]:
                    if pix.neighbors[1][0] == pix2.neighbors[1][0]:
                        if pix.neighbors[1][1] == pix2.neighbors[1][1]:
                            found = True
                            break
            if found: continue

            # find next same
            for i in range(self.voxel_res):
                nind = pix.ind_abs.copy()
                nind[0] += i
                found = False
                for pix2 in rough_pixs:
                    if pix2.outside: continue
                    if pix2.ind_abs[0] == nind[0] and pix2.ind_abs[1] == nind[1]:
                        if pix.neighbors[1][0] == pix2.neighbors[1][0]:
                            if pix.neighbors[1][1] == pix2.neighbors[1][1]:
                                found = True
                                pix_end = pix2
                                break
                if not found: break

            # start
            ind = list(pix.ind_abs)
            ind.insert(ax, (self.voxel_res - 1) * (1 - direction) + (2 * direction - 1) * lay_num)  # 0 when n is 1, voxel_res-1 when n is 0
            add = [0, 0, 0]
            add[ax] = 1 - direction
            i_pt = get_index(ind, add, self.voxel_res)
            pt1 = get_vertex(i_pt, self.joint_verts[n], self.vertex_num)
            # end
            ind = list(pix_end.ind_abs)
            ind.insert(ax, (self.voxel_res - 1) * (1 - direction) + (2 * direction - 1) * lay_num)  # 0 when n is 1, voxel_res-1 when n is 0
            add = [0, 0, 0]
            add[ax] = 1 - direction
            add[dir_ax] = 1
            i_pt = get_index(ind, add, self.voxel_res)
            pt2 = get_vertex(i_pt, self.joint_verts[n], self.vertex_num)

            ### REFINE THIS FUNCTION
            dir_add1 = pix.neighbors[dir_ax][0] * 2.5 * self.fab.vradius * dir_vec
            dir_add2 = -pix_end.neighbors[dir_ax][1] * 2.5 * self.fab.vradius * dir_vec

            pt1 = pt1 + v_vrad * off_vec + dir_add1
            pt2 = pt2 + v_vrad * off_vec + dir_add2
            for i in range(no_lanes):
                # skip lane if on blocked side in off direction
                if pix.neighbors[1][0] == 1 and i == 0:
                    continue
                elif pix.neighbors[1][1] == 1 and i == no_lanes - 1:
                    continue

                ptA = pt1 + lane_width * off_vec * i
                ptB = pt2 + lane_width * off_vec * i
                pts = [ptA, ptB]
                if i % 2 == 1: pts.reverse()
                for pt in pts: mverts.append(MillVertex(pt))
            mvertices.append(mverts)
        return mvertices

    def edge_milling_path(self, lay_num, n):
        mverts = []

        if len(self.fixed_sides.sides[n]) == 1 and self.fixed_sides.sides[n][0].ax != self.sliding_axis:

            # ax direction of current fixed_sides side
            ax = self.fixed_sides.sides[n][0].ax
            direction = self.fixed_sides.sides[n][0].direction

            # oax - axis perp. to component axis
            oax = [0, 1, 2]
            oax.remove(self.sliding_axis)
            oax.remove(ax)
            oax = oax[0]

            # fabrication direction
            fdir = self.mesh.fab_directions[n]

            # check so that that part is not removed anyways...
            # i.e. if the whole bottom row in that direction is of other material
            ind = [0, 0, 0]
            ind[ax] = (1 - direction) * (self.voxel_res - 1)
            ind[self.sliding_axis] = fdir * (self.voxel_res - 1)
            free = True
            for i in range(self.voxel_res):
                ind[oax] = i
                val = self.mesh.voxel_matrix[tuple(ind)]
                if int(val) == n:
                    free = False
                    break

            if not free:
                # define start (pt0) and end (pt1) points of edge
                ind = [0, 0, 0]
                add = [0, 0, 0]
                ind[ax] = (1 - direction) * self.voxel_res
                ind[self.sliding_axis] = self.voxel_res * (1 - fdir) + (2 * fdir - 1) * lay_num
                i_pt = get_index(ind, add, self.voxel_res)
                pt0 = get_vertex(i_pt, self.joint_verts[n], self.vertex_num)
                ind[oax] = self.voxel_res
                i_pt = get_index(ind, add, self.voxel_res)
                pt1 = get_vertex(i_pt, self.joint_verts[n], self.vertex_num)

                # offset edge line by radius of millingbit
                dir_vec = normalize(pt0 - pt1)
                sax_vec = [0, 0, 0]
                sax_vec[self.sliding_axis] = 2 * fdir - 1
                off_vec = rotate_vector_around_axis(dir_vec, sax_vec, math.radians(90))
                off_vec = (2 * direction - 1) * self.fab.vradius * off_vec
                pt0 = pt0 + off_vec
                pt1 = pt1 + off_vec

                # Write to milling_verts
                mverts = [MillVertex(pt0), MillVertex(pt1)]

        return mverts

    def offset_verts(self, neighbor_vectors, neighbor_vectors_a, neighbor_vectors_b, verts, lay_num, n):
        outline = []
        corner_artifacts = []

        fdir = self.mesh.fab_directions[n]

        test_first = True
        for i, rv in enumerate(list(verts)):  # browse each vertex in the outline

            # remove verts with neighbor count 2 #OK
            if rv.region_count == 2 and rv.block_count == 2: continue  # redundant
            if rv.block_count == 0: continue  # redundant
            if rv.ind[0] < 0 or rv.ind[0] > self.voxel_res: continue  # out of bounds
            if rv.ind[1] < 0 or rv.ind[1] > self.voxel_res: continue  # out of bounds

            # add vertex information #OK
            ind = rv.ind.copy()
            ind.insert(self.sliding_axis, (self.voxel_res - 1) * (1 - fdir) + (2 * fdir - 1) * lay_num)
            add = [0, 0, 0]
            add[self.sliding_axis] = 1 - fdir
            i_pt = get_index(ind, add, self.voxel_res)
            pt = get_vertex(i_pt, self.joint_verts[n], self.vertex_num)

            # move vertex according to boundary condition <---needs to be updated
            off_vecs = []
            if rv.block_count == 1:
                nind = tuple(np.argwhere(rv.neighbors == 1)[0])
                off_vecs.append(-neighbor_vectors[nind])
            if rv.region_count == 1 and rv.free_count != 3:
                nind = tuple(np.argwhere(rv.neighbors == 0)[0])
                off_vecs.append(neighbor_vectors[nind])
                if np.any(rv.flat_neighbor_values == -2):
                    nind = tuple(np.argwhere(rv.neighbor_values == -2)[0])
                    off_vecs.append(neighbor_vectors[nind])

            off_vec = np.average(off_vecs, axis=0)
            # check if it is an outer corner that should be rounded
            rounded = False
            if rv.region_count == 3:  # outer corner, check if it should be rounded or not
                # check if this outer corner correspond to an inner corner of another material
                for n2 in range(self.timber_count):
                    if n2 == n: continue
                    cnt = np.sum(rv.flat_neighbor_values == n2)
                    if cnt == 3:
                        rounded = True
                    elif cnt == 2:
                        # Check if it is a diagonal
                        dia1 = rv.neighbor_values[0][0] == rv.neighbor_values[1][1]
                        dia2 = rv.neighbor_values[0][1] == rv.neighbor_values[1][0]
                        if dia1 or dia2:
                            rounded = True
            if rounded:
                nind = tuple(np.argwhere(rv.neighbors == 1)[0])
                off_vec_a = -neighbor_vectors_a[nind]
                off_vec_b = -neighbor_vectors_b[nind]
                le2 = math.sqrt(math.pow(2 * np.linalg.norm(off_vec_a + off_vec_b), 2) - math.pow(2 * self.fab.vradius,
                                                                                                  2)) - np.linalg.norm(
                    off_vec_a)
                off_vec_a2 = set_vector_length(off_vec_a, le2)
                off_vec_b2 = set_vector_length(off_vec_b, le2)

                # define end points and the center point of the arc
                pt1 = pt + off_vec_a - off_vec_b2
                pt2 = pt + off_vec_b - off_vec_a2
                pts = [pt1, pt2]
                ctr = pt - off_vec_a - off_vec_b  # arc center

                # Reorder pt1 and pt2
                if len(outline) > 0:  # if it is not the first point in the outline
                    ppt = outline[-1].pt
                    v1 = pt1 - ppt
                    v2 = pt2 - ppt
                    ang1 = angle_between(v1, off_vec_b)  # should be 0 if order is already good
                    ang2 = angle_between(v2, off_vec_b)  # should be more than 0
                    if ang1 > ang2: pts.reverse()
                outline.append(MillVertex(pts[0], is_arc=True, arc_ctr=ctr))
                outline.append(MillVertex(pts[1], is_arc=True, arc_ctr=ctr))

                # Extreme case where corner is very rounded and everything is not cut
                dist = np.linalg.norm(pt - ctr)
                if dist > self.fab.vdiam and lay_num < self.voxel_res - 1:
                    artifact = []
                    v0 = self.fab.vdiam * normalize(pt + off_vec - pts[0])
                    v1 = self.fab.vdiam * normalize(pt + off_vec - pts[1])
                    vp = self.fab.vradius * normalize(pts[1] - pts[0])
                    pts3 = [pts[0] - vp + v0, pt + 2 * off_vec, pts[1] + vp + v1]

                    while np.linalg.norm(pts3[2] - pts3[0]) > self.fab.vdiam:
                        pts3[0] += vp
                        pts3[1] += -off_vec
                        pts3[2] += -vp

                        for j in range(3): artifact.append(MillVertex(pts3[j]))

                        pts3.reverse()
                        vp = -vp
                    if len(artifact) > 0:
                        corner_artifacts.append(artifact)

            else:  # other corner
                pt = pt + off_vec
                outline.append(MillVertex(pt))
            if len(outline) > 2 and outline[0].is_arc and test_first:
                # if the previous one was an arc and it was the first point of the outline,
                # so we couldn't verify the order of the points
                # we might need to retrospectively switch order of the arc points
                npt = outline[2].pt
                d1 = np.linalg.norm(outline[0].pt - npt)
                d2 = np.linalg.norm(outline[1].pt - npt)
                if d1 < d2: outline[0], outline[1] = outline[1], outline[0]
                test_first = False

        return outline, corner_artifacts

    def get_milling_end_points(self, n, last_z):
        verts = []
        mverts = []

        r = g = b = tx = ty = 0.0

        fdir = self.mesh.fab_directions[n]

        origin_vert = [0, 0, 0]
        origin_vert[self.sliding_axis] = last_z

        extra_zheight = 15 / self.ratio
        above_origin_vert = [0, 0, 0]
        above_origin_vert[self.sliding_axis] = last_z - (2 * fdir - 1) * extra_zheight

        mverts.append(MillVertex(origin_vert, is_traversing=True))
        mverts.append(MillVertex(above_origin_vert, is_traversing=True))
        verts.extend([origin_vert[0], origin_vert[1], origin_vert[2], r, g, b, tx, ty])
        verts.extend([above_origin_vert[0], above_origin_vert[1], above_origin_vert[2], r, g, b, tx, ty])

        return verts, mverts

    def get_layered_vertices(self, outline, n, lay_num, no_z, dep):
        verts = []
        mverts = []

        r = g = b = tx = ty = 0.0

        fdir = self.mesh.fab_directions[n]

        # add startpoint
        start_vert = [outline[0].x, outline[0].y, outline[0].z]
        safe_height = outline[0].pt[self.sliding_axis] - (2 * fdir - 1) * (lay_num * self.voxel_sizes[self.sliding_axis] + 2 * dep)
        start_vert[self.sliding_axis] = safe_height
        mverts.append(MillVertex(start_vert, is_traversing=True))
        verts.extend([start_vert[0], start_vert[1], start_vert[2], r, g, b, tx, ty])
        if lay_num != 0:
            start_vert2 = [outline[0].x, outline[0].y, outline[0].z]
            safe_height2 = outline[0].pt[self.sliding_axis] - (2 * fdir - 1) * dep
            start_vert2[self.sliding_axis] = safe_height2
            mverts.append(MillVertex(start_vert2, is_traversing=True))
            verts.extend([start_vert2[0], start_vert2[1], start_vert2[2], r, g, b, tx, ty])

        # add layers with Z-height
        # set start number (one layer earlier if first layer)
        if lay_num == 0:
            stn = 0
        else:
            stn = 1

        # set end number (one layer more if last layer and not sliding direction aligned component)
        if lay_num == self.voxel_res - 1 and self.sliding_axis != self.fixed_sides.sides[n][0].ax:
            enn = no_z + 2
        else:
            enn = no_z + 1
        if self.increm_depth:
            enn += 1
            seg_props = get_segment_proportions(outline)
        else:
            seg_props = [1.0] * len(outline)
        # calculate depth for increm_depth setting

        for num in range(stn, enn):
            if self.increm_depth and num == enn - 1: seg_props = [0.0] * len(outline)
            for i, (mv, sp) in enumerate(zip(outline, seg_props)):
                pt = [mv.x, mv.y, mv.z]
                pt[self.sliding_axis] += (2 * fdir - 1) * (num - 1 + sp) * dep
                if mv.is_arc:
                    ctr = [mv.arc_ctr[0], mv.arc_ctr[1], mv.arc_ctr[2]]
                    ctr[self.sliding_axis] += (2 * fdir - 1) * (num - 1 + sp) * dep
                    mverts.append(MillVertex(pt, is_arc=True, arc_ctr=ctr))
                else:
                    mverts.append(MillVertex(pt))
                if i > 0:
                    pmv = outline[i - 1]
                if i > 0 and is_connected_arc(mv, pmv):
                    ppt = [pmv.x, pmv.y, pmv.z]
                    ppt[self.sliding_axis] += (2 * fdir - 1) * (num - 1 + sp) * dep
                    pctr = [pmv.arc_ctr[0], pmv.arc_ctr[1], pmv.arc_ctr[2]]
                    pctr[self.sliding_axis] += (2 * fdir - 1) * (num - 1 + sp) * dep
                    arc_pts = arc_points(ppt, pt, pctr, ctr, joint_type.sliding_axis, math.radians(5))
                    for arc_pt in arc_pts: verts.extend([arc_pt[0], arc_pt[1], arc_pt[2], r, g, b, tx, ty])
                else:
                    verts.extend([pt[0], pt[1], pt[2], r, g, b, tx, ty])

            outline.reverse()

def normalize(v: ArrayLike) -> ArrayLike:
    norm = np.linalg.norm(v)  # norm can return float or ndarray
    if norm == 0:
        return v
    else:
        return v / norm


def mat_from_fields(hfs: list,
                    ax: int) -> ZeroArray:  # duplicated function - also exists in Geometries, mat is numpy zeroes
    dim = len(hfs[0])
    mat = np.zeros(shape=(dim, dim, dim))
    for i in range(dim):
        for j in range(dim):
            for k in range(dim):
                ind = [i, j]
                ind3d = ind.copy()
                ind3d.insert(ax, k)
                ind3d = tuple(ind3d)
                ind2d = tuple(ind)
                h = 0
                for n, hf in enumerate(hfs):
                    if k < hf[ind2d]:
                        mat[ind3d] = n
                    else:
                        mat[ind3d] = n + 1
    mat = np.array(mat)
    return mat


def angle_between(vector_1: ArrayLike, vector_2: ArrayLike) -> DegreeArray:
    unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
    unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
    dot_product = np.dot(unit_vector_1, unit_vector_2)
    angle = np.arccos(dot_product)
    return angle


# noinspection PyDefaultArgument
def rotate_vector_around_axis(vec: list = [3, 5, 0], axis: list = [4, 4, 1], theta: float = 1.2) -> DotProduct:
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










def get_region_outline_vertices(reg_inds, lay_mat, org_lay_mat, pad_loc, n):
    # also duplicate verts on diagonal
    reg_verts = []
    for i in range(lay_mat.shape[0] + 1):
        for j in range(lay_mat.shape[1] + 1):
            ind = [i, j]
            neigbors, neighbor_values = get_neighbors_in_out(ind, reg_inds, lay_mat, org_lay_mat, n)
            neigbors = np.array(neigbors)
            abs_ind = ind.copy()
            ind[0] -= pad_loc[0][0]
            ind[1] -= pad_loc[1][0]
            if np.any(neigbors.flatten() == 0) and not np.all(
                    neigbors.flatten() == 0):  # some but not all region neighbors
                dia1 = neigbors[0][1] == neigbors[1][0]
                dia2 = neigbors[0][0] == neigbors[1][1]
                if np.sum(neigbors.flatten() == 0) == 2 and np.sum(
                        neigbors.flatten() == 1) == 2 and dia1 and dia2:  # diagonal detected
                    other_indices = np.argwhere(neigbors == 0)
                    for oind in other_indices:
                        oneigbors = copy.deepcopy(neigbors)
                        oneigbors[tuple(oind)] = 1
                        oneigbors = np.array(oneigbors)
                        reg_verts.append(RegionVertex(ind, abs_ind, oneigbors, neighbor_values, dia=True))
                else:  # normal situation
                    if any_minus_one_neighbor(ind, lay_mat):
                        mon = True
                    else:
                        mon = False
                    reg_verts.append(RegionVertex(ind, abs_ind, neigbors, neighbor_values, minus_one_neighbor=mon))
    return reg_verts


# noinspection PyChainedComparisons
def get_diff_neighbors(mat2, inds, val):
    new_inds = list(inds)
    for ind in inds:
        for ax in range(2):
            for direction in range(-1, 2, 2):
                ind2 = ind.copy()
                ind2[ax] += direction
                if ind2[ax] >= 0 and ind2[ax] < mat2.shape[ax]:
                    val2 = mat2[tuple(ind2)]
                    if val2 == val or val2 == -1: continue
                    unique = True
                    for ind3 in new_inds:
                        if ind2[0] == ind3[0] and ind2[1] == ind3[1]:
                            unique = False
                            break
                    if unique: new_inds.append(ind2)
    if len(new_inds) > len(inds):
        new_inds = get_diff_neighbors(mat2, new_inds, val)
    return new_inds








def set_starting_vert(verts):
    first_i = None
    second_i = None
    for i, rv in enumerate(verts):
        if rv.block_count > 0:
            if rv.free_count > 0:
                first_i = i
            else:
                second_i = i
    if first_i is None:
        first_i = second_i
    if first_i is None: first_i = 0
    verts.insert(0, verts[first_i])
    verts.pop(first_i + 1)
    return verts


def get_sublist_of_ordered_verts(verts):
    ord_verts = []

    # Start ordered verts with the first item (simultaneously remove from main list)
    ord_verts.append(verts[0])
    verts.remove(verts[0])

    browse_num = len(verts)
    for i in range(browse_num):
        found_next = False
        # try all directions to look for next vertex
        for vax in range(2):
            for vdir in range(-1, 2, 2):
                # check if there is an available vertex
                next_ind = ord_verts[-1].ind.copy()
                next_ind[vax] += vdir
                next_rv = None
                for rv in verts:
                    if rv.ind == next_ind:
                        if len(ord_verts) > 1 and rv.ind == ord_verts[-2].ind: break  # prevent going back
                        # check so that it is not crossing a blocked region etc
                        # 1) from point of view of previous point
                        p_neig = ord_verts[-1].neighbors
                        vaxval = int(0.5 * (vdir + 1))
                        nind0 = [0, 0]
                        nind0[vax] = vaxval
                        nind1 = [1, 1]
                        nind1[vax] = vaxval
                        ne0 = p_neig[nind0[0]][nind0[1]]
                        ne1 = p_neig[nind1[0]][nind1[1]]
                        if ne0 != 1 and ne1 != 1: continue  # no block
                        if int(0.5 * (ne0 + 1)) == int(0.5 * (ne1 + 1)): continue  # trying to cross blocked material
                        # 2) from point of view of point currently tested
                        nind0 = [0, 0]
                        nind0[vax] = 1 - vaxval
                        nind1 = [1, 1]
                        nind1[vax] = 1 - vaxval
                        ne0 = rv.neighbors[nind0[0]][nind0[1]]
                        ne1 = rv.neighbors[nind1[0]][nind1[1]]
                        if ne0 != 1 and ne1 != 1: continue  # no block
                        if int(0.5 * (ne0 + 1)) == int(0.5 * (ne1 + 1)): continue  # trying to cross blocked material
                        # If you made it here, you found the next vertex!
                        found_next = True
                        ord_verts.append(rv)
                        verts.remove(rv)
                        break
                if found_next: break
            if found_next: break
        if found_next: continue

    # check if outline is closed by ckecing if endpoint finds startpoint

    closed = False
    if len(ord_verts) > 3:  # needs to be at least 4 verts to be able to close
        start_ind = np.array(ord_verts[0].ind.copy())
        end_ind = np.array(ord_verts[-1].ind.copy())
        diff_ind = start_ind - end_ind  # reverse?
        if len(np.argwhere(diff_ind == 0)) == 1:  # difference only in one axis
            vax = np.argwhere(diff_ind != 0)[0][0]
            if abs(diff_ind[vax]) == 1:  # difference is only one step
                vdir = diff_ind[vax]
                # check so that it is not crossing a blocked region etc
                p_neig = ord_verts[-1].neighbors
                vaxval = int(0.5 * (vdir + 1))
                nind0 = [0, 0]
                nind0[vax] = vaxval
                nind1 = [1, 1]
                nind1[vax] = vaxval
                ne0 = p_neig[nind0[0]][nind0[1]]
                ne1 = p_neig[nind1[0]][nind1[1]]
                if ne0 == 1 or ne1 == 1:
                    if int(0.5 * (ne0 + 1)) != int(0.5 * (ne1 + 1)):
                        # If you made it here, you found the next vertex!
                        closed = True

    return ord_verts, verts, closed


def set_vector_length(vec, new_norm):
    norm = np.linalg.norm(vec)
    vec = vec / norm
    vec = new_norm * vec
    return vec


# TODO - unused function
def get_outline(joint_type, verts, lay_num, n):
    fdir = joint_type.mesh.fab_directions[n]
    outline = []
    for rv in verts:
        ind = rv.ind.copy()
        ind.insert(joint_type.sliding_axis, (joint_type.voxel_res - 1) * (1 - fdir) + (2 * fdir - 1) * lay_num)
        add = [0, 0, 0]
        add[joint_type.sliding_axis] = 1 - fdir
        i_pt = get_index(ind, add, joint_type.voxel_res)
        pt = get_vertex(i_pt, joint_type.joint_verts[n], joint_type.vertex_num)
        outline.append(MillVertex(pt))
    return outline


def get_vertex(index, verts, n):
    x = verts[n * index]
    y = verts[n * index + 1]
    z = verts[n * index + 2]
    return np.array([x, y, z])





def get_segment_proportions(outline):
    olen = 0
    slens = []
    sprops = []

    for i in range(1, len(outline)):
        ppt = outline[i - 1].pt
        pt = outline[i].pt
        dist = np.linalg.norm(pt - ppt)
        slens.append(dist)
        olen += dist

    olen2 = 0
    sprops.append(0.0)
    for slen in slens:
        olen2 += slen
        sprop = olen2 / olen
        sprops.append(sprop)

    return sprops




    # add endpoint
    end_vert = [outline[0].x, outline[0].y, outline[0].z]
    end_vert[joint_type.sliding_axis] = safe_height
    mverts.append(MillVertex(end_vert, is_traversing=True))
    verts.extend([end_vert[0], end_vert[1], end_vert[2], r, g, b, tx, ty])

    return verts, mverts


def any_minus_one_neighbor(ind, lay_mat):
    # TODO: what is this flag exactly?
    flag = False
    for add0 in range(-1, 1, 1):
        temp = []
        temp2 = []
        for add1 in range(-1, 1, 1):
            # Define neighbor index to test
            nind = [ind[0] + add0, ind[1] + add1]
            # If test index is within bounds
            if np.all(np.array(nind) >= 0) and nind[0] < lay_mat.shape[0] and nind[1] < lay_mat.shape[1]:
                # If the value is -1
                if lay_mat[tuple(nind)] == -1:
                    flag = True
                    break
    return flag


def get_neighbors_in_out(ind, reg_inds, lay_mat, org_lay_mat, n):
    in_out = []
    values = []
    for add0 in range(-1, 1, 1):
        temp = []
        temp2 = []
        for add1 in range(-1, 1, 1):

            # Define neighbor index to test
            nind = [ind[0] + add0, ind[1] + add1]

            # FIND TYPE
            neighbor_type = -1
            val = None
            # Check if this index is in the list of region-included indices
            for rind in reg_inds:
                if rind[0] == nind[0] and rind[1] == nind[1]:
                    neighbor_type = 0  # in region
                    break
            if neighbor_type != 0:
                # If there are out of bound indices they are free
                if np.any(np.array(nind) < 0) or nind[0] >= lay_mat.shape[0] or nind[1] >= lay_mat.shape[1]:
                    neighbor_type = 2  # free
                    val = -1
                elif lay_mat[tuple(nind)] < 0:
                    neighbor_type = 2  # free
                    val = -2
                else:
                    neighbor_type = 1  # blocked

            if val == None:
                val = org_lay_mat[tuple(nind)]

            temp.append(neighbor_type)
            temp2.append(val)
        in_out.append(temp)
        values.append(temp2)
    return in_out, values


def filleted_points(pt, one_voxel, off_dist, ax, n):
    ##
    addx = (one_voxel[0] * 2 - 1) * off_dist
    addy = (one_voxel[1] * 2 - 1) * off_dist
    ###
    pt1 = pt.copy()
    add = [addx, -addy]
    add.insert(ax, 0)
    pt1[0] += add[0]
    pt1[1] += add[1]
    pt1[2] += add[2]
    #
    pt2 = pt.copy()
    add = [-addx, addy]
    add.insert(ax, 0)
    pt2[0] += add[0]
    pt2[1] += add[1]
    pt2[2] += add[2]
    #
    if n % 2 == 1: pt1, pt2 = pt2, pt1
    return [pt1, pt2]

# TODO - unused function
def is_additional_outer_corner(joint_type, rv, ind, ax, n):
    outer_corner = False
    if rv.region_count == 1 and rv.block_count == 1:
        other_fixed_sides = joint_type.fixed_sides.sides.copy()
        other_fixed_sides.pop(n)
        for sides in other_fixed_sides:
            for side in sides:
                if side.ax == ax: continue
                axes = [0, 0, 0]
                axes[side.ax] = 1
                axes.pop(ax)
                oax = axes.index(1)
                not_oax = axes.index(0)
                if rv.ind[oax] == odir * joint_type.voxel_res:
                    if rv.ind[not_oax] != 0 and rv.ind[not_oax] != joint_type.voxel_res:
                        outer_corner = True
                        break
            if outer_corner: break
    return outer_corner



