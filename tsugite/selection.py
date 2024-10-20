import numpy as np
import pyrr
import copy
import math

from fixed_sides import FixedSide
from utils import *

def angle_between_with_direction(v0: ArrayLike, v1: ArrayLike) -> DegreeFloat:
    v0 = v0 / np.linalg.norm(v0)
    v1 = v1 / np.linalg.norm(v1)
    angle = np.math.atan2(np.linalg.det([v0, v1]), np.dot(v0, v1))
    return math.degrees(angle)


def unitize(v: ArrayLike) -> ArrayLike:
    uv = v / np.linalg.norm(v)
    return uv


def get_same_height_neighbors(hfield: list, inds: list) -> list:
    dim = len(hfield)
    val = hfield[tuple(inds[0])]        # cast the inds[0] to tuple
    new_inds = list(inds)
    for ind in inds:
        for ax in range(2):
            for direction in range(-1, 2, 2):
                ind2 = ind.copy()
                ind2[ax] += direction
                if np.all(ind2 >= 0) and np.all(ind2 < dim):
                    val2 = hfield[tuple(ind2)]
                    if val2 == val:
                        unique = True
                        for ind3 in new_inds:
                            if ind2[0] == ind3[0] and ind2[1] == ind3[1]:
                                unique = False
                                break
                        if unique: new_inds.append(ind2)
    if len(new_inds) > len(inds):
        new_inds = get_same_height_neighbors(hfield, new_inds)
    return new_inds


# noinspection PyAttributeOutsideInit,PyChainedComparisons
class Selection:
    def __init__(self, geom) -> None:
        self.state: TimberState = -1
        self.suggestions_state: HoveringState = -1
        self.gallery_state: HoveringState = -1
        self.geom = geom
        self.n = self.x = self.y = None
        self.refresh = False
        self.shift = False
        self.faces = []
        self.new_fixed_sides_for_display = None
        self.val = 0

    def update_pick(self, x, y, n, direction) -> None:
        self.n = n
        self.x = x
        self.y = y
        self.direction = direction
        if self.x is not None and self.y is not None:
            if self.shift:
                self.faces = get_same_height_neighbors(self.geom.height_fields[n - direction],
                                                       [np.array([self.x, self.y])])
            else:
                self.faces = [np.array([self.x, self.y])]

    # to check
    def start_pull(self, mouse_pos) -> None:
        self.state = 2
        self.start_pos = np.array([mouse_pos[0], -mouse_pos[1]])
        self.start_height = self.geom.height_fields[self.n - self.direction][self.x][self.y]
        self.geom.joint_type.combine_and_buffer_indices()  # for selection area

    def end_pull(self) -> None:
        if self.val != 0: self.geom.edit_height_fields(self.faces, self.current_height, self.n, self.direction)
        self.state = -1
        self.refresh = True

    def edit(self, mouse_pos: list[int, int], screen_xrot, screen_yrot, w: int = 1600, h: int = 1600) -> None:
        self.current_pos = np.array([mouse_pos[0], -mouse_pos[1]])
        self.current_height = self.start_height

        ## Mouse vector
        mouse_vec = np.array(self.current_pos - self.start_pos)
        mouse_vec = mouse_vec.astype(float)
        mouse_vec[0] = 2 * mouse_vec[0] / w
        mouse_vec[1] = 2 * mouse_vec[1] / h

        ## Sliding direction vector
        # sdir_vec = [0, 0, 0]
        sdir_vec = np.copy(self.geom.joint_type.pos_vecs[self.geom.joint_type.sliding_axis])
        rot_x = pyrr.Matrix33.from_x_rotation(screen_xrot)
        rot_y = pyrr.Matrix33.from_y_rotation(screen_yrot)
        sdir_vec = np.dot(sdir_vec, rot_x * rot_y)
        sdir_vec = np.delete(sdir_vec, 2)  # delete Z-value

        ## Calculate angle between mouse vector and sliding direction vector
        cosang = np.dot(mouse_vec, sdir_vec)  # Negative / positive depending on direction
        val = int(np.linalg.norm(mouse_vec) / np.linalg.norm(sdir_vec) + 0.5)
        if cosang is not None and cosang < 0: val = -val
        if self.start_height + val > self.geom.joint_type.voxel_res:
            val = self.geom.joint_type.voxel_res - self.start_height
        elif self.start_height + val < 0:
            val = -self.start_height
        self.current_height = self.start_height + val
        self.val = int(val)

    def start_move(self, mouse_pos: list[int, int], h: int = 1600) -> None:
        self.state = 12
        self.start_pos = np.array([mouse_pos[0], h - mouse_pos[1]])
        self.new_fixed_sides = self.geom.joint_type.fixed_sides.sides[self.n]
        self.new_fixed_sides_for_display = self.geom.joint_type.fixed_sides.sides[self.n]
        self.geom.joint_type.combine_and_buffer_indices()  # for move preview outline

    def end_move(self) -> None:
        self.geom.joint_type.update_component_position(self.new_fixed_sides, self.n)
        self.state = -1
        self.new_fixed_sides_for_display = None

    def move(self, mouse_pos: list[int, int], screen_xrot, screen_yrot, w: int = 1600, h: int = 1600) \
            -> None:  # actually move OR rotate
        sax = self.geom.joint_type.sliding_axis
        noc = self.geom.joint_type.timber_count
        self.new_fixed_sides = copy.deepcopy(self.geom.joint_type.fixed_sides.sides[self.n])
        self.new_fixed_sides_for_display = copy.deepcopy(self.geom.joint_type.fixed_sides.sides[self.n])
        self.current_pos = np.array([mouse_pos[0], h - mouse_pos[1]])
        ## Mouse vector
        mouse_vec = np.array(self.current_pos - self.start_pos)
        mouse_vec = mouse_vec.astype(float)
        mouse_vec[0] = 2 * mouse_vec[0] / w
        mouse_vec[1] = 2 * mouse_vec[1] / h
        ## Check that the move distance is above some threshold
        move_dist = np.linalg.norm(mouse_vec)
        if move_dist > 0.01:
            ## Get component direction vector
            comp_ax = self.geom.joint_type.fixed_sides.sides[self.n][0].ax  # component axis
            comp_dir = self.geom.joint_type.fixed_sides.sides[self.n][0].direction
            comp_len = 2.5 * (2 * comp_dir - 1) * self.geom.joint_type.component_size
            comp_vec = comp_len * unitize(self.geom.joint_type.pos_vecs[comp_ax])
            ## Flatten vector to screen
            rot_x = pyrr.Matrix33.from_x_rotation(screen_xrot)
            rot_y = pyrr.Matrix33.from_y_rotation(screen_yrot)
            comp_vec = np.dot(comp_vec, rot_x * rot_y)
            comp_vec = np.delete(comp_vec, 2)  # delete Z-value
            ## Calculate angle between mouse vector and component vector
            ang = angle_between_with_direction(mouse_vec, comp_vec)
            oax = None
            absang = abs(ang) % 180
            if absang > 45 and absang < 135:  # Timber rotation mode
                # Check plane of rotating by checking which axis the vector is more aligned to
                other_axes = [0, 1, 2]
                other_axes.pop(comp_ax)
                # The axis that is flatter to the screen will be processed
                maxlen = 0
                for i in range(len(other_axes)):
                    other_vec = [0, 0, 0]
                    other_vec[other_axes[i]] = 1
                    ## Flatten vector to screen
                    other_vec = np.dot(other_vec, rot_x * rot_y)
                    other_vec = np.delete(other_vec, 2)  # delete Z-value
                    ## Check length
                    other_length = np.linalg.norm(other_vec)
                    if other_length > maxlen:
                        maxlen = other_length
                        oax = other_axes[i]
                # check rotation direction
                clockwise = True
                if ang < 0: clockwise = False
                # screen_direction
                lax = [0, 1, 2]
                lax.remove(comp_ax)
                lax.remove(oax)
                lax = lax[0]
                screen_dir = 1
                screen_vec = self.geom.joint_type.pos_vecs[lax]
                screen_vec = np.dot(screen_vec, rot_x * rot_y)
                if screen_vec[2] < 0: screen_dir = -1
                ###
                self.new_fixed_sides_for_display = []
                for i in range(len(self.geom.joint_type.fixed_sides.sides[self.n])):
                    ndir = self.geom.joint_type.fixed_sides.sides[self.n][i].direction
                    ordered = False
                    if comp_ax < oax and oax - comp_ax == 1:
                        ordered = True
                    elif oax < comp_ax and comp_ax - oax == 2:
                        ordered = True
                    if (clockwise and not ordered) or (not clockwise and ordered):
                        ndir = 1 - ndir
                    if screen_dir > 0: ndir = 1 - ndir
                    side = FixedSide(oax, ndir)
                    self.new_fixed_sides_for_display.append(side)
                    if side.ax == sax and side.direction == 0 and self.n != 0: blocked = True; break
                    if side.ax == sax and side.direction == 1 and self.n != noc - 1: blocked = True; break
            else:  # Timber moving mode
                length_ratio = np.linalg.norm(mouse_vec) / np.linalg.norm(comp_vec)
                side_num = len(self.geom.joint_type.fixed_sides.sides[self.n])
                if side_num == 1 and absang > 135:  # currently L
                    if length_ratio < 0.5:  # moved just a bit, L to T
                        self.new_fixed_sides_for_display = [FixedSide(comp_ax, 0), FixedSide(comp_ax, 1)]
                    elif length_ratio < 2.0:  # moved a lot, L to other L
                        self.new_fixed_sides_for_display = [FixedSide(comp_ax, 1 - comp_dir)]
                elif side_num == 2:  # currently T
                    if absang > 135:
                        self.new_fixed_sides_for_display = [FixedSide(comp_ax, 1)]  # positive direction
                    else:
                        # negative direction self.new_fixed_sides_for_display = [FixedSide(comp_ax,0)]
                        self.new_fixed_sides_for_display = [FixedSide(comp_ax,
                                                                      0)]
            # check if the direction is blocked
            blocked = False
            for side in self.new_fixed_sides_for_display:
                if side.is_unique(self.geom.joint_type.fixed_sides.sides[self.n]):
                    if side.is_unique(self.geom.joint_type.fixed_sides.unblocked):
                        blocked = True
            if blocked:
                all_same = True
                for side in self.new_fixed_sides_for_display:
                    if side.is_unique(self.geom.joint_type.fixed_sides.sides[self.n]):
                        all_same = False
                if all_same: blocked = False
            if not blocked: self.new_fixed_sides = self.new_fixed_sides_for_display
        if not np.equal(self.geom.joint_type.fixed_sides.sides[self.n], np.array(self.new_fixed_sides_for_display)).all():
            # for move/rotate preview outline # can't you display this by transformation instead?
            self.geom.joint_type.combine_and_buffer_indices()
