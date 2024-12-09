import pyrr
import numpy as np

import OpenGL.GL as GL
import OpenGL.GL.shaders as GLSH

from core.utils import Utils

from buffer import ElementProperties
from view_settings import ViewSettings
from joint_types import JointType


# noinspection PyAttributeOutsideInit,PyChainedComparisons
class Display:
    def __init__(self, _glWidget, joint_type: JointType) -> None:       # parent is GLWidget
        self._glWidget = _glWidget
        self.joint_type = joint_type
        self.view = ViewSettings()
        self.create_color_shaders()
        self.create_texture_shaders()

    def update(self) -> None:
        self.init_shader(self.col_shader_program)
        if (self.view.open_joint and self.view.open_ratio < self.joint_type.timber_count - 1) or (
                not self.view.open_joint and self.view.open_ratio > 0):
            self.view.set_joint_opening_distance(self.joint_type.timber_count)

    # extension declaration is needed for versions lower than GLSL 4.30
    def create_color_shaders(self) -> None:
        vertex_shader = """
        #extension GL_ARB_explicit_attrib_location : require
        #extension GL_ARB_explicit_uniform_location : require
        layout(location = 0) in vec3 position;
        layout(location = 1) in vec3 color;
        layout(location = 2) in vec2 inTexCoords;
        layout(location = 3) uniform mat4 transform;
        layout(location = 4) uniform mat4 translate;
        layout(location = 5) uniform vec3 myColor;
        out vec3 newColor;
        out vec2 outTexCoords;
        void main()
        {
            gl_Position = transform* translate* vec4(position, 1.0f);
            newColor = myColor;
            outTexCoords = inTexCoords;
        }
        """

        fragment_shader = """
        in vec3 newColor;
        in vec2 outTexCoords;
        out vec4 outColor;
        uniform sampler2D samplerTex;
        void main()
        {
            outColor = vec4(newColor, 1.0);
        }
        """

        # Compiling the color shaders
        self.col_shader_program = Utils.initialize_program(vertex_shader, fragment_shader)

    def create_texture_shaders(self) -> None:

        vertex_shader = """
        #extension GL_ARB_explicit_attrib_location : require
        #extension GL_ARB_explicit_uniform_location : require
        layout(location = 0) in vec3 position;
        layout(location = 1) in vec3 color;
        layout(location = 2) in vec2 inTexCoords;
        layout(location = 3) uniform mat4 transform;
        layout(location = 4) uniform mat4 translate;
        out vec3 newColor;
        out vec2 outTexCoords;
        void main()
        {
            gl_Position = transform* translate* vec4(position, 1.0f);
            newColor = color;
            outTexCoords = inTexCoords;
        }
        """

        fragment_shader = """
        in vec3 newColor;
        in vec2 outTexCoords;
        out vec4 outColor;
        uniform sampler2D samplerTex;
        void main()
        {
            outColor = texture(samplerTex, outTexCoords);
        }
        """

        # Compiling the shaders
        self.tex_shader_program = Utils.initialize_program(vertex_shader, fragment_shader)

    def init_shader(self, shader) -> None:
        GL.glUseProgram(shader)
        rot_x = pyrr.Matrix44.from_x_rotation(self.view.xrot)
        rot_y = pyrr.Matrix44.from_y_rotation(self.view.yrot)
        GL.glUniformMatrix4fv(3, 1, GL.GL_FALSE, rot_x * rot_y)

    def draw_geometries(self, geos, clear_depth_buffer=True, translation_vec=np.array([0, 0, 0])):
        # Define translation matrices for opening
        move_vec = [0, 0, 0]
        move_vec[self.joint_type.sliding_axis] = self.view.open_ratio * self.joint_type.component_size
        move_vec = np.array(move_vec)
        moves = []
        
        for n in range(self.joint_type.timber_count):
            tot_move_vec = (2 * n + 1 - self.joint_type.timber_count) / (self.joint_type.timber_count - 1) * move_vec
            move_mat = pyrr.matrix44.create_from_translation(tot_move_vec + translation_vec)
            moves.append(move_mat)

        if clear_depth_buffer:
            GL.glClear(GL.GL_DEPTH_BUFFER_BIT)

        for geo in geos:
            if geo is None: continue
            if self.view.hidden[geo.n]: continue
            GL.glUniformMatrix4fv(4, 1, GL.GL_FALSE, moves[geo.n])
            GL.glDrawElements(geo.draw_type, geo.count, GL.GL_UNSIGNED_INT, GL.ctypes.c_void_p(4 * geo.start_index))

    def resizeGL(self, width, height):
        GL.glViewport(0, 0, width, height)
        GL.glMatrixMode(GL.GL_PROJECTION)
        GL.glLoadIdentity()
        aspect = width / float(height)

        # aspect = 1.267
        # oratio = aspect
        # if height * oratio > width:
        #     height = width / oratio
        #     # height = w / oratio
        # else:
        #     width = height * oratio

        # print(aspect)
        GL.gluPerspective(45.0, aspect, 1.0, 100.0)
        GL.glMatrixMode(GL.GL_MODELVIEW)

    def draw_geometries_with_excluded_area(self, show_geos, screen_geos, translation_vec=np.array([0, 0, 0])):
        # Define translation matrices for opening
        move_vec = [0, 0, 0]
        move_vec[self.joint_type.sliding_axis] = self.view.open_ratio * self.joint_type.component_size
        move_vec = np.array(move_vec)
        moves = []
        moves_show = []

        for n in range(self.joint_type.timber_count):
            tot_move_vec = (2 * n + 1 - self.joint_type.timber_count) / (self.joint_type.timber_count - 1) * move_vec
            move_mat = pyrr.matrix44.create_from_translation(tot_move_vec)
            moves.append(move_mat)
            move_mat_show = pyrr.matrix44.create_from_translation(tot_move_vec + translation_vec)
            moves_show.append(move_mat_show)

        GL.glClear(GL.GL_DEPTH_BUFFER_BIT)
        GL.glDisable(GL.GL_DEPTH_TEST)
        GL.glColorMask(GL.GL_FALSE, GL.GL_FALSE, GL.GL_FALSE, GL.GL_FALSE)
        GL.glEnable(GL.GL_STENCIL_TEST)
        GL.glStencilFunc(GL.GL_ALWAYS, 1, 1)
        GL.glStencilOp(GL.GL_REPLACE, GL.GL_REPLACE, GL.GL_REPLACE)
        GL.glDepthRange(0.0, 0.9975)

        for geo in show_geos:
            if geo is None: continue
            if self.view.hidden[geo.n]: continue
            GL.glUniformMatrix4fv(4, 1, GL.GL_FALSE, moves_show[geo.n])
            GL.glDrawElements(geo.draw_type, geo.count, GL.GL_UNSIGNED_INT, GL.ctypes.c_void_p(4 * geo.start_index))

        GL.glEnable(GL.GL_DEPTH_TEST)
        GL.glStencilFunc(GL.GL_EQUAL, 1, 1)
        GL.glStencilOp(GL.GL_KEEP, GL.GL_KEEP, GL.GL_KEEP)
        GL.glDepthRange(0.0025, 1.0)

        for geo in screen_geos:
            if geo is None: continue
            if self.view.hidden[geo.n]: continue
            GL.glUniformMatrix4fv(4, 1, GL.GL_FALSE, moves[geo.n])
            GL.glDrawElements(geo.draw_type, geo.count, GL.GL_UNSIGNED_INT, GL.ctypes.c_void_p(4 * geo.start_index))
        GL.glDisable(GL.GL_STENCIL_TEST)
        GL.glColorMask(GL.GL_TRUE, GL.GL_TRUE, GL.GL_TRUE, GL.GL_TRUE)
        GL.glDepthRange(0.0, 0.9975)
        
        for geo in show_geos:
            if geo is None: continue
            if self.view.hidden[geo.n]: continue
            GL.glUniformMatrix4fv(4, 1, GL.GL_FALSE, moves_show[geo.n])
            GL.glDrawElements(geo.draw_type, geo.count, GL.GL_UNSIGNED_INT, GL.ctypes.c_void_p(4 * geo.start_index))

    def pick(self, xpos, ypos, height):

        if not self.view.gallery:
            ######################## COLOR SHADER ###########################
            GL.glUseProgram(self.col_shader_program)
            GL.glClearColor(1.0, 1.0, 1.0, 1.0)  # white
            GL.glEnable(GL.GL_DEPTH_TEST)
            GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT | GL.GL_STENCIL_BUFFER_BIT)
            GL.glMatrixMode(GL.GL_MODELVIEW)
            rot_x = pyrr.Matrix44.from_x_rotation(self.view.xrot)
            rot_y = pyrr.Matrix44.from_y_rotation(self.view.yrot)
            GL.glUniformMatrix4fv(3, 1, GL.GL_FALSE, rot_x * rot_y)
            GL.glPolygonOffset(1.0, 1.0)

            ########################## Draw colorful top faces ##########################

            # Draw colorful geometries
            col_step = 1.0 / (2 + 2 * self.joint_type.voxel_res * self.joint_type.voxel_res)
            for n in range(self.joint_type.timber_count):
                col = np.zeros(3, dtype=np.float64)
                col[n % 3] = 1.0
                if n > 2: col[(n + 1) % self.joint_type.voxel_res] = 1.0
                GL.glUniform3f(5, col[0], col[1], col[2])
                self.draw_geometries([self.joint_type.mesh.indices_fpick_not_top[n]], clear_depth_buffer=False)
                if n == 0 or n == self.joint_type.timber_count - 1:
                    mos = 1
                else:
                    mos = 2
                # mos is "number of sides"
                for m in range(mos):
                    # Draw top faces
                    for i in range(self.joint_type.voxel_res * self.joint_type.voxel_res):
                        col -= col_step
                        GL.glUniform3f(5, col[0], col[1], col[2])
                        top = ElementProperties(GL.GL_QUADS, 4,
                                                self.joint_type.mesh.indices_fpick_top[n].start_index + mos * 4 * i + 4 * m,
                                                n)
                        self.draw_geometries([top], clear_depth_buffer=False)

        ############### Read pixel color at mouse position ###############
        mouse_pixel = GL.glReadPixelsub(xpos, height - ypos, 1, 1, GL.GL_RGB, outputType=None)[0][0]
        mouse_pixel = np.array(mouse_pixel)
        pick_n = pick_d = pick_x = pick_y = None
        self.joint_type.mesh.select.suggestions_state = -1
        self.joint_type.mesh.select.gallery_state = -1
        if not self.view.gallery:
            if xpos > self._glWidget.width - self._glWidget.wstep:  # suggestion side
                if ypos > 0 and ypos < self._glWidget.height:
                    index = int(ypos / self._glWidget.hstep)
                    if self.joint_type.mesh.select.suggestions_state != index:
                        self.joint_type.mesh.select.suggestions_state = index

            elif not np.all(mouse_pixel == 255):  # not white / background
                non_zeros = np.where(mouse_pixel != 0)
                if len(non_zeros) > 0:
                    if len(non_zeros[0] > 0):
                        pick_n = non_zeros[0][0]
                        if len(non_zeros[0]) > 1:
                            pick_n = pick_n + self.joint_type.voxel_res
                            if mouse_pixel[0] == mouse_pixel[2]: pick_n = 5
                        val = 255 - mouse_pixel[non_zeros[0][0]]
                        i = int(0.5 + val * (2 + 2 * self.joint_type.voxel_res * self.joint_type.voxel_res) / 255) - 1
                        if i >= 0:
                            pick_x = (int(i / self.joint_type.voxel_res)) % self.joint_type.voxel_res
                            pick_y = i % self.joint_type.voxel_res
                        pick_d = 0
                        if pick_n == self.joint_type.timber_count - 1:
                            pick_d = 1
                        elif int(i / self.joint_type.voxel_res) >= self.joint_type.voxel_res:
                            pick_d = 1
                        # print(pick_n,pick_d,pick_x,pick_y)
        """
        else: #gallerymode
            if xpos>0 and xpos<2000 and ypos>0 and ypos<1600:
                i = int(xpos/400)
                j = int(ypos/400)
                index = i*4+j
                mesh.select.gallery_state=index
                mesh.select.state = -1
                mesh.select.suggestions_state = -1
        """
        ### Update selection
        if pick_x is not None and pick_d is not None and pick_y is not None and pick_n is not None:

            ### Initialize selection
            new_pos = False
            if pick_x != self.joint_type.mesh.select.x or pick_y != self.joint_type.mesh.select.y or pick_n != self.joint_type.mesh.select.n or pick_d != self.joint_type.mesh.select.direction or self.joint_type.mesh.select.refresh:
                self.joint_type.mesh.select.update_pick(pick_x, pick_y, pick_n, pick_d)
                self.joint_type.mesh.select.refresh = False
                self.joint_type.mesh.select.state = 0  # hovering

        elif pick_n is not None:
            self.joint_type.mesh.select.state = 10  # hovering component body
            self.joint_type.mesh.select.update_pick(pick_x, pick_y, pick_n, pick_d)
        else:
            self.joint_type.mesh.select.state = -1
        GL.glClearColor(1.0, 1.0, 1.0, 1.0)

    def selected(self):
        ################### Draw top face that is currently being hovered ##########
        # Draw base face (hovered)
        if self.joint_type.mesh.select.state == 0:
            GL.glClear(GL.GL_DEPTH_BUFFER_BIT | GL.GL_STENCIL_BUFFER_BIT)
            GL.glUniform3f(5, 0.2, 0.2, 0.2)  # dark grey
            G1 = self.joint_type.mesh.indices_fpick_not_top
            for face in self.joint_type.mesh.select.faces:
                if self.joint_type.mesh.select.n == 0 or self.joint_type.mesh.select.n == self.joint_type.timber_count - 1:
                    mos = 1
                else:
                    mos = 2
                index = int(self.joint_type.voxel_res * face[0] + face[1])
                top = ElementProperties(GL.GL_QUADS, 4, self.joint_type.mesh.indices_fpick_top[
                    self.joint_type.mesh.select.n].start_index + mos * 4 * index + (mos - 1) * 4 * self.joint_type.mesh.select.direction,
                                        self.joint_type.mesh.select.n)
                # top = ElementProperties(GL_QUADS, 4, mesh.indices_fpick_top[mesh.select.n].start_index+4*index, mesh.select.n)
                self.draw_geometries_with_excluded_area([top], G1)
        # Draw pulled face
        if self.joint_type.mesh.select.state == 2:
            GL.glPushAttrib(GL.GL_ENABLE_BIT)
            GL.glLineWidth(3)
            GL.glEnable(GL.GL_LINE_STIPPLE)
            GL.glLineStipple(2, 0xAAAA)
            for val in range(0, abs(self.joint_type.mesh.select.val) + 1):
                if self.joint_type.mesh.select.val < 0: val = -val
                pulled_vec = [0, 0, 0]
                pulled_vec[self.joint_type.sliding_axis] = val * self.joint_type.voxel_sizes[self.joint_type.sliding_axis]
                self.draw_geometries([self.joint_type.mesh.outline_selected_faces], translation_vec=np.array(pulled_vec))
            GL.glPopAttrib()

    def difference_suggestion(self, index):
        GL.glPushAttrib(GL.GL_ENABLE_BIT)
        # draw faces of additional part
        # glUniform3f(5, 1.0, 1.0, 1.0) # white
        # for n in range(self.joint_type.timber_count):
        #    G0 = [self.joint_type.suggestions[index].indices_fall[n]]
        #    G1 = self.joint_type.mesh.indices_fall
        #    self.draw_geometries_with_excluded_area(G0,G1)

        # draw faces of subtracted part
        # glUniform3f(5, 1.0, 0.5, 0.5) # pink/red
        # for n in range(self.joint_type.timber_count):
        #    G0 = [self.joint_type.mesh.indices_fall[n]]
        #    G1 = self.joint_type.suggestions[index].indices_fall
        #    self.draw_geometries_with_excluded_area(G0,G1)

        # draw outlines
        GL.glUniform3f(5, 0.0, 0.0, 0.0)  # black
        GL.glLineWidth(3)
        GL.glEnable(GL.GL_LINE_STIPPLE)
        GL.glLineStipple(2, 0xAAAA)
        for n in range(self.joint_type.timber_count):
            G0 = [self.joint_type.suggestions[index].indices_lns[n]]
            G1 = self.joint_type.suggestions[index].indices_fall
            self.draw_geometries_with_excluded_area(G0, G1)
        GL.glPopAttrib()

    def moving_rotating(self):
        # Draw moved_rotated component before action is finalized
        if self.joint_type.mesh.select.state == 12 and self.joint_type.mesh.outline_selected_component is not None:
            GL.glPushAttrib(GL.GL_ENABLE_BIT)
            GL.glLineWidth(3)
            GL.glEnable(GL.GL_LINE_STIPPLE)
            GL.glLineStipple(2, 0xAAAA)
            self.draw_geometries([self.joint_type.mesh.outline_selected_component])
            GL.glPopAttrib()

    def joint_geometry(self, mesh=None, lw=3, hidden=True, zoom=False):

        if mesh is None: mesh = self.joint_type.mesh

        ############################# Draw hidden lines #############################
        GL.glClear(GL.GL_DEPTH_BUFFER_BIT)
        GL.glUniform3f(5, 0.0, 0.0, 0.0)  # black
        GL.glPushAttrib(GL.GL_ENABLE_BIT)
        GL.glLineWidth(1)
        GL.glLineStipple(3, 0xAAAA)  # dashed line
        GL.glEnable(GL.GL_LINE_STIPPLE)
        if hidden and self.view.show_hidden_lines:
            for n in range(mesh.joint_type.timber_count):
                G0 = [mesh.indices_lns[n]]
                G1 = [mesh.indices_fall[n]]
                self.draw_geometries_with_excluded_area(G0, G1)
        GL.glPopAttrib()

        ############################ Draw visible lines #############################
        for n in range(mesh.joint_type.timber_count):
            if not mesh.main_mesh or (
                    mesh.eval.interlocks[n] and self.view.show_feedback) or not self.view.show_feedback:
                GL.glUniform3f(5, 0.0, 0.0, 0.0)  # black
                GL.glLineWidth(lw)
            else:
                GL.glUniform3f(5, 1.0, 0.0, 0.0)  # red
                GL.glLineWidth(lw + 1)
            G0 = [mesh.indices_lns[n]]
            G1 = mesh.indices_fall
            self.draw_geometries_with_excluded_area(G0, G1)

        if mesh.main_mesh:
            ################ When joint is fully open, draw dahsed lines ################
            if hidden and not self.view.hidden[0] and not self.view.hidden[1] and self.view.open_ratio == 1 + 0.5 * (
                    mesh.joint_type.timber_count - 2):
                GL.glUniform3f(5, 0.0, 0.0, 0.0)  # black
                GL.glPushAttrib(GL.GL_ENABLE_BIT)
                GL.glLineWidth(2)
                GL.glLineStipple(1, 0x00FF)
                GL.glEnable(GL.GL_LINE_STIPPLE)
                G0 = mesh.indices_open_lines
                G1 = mesh.indices_fall
                self.draw_geometries_with_excluded_area(G0, G1)
                GL.glPopAttrib()

    def end_grains(self):
        self.init_shader(self.tex_shader_program)
        G0 = self.joint_type.mesh.indices_fend
        G1 = self.joint_type.mesh.indices_not_fend
        self.draw_geometries_with_excluded_area(G0, G1)
        self.init_shader(self.col_shader_program)

    def unfabricatable(self):
        col = [1.0, 0.8, 0.5]  # orange
        GL.glUniform3f(5, col[0], col[1], col[2])
        for n in range(self.joint_type.timber_count):
            if not self.joint_type.mesh.eval.fab_direction_ok[n]:
                G0 = [self.joint_type.mesh.indices_fall[n]]
                G1 = []
                for n2 in range(self.joint_type.timber_count):
                    if n2 != n: G1.append(self.joint_type.mesh.indices_fall[n2])
                self.draw_geometries_with_excluded_area(G0, G1)

    def unconnected(self):
        # 1. Draw hidden geometry
        col = [1.0, 0.8, 0.7]  # light red orange
        GL.glUniform3f(5, col[0], col[1], col[2])
        for n in range(self.joint_type.mesh.joint_type.timber_count):
            if not self.joint_type.mesh.eval.connected[n]:
                self.draw_geometries([self.joint_type.mesh.indices_not_fcon[n]])

        # 1. Draw visible geometry
        col = [1.0, 0.2, 0.0]  # red orange
        GL.glUniform3f(5, col[0], col[1], col[2])
        G0 = self.joint_type.mesh.indices_not_fcon
        G1 = self.joint_type.mesh.indices_fcon
        self.draw_geometries_with_excluded_area(G0, G1)

    def unbridged(self):
        # Draw colored faces when unbridged
        for n in range(self.joint_type.timber_count):
            if not self.joint_type.mesh.eval.bridged[n]:
                for m in range(2):  # browse the two parts
                    # a) Unbridge part 1
                    col = self.view.unabridged_colors[n][m]
                    GL.glUniform3f(5, col[0], col[1], col[2])
                    G0 = [self.joint_type.mesh.indices_not_fbridge[n][m]]
                    G1 = [self.joint_type.mesh.indices_not_fbridge[n][1 - m],
                          self.joint_type.mesh.indices_fall[1 - n],
                          self.joint_type.mesh.indices_not_fcon[n]]  # needs reformulation for 3 components
                    self.draw_geometries_with_excluded_area(G0, G1)

    def checker(self):
        # 1. Draw hidden geometry
        GL.glUniform3f(5, 1.0, 0.2, 0.0)  # red orange
        GL.glLineWidth(8)
        for n in range(self.joint_type.mesh.joint_type.timber_count):
            if self.joint_type.mesh.eval.checker[n]:
                self.draw_geometries([self.joint_type.mesh.indices_chess_lines[n]])
        GL.glUniform3f(5, 0.0, 0.0, 0.0)  # back to black

    def arrows(self):
        # glClear(GL_DEPTH_BUFFER_BIT)
        GL.glUniform3f(5, 0.0, 0.0, 0.0)
        ############################## Direction arrows ################################
        for n in range(self.joint_type.timber_count):
            if self.joint_type.mesh.eval.interlocks[n]:
                GL.glUniform3f(5, 0.0, 0.0, 0.0)  # black
            else:
                GL.glUniform3f(5, 1.0, 0.0, 0.0)  # red
            GL.glLineWidth(3)
            G1 = self.joint_type.mesh.indices_fall
            G0 = self.joint_type.mesh.indices_arrows[n]
            d0 = 2.55 * self.joint_type.component_size
            d1 = 1.55 * self.joint_type.component_size
            if len(self.joint_type.fixed_sides.sides[n]) == 2: d0 = d1
            for side in self.joint_type.fixed_sides.sides[n]:
                vec = d0 * (2 * side.direction - 1) * self.joint_type.pos_vecs[side.ax] / np.linalg.norm(
                    self.joint_type.pos_vecs[side.ax])
                # draw_geometries_with_excluded_area(window,G0,G1,translation_vec=vec)
                self.draw_geometries(G0, translation_vec=vec)

    def nondurable(self):
        # 1. Draw hidden geometry
        col = [1.0, 1.0, 0.8]  # super light yellow
        GL.glUniform3f(5, col[0], col[1], col[2])
        for n in range(self.joint_type.timber_count):
            self.draw_geometries_with_excluded_area([self.joint_type.mesh.indices_fbrk[n]],
                                                    [self.joint_type.mesh.indices_not_fbrk[n]])

        # Draw visible geometry
        col = [1.0, 1.0, 0.4]  # light yellow
        GL.glUniform3f(5, col[0], col[1], col[2])
        self.draw_geometries_with_excluded_area(self.joint_type.mesh.indices_fbrk, self.joint_type.mesh.indices_not_fbrk)

    def milling_paths(self):
        if len(self.joint_type.mesh.indices_milling_path) == 0: self.view.show_milling_path = False
        if self.view.show_milling_path:
            cols = [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0], [1.0, 1.0, 0], [0.0, 1.0, 1.0], [1.0, 0, 1.0]]
            GL.glLineWidth(3)
            for n in range(self.joint_type.timber_count):
                if self.joint_type.mesh.eval.fab_direction_ok[n]:
                    GL.glUniform3f(5, cols[n][0], cols[n][1], cols[n][2])
                    self.draw_geometries([self.joint_type.mesh.indices_milling_path[n]])

    def resizeEvent(self, event):
        print(' resizeEvent')
        self.resize(self.width(), self.height())
