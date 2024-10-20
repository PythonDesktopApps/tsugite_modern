import time
from math import tan, pi

import PyQt5.QtWidgets as qtw
import PyQt5.QtGui as qtg
import PyQt5.QtCore as qtc
import PyQt5.QtOpenGL as qgl

import OpenGL.GL as GL  # imports start with gl

from joint_types import JointType
from geometries import Geometries
from display import Display


# noinspection PyAttributeOutsideInit
class GLWidget(qgl.QGLWidget):

    def __init__(self, main_window=None, *__args):
        # commennt for now, focus first on refactoring the actual code
        # fmt = qgl.QGLFormat()
        # fmt.setVersion(3, 3)
        # fmt.setProfile(qgl.QGLFormat.CoreProfile)
        # fmt.setSampleBuffers(True)
        # super().__init__(fmt, main_window, *__args)

        super().__init__(main_window, *__args)

        self.parent = main_window
        # self.setMinimumSize(800, 800)
        self.setMouseTracking(True)
        self.click_time = time.time()
        self.x = 0
        self.y = 0


    def initializeGL(self):

        self.print_system_info()

        self.gl_settings

        # widgets
        sliding_axis = self.parent.cmb_sliding_axis.currentIndex() # string x, y, z
        voxel_res = self.parent.spb_voxel_res.value() # int [2:5]
        angle = self.parent.spb_angle.value() # int [-80: 80]
        xdim = self.parent.spb_xdim.value() # float [10:150]
        ydim = self.parent.spb_ydim.value() # float [10:150]
        zdim = self.parent.spb_zdim.value() # float [10:150]
        milling_diam = self.parent.spb_milling_diam.value() # float [1:50]
        tolerances = self.parent.spb_tolerances.value() # float [0.15, 5]
        milling_speed = self.parent.spb_milling_speed.value() # int [100, 1000]
        spindle_speed = self.parent.spb_spindle_speed.value() # int [1000, 10000]
        alignment_axis = self.parent.cmb_alignment_axis.currentIndex() # str x-, y-, x+, y+
        increm_depth = self.parent.chk_increm_depth.isChecked() # bool
        arc_interp = self.parent.chk_arc_interp.isChecked() # bool

        if self.parent.rdo_gcode.isChecked():
            ext = "gcode"
        elif self.parent.rdo_nc.isChecked():
            ext = "nc"
        elif self.parent.rdo_sbp.isChecked():
            ext = "sbp"
        else:
            ext = "gcode"

        # joint_type and display objects are related to OpenGL hence initialized here
        # instead of the __init__
        self.joint_type = JointType(self, fs=[[[2, 0]], [[2, 1]]], sliding_axis=sliding_axis, voxel_res=voxel_res, angle=angle,
                                    timber_dims=[xdim, ydim, zdim], tolerances=tolerances, milling_diam=milling_diam,
                                    milling_speed=milling_speed, spindle_speed=spindle_speed, fab_ext=ext,
                                    alignment_axis=alignment_axis, increm_depth=increm_depth, arc_interp=arc_interp)

        self.display = Display(self, self.joint_type)

    def print_system_info(self):
        vendor = GL.glGetString(GL.GL_VENDOR).decode('utf-8')
        renderer = GL.glGetString(GL.GL_RENDERER).decode('utf-8')
        opengl = GL.glGetString(GL.GL_VERSION).decode('utf-8')
        glsl = GL.glGetString(GL.GL_SHADING_LANGUAGE_VERSION).decode('utf-8')

        result = ''.join(['Vendor: ', vendor, '\n',
                          'Renderer: ', renderer, '\n',
                          'OpenGL version supported: ', opengl, '\n',
                          'GLSL version supported: ', glsl])
        print(result)

    def gl_settings(self):
        # self.qglClearColor(qtg.QColor(255, 255, 255))
        GL.glClearColor(255, 255, 255, 1)
        GL.glEnable(GL.GL_DEPTH_TEST)
        GL.glDepthFunc(GL.GL_LESS)
        # the shapes are basically behind the white background
        # if you enabled face culling, they will not show
        # GL.glEnable(GL.GL_CULL_FACE)

    def clear(self):
        # color it white for better visibility
        GL.glClearColor(255, 255, 255, 1)
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
        
    def resizeGL(self, w, h):
        def perspective(fovY, aspect, zNear, zFar):
            fH = tan(fovY / 360. * pi) * zNear
            fW = fH * aspect
            GL.glFrustum(-fW, fW, -fH, fH, zNear, zFar)

        # oratio = self.width() /self.height()
        ratio = 1.267

        if h * ratio > w:
            h = round(w / ratio)

        else:
            w = round(h * ratio)

        GL.glViewport(0, 0, w, h)
        GL.glMatrixMode(GL.GL_PROJECTION)
        GL.glLoadIdentity()

        # inner function
        perspective(45.0, ratio, 1, 1000)

        GL.glMatrixMode(GL.GL_MODELVIEW)
        self.width = w
        self.height = h
        self.wstep = int(0.5 + w / 5)
        self.hstep = int(0.5 + h / 4)

    def paintGL(self):
        self.clear()
        
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT | GL.GL_STENCIL_BUFFER_BIT)
        # glViewport(0,0,self.width-self.wstep,self.height)
        GL.glLoadIdentity()

        self.display.update()
        # ortho = np.multiply(np.array((-2, +2, -2, +2), dtype=float), self.zoomFactor)
        # glOrtho(ortho[0], ortho[1], ortho[2], ortho[3], 4.0, 15.0)

        GL.glViewport(0, 0, self.width - self.wstep, self.height)
        # glLoadIdentity()
        # Color picking / editing
        # Pick faces -1: nothing, 0: hovered, 1: adding, 2: pulling

        # Draw back buffer colors
        if not self.joint_type.mesh.select.state == 2 and not self.joint_type.mesh.select.state == 12:
            self.display.pick(self.x, self.y, self.height)
            GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT | GL.GL_STENCIL_BUFFER_BIT)
        elif self.joint_type.mesh.select.state == 2:  # Edit joint geometry
            self.joint_type.mesh.select.edit([self.x, self.y], self.display.view.xrot, self.display.view.yrot, w=self.width,
                                             h=self.height)
        elif self.joint_type.mesh.select.state == 12:  # Edit timber orientation/position
            self.joint_type.mesh.select.move([self.x, self.y], self.display.view.xrot, self.display.view.yrot)

        # Display main geometry
        self.display.end_grains()
        if self.display.view.show_feedback:
            self.display.unfabricatable()
            self.display.nondurable()
            self.display.unconnected()
            self.display.unbridged()
            self.display.checker()
            self.display.arrows()
            show_area = False  # <--replace by checkbox...
            if show_area:
                self.display.area()
        self.display.joint_geometry()

        if self.joint_type.mesh.select.suggestions_state >= 0:
            index = self.joint_type.mesh.select.suggestions_state
            if len(self.joint_type.suggestions) > index:
                self.display.difference_suggestion(index)

        # Display editing in action
        self.display.selected()
        self.display.moving_rotating()

        # Display milling paths
        self.display.milling_paths()

        # Suggestions
        if self.display.view.show_suggestions:
            for i in range(len(self.joint_type.suggestions)):
                # hquater = self.height / 4
                # wquater = self.width / 5
                GL.glViewport(self.width - self.wstep, self.height - self.hstep * (i + 1), self.wstep, self.hstep)
                GL.glLoadIdentity()
                if i == self.joint_type.mesh.select.suggestions_state:
                    GL.glEnable(GL.GL_SCISSOR_TEST)
                    GL.glScissor(self.width - self.wstep, self.height - self.hstep * (i + 1), self.wstep, self.hstep)
                    GL.glClearDepth(1.0)
                    GL.glClearColor(0.9, 0.9, 0.9, 1.0)  # light grey
                    GL.glClear(GL.GL_COLOR_BUFFER_BIT)
                    GL.glDisable(GL.GL_SCISSOR_TEST)
                self.display.joint_geometry(mesh=self.joint_type.suggestions[i], lw=2, hidden=False)

    def mousePressEvent(self, e):
        if e.button() == qtc.Qt.LeftButton:
            if time.time() - self.click_time < 0.2:
                self.display.view.open_joint = not self.display.view.open_joint
            elif self.joint_type.mesh.select.state == 0:  # face hovered
                self.joint_type.mesh.select.start_pull([self.parent.scaling * e.x(), self.parent.scaling * e.y()])
            elif self.joint_type.mesh.select.state == 10:  # body hovered
                self.joint_type.mesh.select.start_move([self.parent.scaling * e.x(), self.parent.scaling * e.y()],
                                                       h=self.height)

            # SUGGESTION PICK
            elif self.joint_type.mesh.select.suggestions_state >= 0:
                index = self.joint_type.mesh.select.suggestions_state
                if len(self.joint_type.suggestions) > index:
                    self.joint_type.mesh = Geometries(self.joint_type,
                                                      height_fields=self.joint_type.suggestions[index].height_fields)
                    self.joint_type.suggestions = []
                    self.joint_type.combine_and_buffer_indices()
                    self.joint_type.mesh.select.suggestions_state = -1
            # GALLERY PICK -- not implemented currently
            # elif joint_type.mesh.select.gallery_state>=0:
            #    index = joint_type.mesh.select.gallery_state
            #    if index<len(joint_type.gallery_figures):
            #        joint_type.mesh = Geometries(joint_type,height_fields=joint_type.gallery_figures[index].height_fields)
            #        joint_type.gallery_figures = []
            #        view_opt.gallery=False
            #        joint_type.gallery_start_index = -20
            #        joint_type.combine_and_buffer_indices()
            else:
                self.click_time = time.time()
        elif e.button() == qtc.Qt.RightButton:
            self.display.view.start_rotation_xy(self.parent.scaling * e.x(), self.parent.scaling * e.y())

    def mouseMoveEvent(self, e):
        self.x = self.parent.scaling * e.x()
        self.y = self.parent.scaling * e.y()
        if self.display.view.dragged:
            self.display.view.update_rotation_xy(self.x, self.y)

    def mouseReleaseEvent(self, e):
        if e.button() == qtc.Qt.LeftButton:
            if self.joint_type.mesh.select.state == 2:  # face pulled
                self.joint_type.mesh.select.end_pull()
            elif self.joint_type.mesh.select.state == 12:  # body moved
                self.joint_type.mesh.select.end_move()
        elif e.button() == qtc.Qt.RightButton:
            self.display.view.end_rotation()
