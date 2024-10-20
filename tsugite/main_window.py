import math
import numpy as np
import PyQt5.QtWidgets as qtw
import PyQt5.QtGui as qtg
import PyQt5.QtCore as qtc

from PyQt5.uic import loadUi
from PyQt5.QtCore import pyqtSlot

from gl_widget import *
from utils import *


class MainWindow(qtw.QMainWindow):

    def __init__(self, *args):
        super().__init__(*args)
        self.scaling = self.devicePixelRatioF()

        loadUi('Tsugite.ui', self)
        self.setupUi()

        self.title = "Tsugite"
        self.filename = get_untitled_filename("Untitled", "tsu", "_")
        self.setWindowTitle(self.filename.split(os.sep)[-1] + " - " + self.title)
        self.setWindowIcon(qtg.QIcon("tsugite_icon.png"))

        self.glWidget = GLWidget(self)

        self.hly_gl.addWidget(self.glWidget)

        self.statusBar = qtw.QStatusBar()
        self.setStatusBar(self.statusBar)
        self.statusBar.showMessage(
            "To open and close the joint: PRESS 'Open/close joint' button or DOUBLE-CLICK anywhere inside the window.")

        timer = qtc.QTimer(self)
        timer.setInterval(20)  # period, in milliseconds
        timer.timeout.connect(self.glWidget.updateGL)
        timer.start()

    def setupUi(self):
        # get opengl window size - not really needed
        # self.x_range = [10, 500]
        # self.y_range = [10, 500]

        # note that the widgets are made attribute to be reused again
        # ---Design
        self.btn_open_close_joint = self.findChild(qtw.QPushButton, "btn_open_close_joint")
        self.btn_open_close_joint.clicked.connect(self.open_close_joint)

        self.chk_show_feedback = self.findChild(qtw.QCheckBox, "chk_show_feedback")
        self.chk_show_feedback.stateChanged.connect(self.set_feedback_view)

        # suggestions
        self.cmb_sliding_axis = self.findChild(qtw.QComboBox, "cmb_sliding_axis")
        self.cmb_sliding_axis.currentTextChanged.connect(self.change_sliding_axis)

        self.spb_timber_count = self.findChild(qtw.QSpinBox, "spb_timber_count")
        self.spb_timber_count.valueChanged.connect(self.change_number_of_timbers)

        self.spb_voxel_res = self.findChild(qtw.QSpinBox, "spb_voxel_res")
        self.spb_voxel_res.valueChanged.connect(self.change_resolution)

        self.spb_angle = self.findChild(qtw.QDoubleSpinBox, "spb_angle")
        self.spb_angle.valueChanged.connect(self.set_angle_of_intersection)

        self.chk_timber_dim_cubic = self.findChild(qtw.QCheckBox, "chk_timber_dim_cubic")
        self.chk_timber_dim_cubic.stateChanged.connect(self.set_all_timber_same)

        self.spb_xdim = self.findChild(qtw.QDoubleSpinBox, "spb_xdim")
        self.spb_xdim.valueChanged.connect(self.set_timber_X)

        self.spb_ydim = self.findChild(qtw.QDoubleSpinBox, "spb_ydim")
        self.spb_ydim.valueChanged.connect(self.set_timber_Y)

        self.spb_zdim = self.findChild(qtw.QDoubleSpinBox, "spb_zdim")
        self.spb_zdim.valueChanged.connect(self.set_timber_Z)

        self.btn_randomize = self.findChild(qtw.QPushButton, "btn_randomize")
        self.btn_randomize.clicked.connect(self.randomize_geometry)

        self.btn_clear = self.findChild(qtw.QPushButton, "btn_clear")
        self.btn_clear.clicked.connect(self.clear_geometry)

        # gallery
        # ---Fabrication
        self.spb_milling_diam = self.findChild(qtw.QDoubleSpinBox, "spb_milling_diam")
        self.spb_milling_diam.valueChanged.connect(self.set_milling_bit_diameter)

        self.spb_tolerances = self.findChild(qtw.QDoubleSpinBox, "spb_tolerances")
        self.spb_tolerances.valueChanged.connect(self.set_fab_tolerance)

        self.spb_milling_speed = self.findChild(qtw.QSpinBox, "spb_milling_speed")
        self.spb_milling_speed.valueChanged.connect(self.set_fab_speed)

        self.spb_spindle_speed = self.findChild(qtw.QSpinBox, "spb_spindle_speed")
        self.spb_spindle_speed.valueChanged.connect(self.set_fab_spindle_speed)

        self.cmb_alignment_axis = self.findChild(qtw.QComboBox, "cmb_alignment_axis")
        self.cmb_alignment_axis.currentTextChanged.connect(self.set_milling_path_axis_alignment)

        self.chk_increm_depth = self.findChild(qtw.QCheckBox, "chk_increm_depth")
        self.chk_increm_depth.stateChanged.connect(self.set_incremental)

        self.chk_arc_interp = self.findChild(qtw.QCheckBox, "chk_arc_interp")
        self.chk_arc_interp.stateChanged.connect(self.set_interpolation)

        self.btn_show_milling_path = self.findChild(qtw.QPushButton, "btn_show_milling_path")
        self.btn_show_milling_path.clicked.connect(self.set_milling_path_view)

        self.btn_export_milling_path = self.findChild(qtw.QPushButton, "btn_export_milling_path")
        self.btn_export_milling_path.clicked.connect(self.export_gcode)

        self.rdo_gcode = self.findChild(qtw.QRadioButton, "rdo_gcode")
        self.rdo_gcode.toggled.connect(self.set_gcode_as_standard)

        self.rdo_nc = self.findChild(qtw.QRadioButton, "rdo_nc")
        self.rdo_nc.toggled.connect(self.set_nccode_as_standard)

        self.rdo_sbp = self.findChild(qtw.QRadioButton, "rdo_sbp")
        self.rdo_sbp.toggled.connect(self.set_sbp_as_standard)

        # ---MENU
        # ---File
        self.act_new = self.findChild(qtw.QAction, "act_new")
        self.act_new.triggered.connect(self.new_file)

        self.act_open = self.findChild(qtw.QAction, "act_open")
        self.act_open.triggered.connect(self.open_file)

        self.act_save = self.findChild(qtw.QAction, "act_save")
        self.act_save.triggered.connect(self.save_file)

        self.act_saveas = self.findChild(qtw.QAction, "act_saveas")
        self.act_saveas.triggered.connect(self.save_file_as)

        # ---View
        self.act_hidden = self.findChild(qtw.QAction, "act_hidden")
        self.act_hidden.triggered.connect(self.show_hide_hidden_lines)

        self.act_a = self.findChild(qtw.QAction, "act_a")
        self.act_a.triggered.connect(self.show_hide_timbers)

        self.act_b = self.findChild(qtw.QAction, "act_b")
        self.act_b.triggered.connect(self.show_hide_timbers)

        self.act_c = self.findChild(qtw.QAction, "act_c")
        self.act_c.triggered.connect(self.show_hide_timbers)

        self.act_d = self.findChild(qtw.QAction, "act_d")
        self.act_d.triggered.connect(self.show_hide_timbers)

        self.act_all = self.findChild(qtw.QAction, "act_all")
        self.act_all.triggered.connect(self.show_all_timbers)

        self.act_axo = self.findChild(qtw.QAction, "act_axo")
        self.act_axo.triggered.connect(self.set_standard_rotation)

        self.act_pln = self.findChild(qtw.QAction, "act_pln")
        self.act_pln.triggered.connect(self.set_closest_plane_rotation)

    @pyqtSlot()
    def open_close_joint(self):
        self.glWidget.display.view.open_joint = not self.glWidget.display.view.open_joint

    @pyqtSlot()
    def set_feedback_view(self):
        feedback_shown = self.chk_show_feedback.checkState()
        self.glWidget.display.view.show_feedback = feedback_shown

    @pyqtSlot()
    def change_sliding_axis(self):
        ax = self.cmb_sliding_axis.currentIndex()
        # the boolean component is not used
        _, msg = self.glWidget.joint_type.update_sliding_direction(ax)
        print(msg)

    @pyqtSlot()
    def change_number_of_timbers(self):
        val = self.spb_timber_count.value()
        self.glWidget.joint_type.update_number_of_components(val)

    @pyqtSlot()
    def change_resolution(self):
        val = self.spb_voxel_res.value()
        add = val - self.glWidget.joint_type.voxel_res
        self.glWidget.joint_type.update_dimension(add)

    @pyqtSlot()
    def set_angle_of_intersection(self):
        val = self.spb_angle.value()
        self.glWidget.joint_type.update_angle(val)

    @pyqtSlot()
    def set_timber_X(self):
        val = self.spb_xdim.value()
        mp = self.glWidget.display.view.show_milling_path

        # why this block only present in X voxel_res?
        if mp:
            self.glWidget.joint_type.create_and_buffer_vertices(milling_path=True)

        if self.chk_timber_dim_cubic.isChecked():
            self.glWidget.joint_type.update_timber_width_and_height([0, 1, 2], val, milling_path=mp)
            self.spb_ydim.setValue(val)
            self.spb_zdim.setValue(val)
        else:
            self.glWidget.joint_type.update_timber_width_and_height([0], val, milling_path=mp)

    @pyqtSlot()
    def set_timber_Y(self):
        val = self.spb_ydim.value()
        mp = self.glWidget.display.view.show_milling_path

        if self.chk_timber_dim_cubic.isChecked():
            self.glWidget.joint_type.update_timber_width_and_height([0, 1, 2], val, milling_path=mp)
            self.spb_xdim.setValue(val)
            self.spb_zdim.setValue(val)
        else:
            self.glWidget.joint_type.update_timber_width_and_height([1], val, milling_path=mp)

    @pyqtSlot()
    def set_timber_Z(self):
        val = self.spb_zdim.value()
        mp = self.glWidget.display.view.show_milling_path

        if self.chk_timber_dim_cubic.isChecked():
            self.glWidget.joint_type.update_timber_width_and_height([0, 1, 2], val, milling_path=mp)
            self.spb_xdim.setValue(val)
            self.spb_ydim.setValue(val)
        else:
            self.glWidget.joint_type.update_timber_width_and_height([2], val, milling_path=mp)

    @pyqtSlot()
    def set_all_timber_same(self):
        mp = self.glWidget.display.view.show_milling_path

        if self.chk_timber_dim_cubic.isChecked():
            val = self.glWidget.joint_type.real_timber_dims[0]
            self.glWidget.joint_type.update_timber_width_and_height([0, 1, 2], val, milling_path=mp)
            self.spb_ydim.setValue(val)
            self.spb_zdim.setValue(val)

    @pyqtSlot()
    def randomize_geometry(self):
        self.glWidget.joint_type.mesh.randomize_height_fields()

    @pyqtSlot()
    def clear_geometry(self):
        self.glWidget.joint_type.mesh.clear_height_fields()

    @pyqtSlot()
    def set_milling_bit_diameter(self):
        val = self.spb_milling_diam.value()
        self.glWidget.joint_type.fab.real_diam = val
        self.glWidget.joint_type.fab.radius = 0.5 * self.glWidget.joint_type.fab.real_diam - \
                                              self.glWidget.joint_type.fab.tolerances
        self.glWidget.joint_type.fab.diameter = 2 * self.glWidget.joint_type.fab.radius
        self.glWidget.joint_type.fab.vdiam = self.glWidget.joint_type.fab.diameter / self.glWidget.joint_type.ratio
        self.glWidget.joint_type.fab.vradius = self.glWidget.joint_type.fab.radius / self.glWidget.joint_type.ratio

        if self.glWidget.display.view.show_milling_path:
            self.glWidget.joint_type.create_and_buffer_vertices(milling_path=True)
            self.glWidget.joint_type.combine_and_buffer_indices(milling_path=True)

    @pyqtSlot()
    def set_fab_tolerance(self):
        val = self.spb_tolerances.value()
        self.glWidget.joint_type.fab.tolerances = val
        self.glWidget.joint_type.fab.radius = 0.5 * self.glWidget.joint_type.fab.real_diam - \
                                              self.glWidget.joint_type.fab.tolerances
        self.glWidget.joint_type.fab.diameter = 2 * self.glWidget.joint_type.fab.radius
        self.glWidget.joint_type.fab.vdiam = self.glWidget.joint_type.fab.diameter / self.glWidget.joint_type.ratio
        self.glWidget.joint_type.fab.vradius = self.glWidget.joint_type.fab.radius / self.glWidget.joint_type.ratio
        self.glWidget.joint_type.fab.vtolerances = self.glWidget.joint_type.fab.tolerances / self.glWidget.joint_type.ratio

        if self.glWidget.display.view.show_milling_path:
            self.glWidget.joint_type.create_and_buffer_vertices(milling_path=True)
            self.glWidget.joint_type.combine_and_buffer_indices(milling_path=True)

    @pyqtSlot()
    def set_fab_speed(self):
        val = self.spb_milling_speed.value()
        self.glWidget.joint_type.fab.milling_speed = val

    @pyqtSlot()
    def set_fab_spindle_speed(self):
        val = self.spb_spindle_speed.value()
        self.glWidget.joint_type.fab.spindle_speed = val

    @pyqtSlot()
    def set_milling_path_axis_alignment(self):
        val = self.cmb_alignment_axis.currentIndex()
        self.glWidget.joint_type.fab.alignment_axis = val

    @pyqtSlot()
    def set_incremental(self):
        val = self.chk_increm_depth.isChecked()
        self.glWidget.joint_type.increm_depth = val

    @pyqtSlot()
    def set_interpolation(self):
        val = self.chk_arc_interp.isChecked()
        self.glWidget.joint_type.fab.arc_interp = val

    @pyqtSlot()
    def set_milling_path_view(self):
        self.glWidget.display.view.show_milling_path = not self.glWidget.display.view.show_milling_path
        milling_path_showed = self.glWidget.display.view.show_milling_path
        self.glWidget.joint_type.create_and_buffer_vertices(milling_path=milling_path_showed)
        self.glWidget.joint_type.combine_and_buffer_indices(milling_path=milling_path_showed)

    @pyqtSlot()
    def export_gcode(self):
        if not self.glWidget.display.view.show_milling_path:
            self.glWidget.display.view.show_milling_path = True
            self.glWidget.joint_type.create_and_buffer_vertices(milling_path=True)
            self.glWidget.joint_type.combine_and_buffer_indices(milling_path=True)
        self.glWidget.joint_type.fab.export_gcode(filename_tsu=self.filename)

    @pyqtSlot()
    def set_gcode_as_standard(self):
        if self.rdo_gcode.isChecked():
            self.glWidget.joint_type.fab.export_ext = "gcode"

    @pyqtSlot()
    def set_nccode_as_standard(self):
        if self.rdo_nc.isChecked():
            self.glWidget.joint_type.fab.export_ext = "nc"

    @pyqtSlot()
    def set_sbp_as_standard(self):
        if self.rdo_sbp.isChecked():
            self.glWidget.joint_type.fab.export_ext = "sbp"

    @pyqtSlot()
    def new_file(self):
        self.filename = get_untitled_filename("Untitled", "tsu", "_")
        self.setWindowTitle(self.filename.split("/")[-1] + " - " + self.title)
        self.glWidget.display.view.show_milling_path = False
        self.glWidget.joint_type.reset()
        self.set_ui_values()
        self.show_all_timbers()

    @pyqtSlot()
    def open_file(self):
        filename, _ = qtw.QFileDialog.getOpenFileName(filter="Tsugite files (*.tsu)")
        if filename != '':
            self.filename = filename
            self.setWindowTitle(self.filename.split("/")[-1] + " - " + self.title)
            self.chk_timber_dim_cubic.setChecked(False)
            self.glWidget.joint_type.open(self.filename)
            self.set_ui_values()

    @pyqtSlot()
    def save_file(self):
        self.glWidget.joint_type.save(self.filename)

    @pyqtSlot()
    def save_file_as(self):
        filename, _ = qtw.QFileDialog.getSaveFileName(filter="Tsugite files (*.tsu)")
        if filename != '':
            self.filename = filename
            self.setWindowTitle(self.filename.split("/")[-1] + " - " + self.title)
            self.glWidget.joint_type.save(self.filename)

    @pyqtSlot()
    def show_hide_hidden_lines(self):
        self.glWidget.display.view.show_hidden_lines = self.act_hidden.isChecked()

    @pyqtSlot()
    def show_hide_timbers(self):
        actions = [
            self.act_a,
            self.act_b,
            self.act_c,
            self.act_d
        ]

        for i, action in enumerate(actions):
            timber_is_checked = action.isChecked()
            self.glWidget.display.view.hidden[i] = not timber_is_checked

    @pyqtSlot()
    def show_all_timbers(self):
        actions = [
            self.act_a,
            self.act_b,
            self.act_c,
            self.act_d
        ]

        for i, action in enumerate(actions):
            timber_is_checked = action.isChecked()
            self.glWidget.display.view.hidden[i] = False

    @pyqtSlot()
    def set_standard_rotation(self):
        self.glWidget.display.view.xrot = 0.8
        self.glWidget.display.view.yrot = 0.4

    @pyqtSlot()
    def set_closest_plane_rotation(self):
        xrot = self.glWidget.display.view.xrot
        yrot = self.glWidget.display.view.yrot
        nang = 0.5 * math.pi
        xrot = round(xrot / nang, 0) * nang
        yrot = round(yrot / nang, 0) * nang
        self.glWidget.display.view.xrot = xrot
        self.glWidget.display.view.yrot = yrot

    def set_ui_values(self):
        self.cmb_sliding_axis.setCurrentIndex(self.glWidget.joint_type.sliding_axis)
        self.spb_timber_count.setValue(self.glWidget.joint_type.timber_count)
        self.spb_voxel_res.setValue(self.glWidget.joint_type.voxel_res)
        self.spb_angle.setValue(self.glWidget.joint_type.angle)
        self.spb_xdim.setValue(self.glWidget.joint_type.real_timber_dims[0])
        self.spb_ydim.setValue(self.glWidget.joint_type.real_timber_dims[1])
        self.spb_zdim.setValue(self.glWidget.joint_type.real_timber_dims[2])

        if np.max(self.glWidget.joint_type.real_timber_dims) == np.min(self.glWidget.joint_type.real_timber_dims):
            self.chk_timber_dim_cubic.setChecked(True)
        else:
            self.chk_timber_dim_cubic.setChecked(False)

        self.spb_milling_diam.setValue(self.glWidget.joint_type.fab.real_diam)
        self.spb_tolerances.setValue(self.glWidget.joint_type.fab.tolerances)
        self.spb_milling_speed.setValue(self.glWidget.joint_type.fab.milling_speed)
        self.spb_spindle_speed.setValue(self.glWidget.joint_type.fab.spindle_speed)
        self.chk_increm_depth.setChecked(self.glWidget.joint_type.increm_depth)
        self.chk_arc_interp.setChecked(self.glWidget.joint_type.fab.arc_interp)
        self.cmb_alignment_axis.setCurrentIndex(self.glWidget.joint_type.fab.alignment_axis)

        if self.glWidget.joint_type.fab.export_ext == "gcode":
            self.rdo_gcode.setChecked(True)
        elif self.glWidget.joint_type.fab.export_ext == "sbp":
            self.rdo_sbp.setChecked(True)
        elif self.glWidget.joint_type.fab.export_ext == "nc":
            self.rdo_nc.setChecked(True)

    def keyPressEvent(self, e):
        if e.key() == qtc.Qt.Key_Shift:
            self.glWidget.joint_type.mesh.select.shift = True
            self.glWidget.joint_type.mesh.select.refresh = True

    def keyReleaseEvent(self, e):
        if e.key() == qtc.Qt.Key_Shift:
            self.glWidget.joint_type.mesh.select.shift = False
            self.glWidget.joint_type.mesh.select.refresh = True
