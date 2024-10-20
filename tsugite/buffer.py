import numpy as np
import OpenGL.GL as gl
from PIL import Image

from utils import *

class ElementProperties:
    def __init__(self, draw_type: DrawTypes, count, start_index, n: int) -> None:
        self.draw_type = draw_type
        self.count = count
        self.start_index = start_index
        self.n = n

class Buffer:
    def __init__(self, joint_type):
        self.joint_type = joint_type                            # parent is JointType
        self.VBO = gl.glGenBuffers(1)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.VBO)
        self.EBO = gl.glGenBuffers(1)
        gl.glBindBuffer(gl.GL_ELEMENT_ARRAY_BUFFER, self.EBO)
        self.vertex_no_info = 8
        image = Image.open("textures/end_grain.jpg")
        self.img_data = np.array(list(image.getdata()), np.uint8)
        image = Image.open("textures/friction_area.jpg")
        self.img_data_fric = np.array(list(image.getdata()), np.uint8)
        image = Image.open("textures/contact_area.jpg")
        self.img_data_cont = np.array(list(image.getdata()), np.uint8)

    # TODO: Buffer has a return?
    def buffer_vertices(self):
        # vertex attribute pointers
        gl.glVertexAttribPointer(0, 3, gl.GL_FLOAT, gl.GL_FALSE, 32, gl.ctypes.c_void_p(0)) #position
        gl.glEnableVertexAttribArray(0)
        gl.glVertexAttribPointer(1, 3, gl.GL_FLOAT, gl.GL_FALSE, 32, gl.ctypes.c_void_p(12)) #color
        gl.glEnableVertexAttribArray(1)
        gl.glVertexAttribPointer(2, 2, gl.GL_FLOAT, gl.GL_FALSE, 32, gl.ctypes.c_void_p(24)) #texture
        gl.glEnableVertexAttribArray(2)
        gl.glGenTextures(3)
        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_S, gl.GL_REPEAT)
        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_T, gl.GL_REPEAT)
        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MIN_FILTER, gl.GL_LINEAR)
        gl.glActiveTexture(gl.GL_TEXTURE0)
        gl.glBindTexture(gl.GL_TEXTURE_2D, 0)
        gl.glTexImage2D(gl.GL_TEXTURE_2D, 0, gl.GL_RGB, 400, 400, 0, gl.GL_RGB, gl.GL_UNSIGNED_BYTE, self.img_data)
        gl.glActiveTexture(gl.GL_TEXTURE1)
        gl.glBindTexture(gl.GL_TEXTURE_2D, 1)
        gl.glTexImage2D(gl.GL_TEXTURE_2D, 0, gl.GL_RGB, 400, 400, 0, gl.GL_RGB, gl.GL_UNSIGNED_BYTE, self.img_data_fric)
        gl.glActiveTexture(gl.GL_TEXTURE2)
        gl.glBindTexture(gl.GL_TEXTURE_2D, 2)
        gl.glTexImage2D(gl.GL_TEXTURE_2D, 0, gl.GL_RGB, 400, 400, 0, gl.GL_RGB, gl.GL_UNSIGNED_BYTE, self.img_data_cont)

        try:
            cnt = 6*len(self.joint_type.verts)
            return gl.glBufferData(gl.GL_ARRAY_BUFFER, cnt, self.joint_type.verts, gl.GL_DYNAMIC_DRAW)
        except:
            print("--------------------------ERROR IN ARRAY BUFFER WRAPPER -------------------------------------")

    def buffer_indices(self):
        cnt = 4*len(self.joint_type.indices)
        return gl.glBufferData(gl.GL_ELEMENT_ARRAY_BUFFER, cnt, self.joint_type.indices, gl.GL_DYNAMIC_DRAW)
