import os
from typing import Any, Optional, Union

from numpy.typing import ArrayLike
from OpenGL.constant import Constant as glConstant

# aliases
# ArrayLike - can be a list, set, np.array
DegreeArray = ArrayLike        # for the return joint_type of arccos
DegreeFloat = float            # for the return joint_type of arccos
ZeroArray = ArrayLike
DotProduct = ArrayLike         # Returns the dot product of a and b. If both are scalars or both are 1-D arrays then
                               # a scalar is returned; otherwise an array is returned.

FilePath = str
FabricationExt = str          # either "gcode", "nc", "sbp"
DrawTypes = glConstant        # GL_QUADS, GL_LINES, GL_TRIANGLES, GL_LINE_STRIP
Direction = int               # 1 for positive, 0 for negative direction, 1 for bottom, 0 for top, in other cases it is -1 and 1

HoveringState = int           # -1: nothing, 0: hovering first, 1: hovering second, and so on. Application to Suggestions and Gallery
TimberState = int             # -1: nothing, 0: hovered, 1: adding, 2: pulling, 10: timber hovered, 12: timber pulled
NeighborType = int            # 0 = in region 1 = outside region, block 2 = outside region, free

FixSide = int                  # Implementation refers to 'Fixed Sides' as which of the six sides of the joint are
                              # connected to the main body of the timber. One fixed_sides side indicates that the joint is located at the
                              # end of a timber, which is the case for the timbers of an L-joint.
                              # Two fixed_sides sides means that the joint is located somewhere in
                              # the middle of the timber, as for the timbers of an X-joint.

def get_untitled_filename(name, ext, sep) -> FilePath:
    # list of all filenames with specified extension in the current directory
    ext_names = []
    for item in os.listdir():
        items = item.split(".")
        if len(items) > 1 and items[1] == ext:
            ext_names.append(items[0])

    # if the name already exists, append separator and number
    fname = name
    cnt = 1
    while fname in ext_names:
        fname = name + sep + str(cnt)
        cnt += 1

    # add path and extension, return
    fname = os.getcwd() + os.sep + fname + "." + ext
    return fname

def minimumSizeHint():
    return qtc.QSize(50, 50)

def sizeHint():
    # print("resize Hint!")
    return qtc.QSize(800, 800)