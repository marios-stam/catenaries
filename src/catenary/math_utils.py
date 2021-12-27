from numpy.core.defchararray import translate
from geometry_msgs.msg import Vector3
from math import degrees, exp, log, sqrt, atan2
import math
import tf.transformations as transformations
import numpy as np
from scipy.spatial import distance

try:
    from .projection import get2DProjection, normalize
except Exception as exception:
    from projection import get2DProjection, normalize


def cosh(z):
    return((exp(z)+exp(-z))/2)


def sinh(z):
    return((exp(z)-exp(-z))/2)


def tanhi(z):
    return((1/2)*log((1+z)/(1-z)))


def getTransformationMatrix(rotation, translation):
    """
    Returns the transformation matrix frame specified by rotation and translation .

    Parameters
    ----------
    rotation : array,Vector3
        Rotation angles on x,y,z axes.
    translation : array,Vector3
        Translation of the new frame.
    Returns
    -------
    output : ndarray
        Transformation Matrix.
    """
    origin, xaxis, yaxis, zaxis = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)
    I = transformations.identity_matrix()
    Rx = transformations.rotation_matrix(math.radians(rotation[0]), xaxis)
    Ry = transformations.rotation_matrix(math.radians(rotation[1]), yaxis)
    Rz = transformations.rotation_matrix(math.radians(rotation[2]), zaxis)
    R = transformations.concatenate_matrices(Rx, Ry, Rz)

    T = transformations.translation_matrix(-np.array(translation))
    M = transformations.concatenate_matrices(R, T)

    return M


def trig(theta):
    return math.cos(theta), math.sin(theta), math.tan(theta)


def calculate2DAngleBetweenPoints(p1, p2):
    x1, y1, z1 = p1
    x2, y2, z2 = p2

    dx = x2-x1
    dy = y2-y1

    angle = math.atan2(dy, dx)
    return angle


class Transformation():
    def __init__(self, rotation, translation) -> None:
        self.matrix = getTransformationMatrix(rotation, translation)
        self.inv_matrix = np.linalg.inv(self.matrix)

    def transformPoint(self, p):
        if len(p) == 3:
            p = np.append(p, 1)

        p_m = np.dot(self.matrix, p)
        return p_m

    def inverseTransformPoint(self, p):
        if len(p) == 3:
            p = np.append(p, 1)

        p_m = np.dot(self.inv_matrix, p)
        return p_m


if __name__ == "__main__":
    P1 = [4.17654461, - 2.72807765, 1.97550017]
    P2 = [5.53085603, - 1.26064962, 2.08729769]

    angle = calculate2DAngleBetweenPoints(P1, P2)
    rotation = [0, 0, degrees(-angle)]
    trans = Transformation(rotation, translation=P1)

    s, coords2D_x, coords2D_y = get2DProjection(list(P1), list(P2))
    end2D = [coords2D_x, coords2D_y]
    print("end2D", end2D)
    # end3D_from_2DProjection = trans.inverseTransformPoint([end2D[0], 0, end2D[1]])

    dir_trans = trans.transformPoint(P2)[:3]
    print("dir_trans:", dir_trans)
    print("Direct rnasform:", dir_trans)
    end3D_from_DirectTransform = trans.inverseTransformPoint([end2D[0],  end2D[1], 0])

    print()
    diff2 = P2-end3D_from_DirectTransform[:3]
    print("diff2:", diff2)
    dist = distance.euclidean(P2, end3D_from_DirectTransform[:3])
    print("dist2:", dist)
