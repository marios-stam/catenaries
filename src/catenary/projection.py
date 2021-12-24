import time
import numpy as np
from skspatial.objects import plane
from sympy import Plane
""" Based on :"Projecting 3D points to 2D plane"
    https://stackoverflow.com/questions/23472048/projecting-3d-points-to-2d-plane
"""


def perpendicular(a):
    b = np.empty_like(a)
    b[0] = -a[1]
    b[1] = a[0]
    return b


def normalize(a):
    a = np.array(a)
    return a/np.linalg.norm(a)


def get2DProjection(origin, target_point):
    t = time.time()
    print("origin:", origin)
    print("target_point:", target_point)

    points = [origin,
              target_point,
              [target_point[0], target_point[1], target_point[2] + 1]]  # add 1 in order not to be collinear

    plane = Plane(points[0], points[1], points[2])
    dt = time.time()-t
    print("             plane  calculation:", dt*1000, "ms")
    plane_normal = np.array(plane.normal_vector)

    print("plane_normal:", plane_normal)

    target_point, origin = np.array(target_point), np.array(origin)

    x_axis = normalize(np.array([target_point[0], target_point[1], 0]))
    y_axis = normalize(np.array([0, 0, 1]))

    s = np.dot(plane_normal, target_point-origin)
    x_coord = np.dot(x_axis, target_point-origin)
    y_coord = np.dot(y_axis, target_point-origin)

    return s, x_coord, y_coord


if __name__ == "__main__":
    # target_point = [2, 2, 1]
    # origin = [0, 0, 0]
    # s, t_1, t_2 = get2DProjection(target_point, origin)

    # print("s:", s)
    # print("t_1:", t_1)
    # print("t_2:", t_2)

    # inputs
    origin = [1.8222946216366225, 2.049465266111678, 1.5597655075075574]
    target_point = [3.763766512360378, 1.5694906459965623, 1.5419566181153574]

    # calculation
    t = time.time()

    points = [origin,
              target_point,
              [target_point[0], target_point[1], target_point[2] + 1]]  # add 1 in order not to be collinear

    plane = Plane(points[0], points[1], points[2])
    plane_normal = np.array(plane.normal_vector)
    print("plane_normal:", plane_normal)

    dt = time.time()-t
    print("plane  calculation:", dt*1000, "ms")
    # expected output
    plane_normal = [-0.47997462011512, -1.94147189072376, 0]
