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

    plane_normal = get_plane_normal_custom(origin, target_point)
    # Calculate differences with the slow old one
    # plane_normal_lib = get_plane_normal_lib(origin, target_point)
    # diff = plane_normal_lib-plane_normal
    # print("differences:", diff)

    target_point, origin = np.array(target_point), np.array(origin)

    x_axis = normalize(np.array([target_point[0], target_point[1], 0]))
    y_axis = normalize(np.array([0, 0, 1]))

    s = np.dot(plane_normal, target_point-origin)
    x_coord = np.dot(x_axis, target_point-origin)
    y_coord = np.dot(y_axis, target_point-origin)

    return s, x_coord, y_coord


def get_plane_normal_lib(origin, target_point):
    points = [origin,
              target_point,
              [target_point[0], target_point[1], target_point[2] + 1]]  # add 1 in order not to be collinear

    plane = Plane(points[0], points[1], points[2])
    plane_normal = np.array(plane.normal_vector)
    return plane_normal


def get_plane_normal_custom(origin, target_point):
    points = [origin,
              target_point,
              [target_point[0], target_point[1], target_point[2] + 1]]  # add 1 in order not to be collinear

    p0, p1, p2 = points
    x0, y0, z0 = p0
    x1, y1, z1 = p1
    x2, y2, z2 = p2

    ux, uy, uz = u = [x1-x0, y1-y0, z1-z0]  # first vector
    vx, vy, vz = v = [x2-x0, y2-y0, z2-z0]  # sec vector

    u_cross_v = [uy*vz-uz*vy, uz*vx-ux*vz, ux*vy-uy*vx]  # cross product

    point = np.array(p1)
    normal = np.array(u_cross_v)

    d = -point.dot(normal)
    # print("d:", d)

    return normal


if __name__ == "__main__":
    # # inputs
    # origin = [1.8222946216366225, 2.049465266111678, 1.5597655075075574]
    # target_point = [3.763766512360378, 1.5694906459965623, 1.5419566181153574]

    # t = time.time()
    # # calculation

    # plane_normal = get_plane_normal_lib(origin, target_point)
    # dt = time.time()-t
    # print("Library plane  calculation:", dt*1000, "ms")

    # t = time.time()
    # plane_normal_custom = get_plane_normal_custom(origin, target_point)
    # dt = time.time()-t
    # print("Custom plane  calculation:", dt*1000, "ms")

    # # expected output
    # plane_normal = [-0.47997462011512, -1.94147189072376, 0]
    # # print("Expected plane normal:", plane_normal)

    # differences = plane_normal-plane_normal_custom
    # print("Differences:", differences)

    p1 = [1, 0, 0]
    p2 = [1, 1, 1]
    print(get2DProjection(p1, p2))
