from sympy.geometry.point import Point2D
from math import log, degrees, e
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from geometry_msgs.msg import Point
import time
from scipy.spatial import distance

try:
    from .math_utils import Transformation, calculate2DAngleBetweenPoints, sqrt, sinh, cosh, tanhi
    from .projection import get2DProjection, normalize
except Exception as exception:
    from math_utils import Transformation, calculate2DAngleBetweenPoints, sqrt, sinh, cosh, tanhi
    from projection import get2DProjection, normalize
    from catenaries import *


def main_3D():
    DEBUG = 0
    PLOT = 0
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('$X$', fontsize=20)
    ax.set_ylabel('$Y$', fontsize=20)
    ax.set_zlabel('$Z$', fontsize=20)

    p1 = np.array([4.1765446068919205, -2.728077654019848, 1.9755001677695203])
    p2 = np.array([5.530856032473361, -1.2606496227406319, 2.0872976938804])
    L = 3.5

    t = time.time()
    points = getCatenaryCurve3D(p1, p2, L)
    diff1 = p1-points[0][:3]
    diff2 = p2-points[-1][:3]
    dt = time.time()-t

    print("==========================================================")
    print("p1:", p1)
    print("p2:", p2)
    print()

    print("Calculation time:", dt*1000, "ms")
    print("Calculated p1:", points[0][:3])
    print("Calculated p2:", points[-1][:3])
    print("diff1:", diff1)
    print("diff2:", diff2)
    # distance between points
    dist = distance.euclidean(p2, points[-1][:3])
    print("dist2:", dist)


def main_2D():
    p1 = [0, 0]
    p2 = [0.9943381206665278, 0.1117975261108799]
    L = 3.5

    t = time.time()
    points = getCatenaryCurve2D(p1, p2, L)
    diff1 = p1-points[0][:3]
    diff2 = p2-points[-1][:3]
    dt = time.time()-t

    print("==========================================================")
    print("p1:", p1)
    print("p2:", p2)
    print()
    print("Calculation time:", dt*1000, "ms")
    print("diff1:", diff1)
    print("diff2:", diff2)
    # distance between points
    dist = distance.euclidean(p2, points[-1][:3])
    print("dist2:", dist)


def benchmark():
    P1 = np.array([1, 1, 0])
    P2 = np.array([2, 2, 0])
    L = 3

    dt_mean = 0
    times = 100
    for i in range(times):
        start = time.time()
        points = getCatenaryCurve3D(P1, P2, L, ax)
        dt = time.time()-start
        dt_mean += dt
    print("dt_mean:", dt_mean/times*1000, "msec")


def forces():
    DEBUG = 0
    PLOT = 0
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('$X$', fontsize=20)
    ax.set_ylabel('$Y$', fontsize=20)
    ax.set_zlabel('$Z$', fontsize=20)

    P1 = np.array([1, 1, 0])
    P2 = np.array([2, 2, 0])
    L = 3

    points = getCatenaryCurve3D(P1, P2, L, ax)

    print(type(points))
    print(points.shape)

    # Tz, Tx = getForcesOnEnds2D(1, P1[0], P2[0], L, n=2)
    # print("Tz:{}    Tx:{}".format(Tz, Tx))

    should_inv = 0
    Force3D = getForcesOnEnds3D(1, P1, P2, L, n=2, forceOnP2=should_inv)
    # Force3D = normalize(Force3D)
    p = P1 if not should_inv else P2
    ax.quiver(p[0], p[1], p[2], Force3D[0], Force3D[1],
              Force3D[2], length=0.1, color=(1, 0, 0, 1))

    plt.show()


def simple():
    DEBUG = 0
    PLOT = 1
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('$X$', fontsize=20)
    ax.set_ylabel('$Y$', fontsize=20)
    ax.set_zlabel('$Z$', fontsize=20)
    # set axes limits
    ax.set_zlim3d(-3, 3)

    P1 = np.array([1.2, 1., 0.])
    P2 = np.array([-1.2, 1., 0.])
    L = 3

    # P1 = np.array([1, 1, 0])
    # P2 = np.array([2, 2, 1])
    # L = 3

    points = getCatenaryCurve3D(P1, P2, L, ax)
    print("END3D:", points[-1])

    plt.show()


if __name__ == "__main__":
    # main_3D()
    # main_2D()
    simple()
