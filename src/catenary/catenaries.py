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
    import test_catenaries as test

DEBUG = 0
PLOT = 0


def approximateAnumerically(r):
    """
    Solving equation r*A-sinh(A)=0 numerically
    """
    if 2*r/e < 1:
        if r < 3:
            Ai = sqrt(6*(r-1))
        else:
            Ai = log(2*r)+log(log(2*r))

        max_error = 0.001
        counter = 0
        while r-sinh(Ai)/Ai > max_error:
            num = sinh(Ai)-r*Ai
            denum = cosh(Ai)-r
            A_i = A_i-num/denum
            counter = counter + 1
        print("Converged in {} loops".format(counter))
        A = Ai
    else:
        A_approx = 0.25 * (1+3*log(2*r))+sqrt(2*log(2*r/e))
        A = A_approx

    return A


def getCatenaryCurve2D(P1, P2, L):
    """
        Based on https://math.stackexchange.com/questions/3557767/how-to-construct-a-catenary-of-a-specified-length-through-two-specified-points
        P1:point 1
        P2:point 2
        L:Length of rope
    """
    if DEBUG:
        print("Getting Catenary2D...")
        print("P1:", P1)
        print("P2:", P2)
        print("L :", L)

    x1, y1 = P1
    x2, y2 = P2

    inverse = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        inverse = True

    dx = x2-x1
    dy = y2-y1

    xb = (x1+x2)/2
    yb = (x1+x2)/2

    assert L**2 > dy**2+dx**2, "No solution,the points are too far apart"

    assert dx > 0, "The points are not in the correct order"

    r = sqrt(L**2-dy**2)/dx

    # rA=sinh(A)
    A = approximateAnumerically(r)
    if A == 0:
        print("Opa A=0")
    a = dx/(2*A)
    b = xb-a*tanhi(dy/L)
    c = y1-a*cosh((x1-b)/a)

    x = x1
    ddx = 0.01*1
    length = (x2-x1)/ddx+1
    xy = np.zeros((int(length), 2), dtype=np.float64)
    counter = 0
    while x < x2-ddx:
        y = a*cosh((x-b)/a)+c
        xy[counter] = [x, y]
        x = x+ddx
        counter = counter+1

    # manualyy enter last one
    x = x2
    y = a*cosh((x-b)/a)+c  # x2 is the last one
    xy[counter] = [x, y]

    if xy[-1][0] == 0 and xy[-1][1] == 0:
        print("length:", length)
        xy = xy[:-1, :]  # delete last row
    else:
        pass

    if inverse:
        return xy[::-1, :]
    else:
        return xy


def getCatenaryCurve3D(P1, P2, L, ax=None):
    angle = calculate2DAngleBetweenPoints(P1, P2)
    rotation = [0, 0, degrees(-angle)]

    trans = Transformation(rotation, translation=P1)

    t0 = time.time()
    s, coords2D_x, coords2D_y = get2DProjection(list(P1), list(P2))
    dt = time.time()-t0

    start2D = [0, 0]
    end2D = [coords2D_x, coords2D_y]

    points2D = getCatenaryCurve2D(start2D, end2D, L)

    if False:
        print("==================== Transformation Errors ====================")
        # start3D = trans.inverseTransformPoint([start2D[0], 0, start2D[1]])
        # end3D = trans.inverseTransformPoint([end2D[0], 0, end2D[1]])
        start3D = P1
        end3D = P2

        # start3D_from_points = trans.inverseTransformPoint([points2D[0][0], 0, points2D[0][1]])
        # end3D_from_points = trans.inverseTransformPoint([points2D[-1][0], 0, points2D[-1][1]])

        start3D_from_2DProjection = trans.inverseTransformPoint([start2D[0], start2D[1], 0])
        print("end2D as input:", end2D)
        end3D_from_2DProjection = trans.inverseTransformPoint([end2D[0], end2D[1], 0])
        print("end3D as output:", end3D)

        diff1 = start3D-start3D_from_2DProjection[:3]
        diff2 = end3D-end3D_from_2DProjection[:3]
        print("diff1:", diff1)
        print("diff2:", diff2)
        dist = distance.euclidean(end3D, end3D_from_2DProjection[:3])
        print("dist2:", dist)

    # Points3D = map(
    #     lambda point: trans.inverseTransformPoint([point[0], 0, point[1]]), points2D)
    # Points3D = np.array(list(Points3D))

    Points3D = [trans.inverseTransformPoint([p[0], 0, p[1]]) for p in points2D]
    if PLOT or ax != None:
        Points3D = np.array(Points3D)
        ax.plot(Points3D[:, 0], Points3D[:, 1], Points3D[:, 2])

    return Points3D


def getForcesOnEnds2D(mass, x1, x2, L, n=2, forceOnP2=False):
    """
    Based on "Decentralized collaborative transport of fabrics using micro-UAVs"
    https://www.researchgate.net/publication/335144536_Decentralized_collaborative_transport_of_fabrics_using_micro-UAVs
    """
    g = 9.8
    Tz = mass*g/n

    x0 = (x1+x2)/n
    """
    The equation L=a*sinh(x0/a) must be solved ,in order to find a
    So approximateAnumerically(r) could be used 
    with r=L/x0 and A=x0/a 
    and a=x0/A
    """
    r = L/x0
    A = approximateAnumerically(r)
    a = x0/A

    Tx = Tz * (a/L)

    if forceOnP2:
        return -Tx, -Tz
    else:
        return Tx, -Tz


def getForcesOnEnds3D(mass, p1, p2, L, n=2, forceOnP2=False):
    angle = calculate2DAngleBetweenPoints(P1, P2)
    rotation = [0, 0, degrees(-angle)]

    trans = Transformation(rotation, translation=P1)
    p2_1 = trans.transformPoint(P2)

    s, coords2D_x, coords2D_y = get2DProjection(list(P1), list(P2))
    print(coords2D_x, coords2D_y)

    Force2Dx, Force2Dz = getForcesOnEnds2D(
        mass, 0, coords2D_x, L, n=2, forceOnP2=forceOnP2)

    Force3D = trans.inverseTransformPoint([Force2Dx, 0, Force2Dz])
    if forceOnP2:
        Force3D[0] = -Force3D[0]
        Force3D[1] = -Force3D[1]

    if DEBUG:
        print("Force3D:", Force3D)

    return Force3D
