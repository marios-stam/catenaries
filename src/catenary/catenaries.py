from sympy.geometry.point import Point2D
from math import log, degrees, e
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from geometry_msgs.msg import Point
import time
from scipy.spatial import distance

try:
    from .math_utils import Transformation, calculate2DAngleBetweenPoints, sqrt, sinh, cosh, tanhi, Line
    from .projection import get2DProjection, normalize
except Exception as exception:
    from math_utils import Transformation, calculate2DAngleBetweenPoints, sqrt, sinh, cosh, tanhi, Line
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
        # print("Converged in {} loops".format(counter))
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
    x1, x2, inverse, a, b, c = get_cat_problem_constants(P1, P2, L)

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

    if inverse:
        return xy[::-1, :]
    else:
        return xy


def findSlope(x1, x2, inverse, a, b, c, lowest_point):
    """
    Finds bounding lines of the catenary curve starting from x1 and x2
    Arguments:
        x1: x coordinate of the start point
        x2: x coordinate of the end point
        inverse: True if the catenary curve is inverted
        a: catenary parameter
        b: catenary parameter
        c: catenary parameter
        lowest_point: lowest point of the catenary curve in 2D
    """

    dx = 0.001
    # x = lowest_point[0]
    x = x1

    length = (x2-lowest_point[0])/dx
    print("length:", length)
    xy = np.zeros((int(length)*2, 2), dtype=np.float64)

    counter = 0
    while x < x2-dx:
        print("x:", x)
        y = a*cosh((x-b)/a)+c
        xy[counter] = [x, y]

        x += dx
        counter = counter+1

    if (xy[-1, 0] == 0 and xy[-1, 1] == 0):
        print("Opa")
        xy[-1] = [x2, a*cosh((x2-b)/a)+c]

    plt.figure()
    plt.plot(lowest_point[0], lowest_point[1], 'ro')
    plt.plot(xy[:, 0], xy[:, 1], 'b')
    plt.show()

    print("xy:")
    for p in xy:
        print(p)

    # get index of the lowest point
    min_z_index = np.argmin(xy, axis=0)[1]
    points_right_numbers = []
    for(i, point) in enumerate(xy[min_z_index+8:-1, :]):
        # line = Line(xy[i+1], point)
        line = Line(point, xy[min_z_index + i+1])
        print("============================================================")
        print("Checking line between {} and {}".format(point, xy[i+1]))

        # check if all points are left of the line
        points_left_counter = 0
        points_left = []
        points_right = []
        for point in xy:
            # print("Checking point:", point)
            if line.isPointLeft(point) == False:
                points_left_counter += 1
                points_left.append(point)
            else:
                points_right.append(point)

        points_right_numbers.append(len(points_right))

        points_left = np.array(points_left)
        points_right = np.array(points_right)

        # print("points_left:", points_left)
        # print("points_right:", points_right)

        # plt.figure()
        # xs = [0, 1]
        # ys = [line.evaluate(xx) for xx in xs]

        # plt.plot(xs, ys, 'b')
        # plt.plot(point[0], point[1], 'ro')
        # plt.plot(xy[i+1][0], xy[i+1][1], 'go')

        # if len(points_left) > 0:
        #     plt.plot(points_left[:, 0], points_left[:, 1], 'yo')

        # plt.plot(points_right[:, 0], points_right[:, 1], 'ko')
        # plt.show()

        # print("points_left:", points_left_counter)
        print("points right:", len(points_right))

    plt.figure()
    plt.plot(points_right_numbers, 'b')
    plt.show()

    print("Error: No line found")
    return None


def getVLowestPoints(P1, P2, L, safety_horiz_distance):
    x1, x2, inverse, a, b, c = get_cat_problem_constants(P1, P2, L)
    print("x1:{}, x2:{}, inverse:{}, a:{}, b:{}, c:{}".format(x1, x2, inverse, a, b, c))
    lowest = getCatenaryCurve2D_optimized_for_lowest_cat_point(P1, P2, L)[0]

    print("lowest:", lowest)

    # right_line = findSlope(x1, x2, inverse, a, b, c, lowest)
    right_line, left_line = findBoundingLines(x1, x2, inverse, a, b, c, lowest, safety_horiz_distance)
    t1 = time.time()

    return lowest, right_line, left_line


def findBoundingLines(x1, x2, inverse, a, b, c, lowest, safety_horiz_distance):
    """
    Finds bounding lines of the catenary curve starting from x1 and x2
    Arguments:
        x1: x coordinate of the start point
        x2: x coordinate of the end point
        inverse: True if the catenary curve is inverted
        a: catenary parameter
        b: catenary parameter
        c: catenary parameter
        lowest: lowest point of the catenary curve in 2D
    """
    dx = 0.08
    # x = lowest[0]
    x = x1

    length = (x2-x)/dx
    xy = np.zeros((int(length), 2), dtype=np.float64)

    counter = 0
    while x < x2-dx:
        y = a*cosh((x-b)/a)+c
        xy[counter] = [x, y]

        x += dx
        counter = counter+1

    xy[-1] = [x2, a*cosh((x2-b)/a)+c]

    # right line
    vert_point = [lowest[0], lowest[1]]
    while True:
        safety_end = [xy[-1][0]+safety_horiz_distance, xy[-1][1]]
        line_right = Line(vert_point, safety_end)

        allPointsLeft = True
        for p in xy[int(len(xy)/2):]:

            if not line_right.isPointLeft(p):
                allPointsLeft = False
                break
        if allPointsLeft:
            break

        vert_point[1] -= 0.2

        # plt.figure()
        # xs = [lowest[0], safety_end[0]]
        # ys = [line.evaluate(xx) for xx in xs]

        # plt.plot(xs, ys, 'r')
        # plt.plot(lowest[0], lowest[1], 'ro')
        # plt.plot(xy[:-1, 0], xy[:-1, 1], 'b')
        # plt.axis('equal')
        # plt.show()

    # left line
    vert_point = [lowest[0], lowest[1]]

    while True:
        safety_start = [xy[0][0]-safety_horiz_distance, xy[0][1]]
        line_left = Line(vert_point, safety_start)

        allPointsRight = True
        for p in xy[:int(len(xy)/2)]:

            if line_left.isPointLeft(p):
                allPointsRight = False
                break

        if allPointsRight:
            break

        vert_point[1] -= 0.2

        # plt.figure()
        # xs = [lowest[0], safety_end[0]]
        # ys = [line.evaluate(xx) for xx in xs]

        # plt.plot(xs, ys, 'r')
        # plt.plot(lowest[0], lowest[1], 'ro')
        # plt.plot(xy[:-1, 0], xy[:-1, 1], 'b')
        # plt.axis('equal')
        # plt.show()

    return line_right, line_left


def getCatenaryCurve2D_optimized_for_lowest_cat_point(P1, P2, L):
    """
    2D Catenary curve calculation optimized for the lowest point
    starting from the middle point and then going right or left to find the lowest point
    similar to binary search
    """
    x1, x2, inverse, a, b, c = get_cat_problem_constants(P1, P2, L)

    # print("x1:", x1)
    # print("x2:", x2)

    x = x1
    ddx = 0.01*1
    length = (x2-x1)/ddx+1
    xy = np.zeros((1, 2), dtype=np.float64)
    middle_point = (x2-x1)/2

    x_curr = middle_point
    y_curr = a*cosh((middle_point-b)/a)+c

    y_right = a*cosh((x_curr+ddx-b)/a)+c
    y_left = a*cosh((x_curr-ddx-b)/a)+c

    if y_right < y_left:
        initial_lowest_is_right = True
        x_curr = middle_point+ddx
    else:
        initial_lowest_is_right = False
        x_curr = middle_point-ddx

    lowest_is_right = initial_lowest_is_right
    counter = 0
    while initial_lowest_is_right == lowest_is_right:

        y_right = a*cosh((x_curr+ddx-b)/a)+c
        y_left = a*cosh((x_curr-ddx-b)/a)+c

        lowest_is_right = y_right < y_left
        if lowest_is_right:
            x_curr = x_curr+ddx
        else:
            x_curr = x_curr-ddx

        # print("counter:", counter)
        # print("x_curr:", x_curr)
        # print("y_left:", y_left)
        # print("y_right:", y_right)
        # print("lowest_is_right:", lowest_is_right)
        # print("============================================================")
        counter += 1
    xy[0] = [x_curr, y_curr]

    return xy


def get_cat_problem_constants(P1, P2, L):
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

    return x1, x2, inverse, a, b, c


def getCatenaryCurve3D(P1, P2, L, ax=None):
    angle = calculate2DAngleBetweenPoints(P1, P2)
    rotation = [0, 0, degrees(-angle)]

    trans = Transformation(rotation, translation=P1)

    t0 = time.time()
    # s, coords2D_x, coords2D_y = get2DProjection(list(P1), list(P2))
    # print("direct:", trans.transformPoint(P2))
    coords2D_x, _,  coords2D_y, _ = trans.transformPoint(P2)
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

        start3D_from_2DProjection = trans.inverseTransformPoint(
            [start2D[0], start2D[1], 0])
        print("end2D as input:", end2D)
        end3D_from_2DProjection = trans.inverseTransformPoint(
            [end2D[0],  0, end2D[1]])
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


def getCatenaryCurve3D_optimized_lowest_cat_point(P1, P2, L, ax=None):
    angle = calculate2DAngleBetweenPoints(P1, P2)
    rotation = [0, 0, degrees(-angle)]

    trans = Transformation(rotation, translation=P1)

    t0 = time.time()
    # s, coords2D_x, coords2D_y = get2DProjection(list(P1), list(P2))
    # print("direct:", trans.transformPoint(P2))
    coords2D_x, _,  coords2D_y, _ = trans.transformPoint(P2)
    dt = time.time()-t0

    start2D = [0, 0]
    end2D = [coords2D_x, coords2D_y]

    points2D = getCatenaryCurve2D_optimized_for_lowest_cat_point(
        start2D, end2D, L)

    Points3D = [trans.inverseTransformPoint([p[0], 0, p[1]]) for p in points2D]
    if PLOT or ax != None:
        Points3D = np.array(Points3D)
        ax.plot(Points3D[:, 0], Points3D[:, 1], Points3D[:, 2])

    return Points3D


def lowest_point(start, end, L):
    """
        Finds the lowest point of a catenary curve given its start, end and length L
    """
    points = getCatenaryCurve3D(start, end, L)

    # find the lowest point based on the z coordinate
    min_z_index = np.argmin(points, axis=0)[2]

    DEBUG = False
    if DEBUG:
        print("start:", start)
        print("end", end)
        print("L:", L)
        print(points[0])
        print(points[-1])

        print(points[min_z_index])
        print("============================================================")

    # return lowest point
    return points[min_z_index][:-1]


def lowest_point_optimized(start, end, L):
    """
        Finds the lowest point of a catenary curve given its start, end and length L
    """
    points = getCatenaryCurve3D_optimized_lowest_cat_point(start, end, L)

    DEBUG = False
    if DEBUG:
        print("start:", start)
        print("end", end)
        print("L:", L)

        print("Lowest:", points[-1])
        print("============================================================")

    # return lowest point
    return points[0][:-1]


def getForcesOnEnds2D(mass, x1, x2, L, n=2, forceOnP2=False):
    """
    Based on "Decentralized collaborative transport of fabrics using micro-UAVs"
    https: // www.researchgate.net/publication/335144536_Decentralized_collaborative_transport_of_fabrics_using_micro-UAVs
    """
    g = 9.8
    Tz = mass*g/n

    x0 = (x1+x2)/n
    """
    The equation L = a*sinh(x0/a) must be solved, in order to find a
    So approximateAnumerically(r) could be used
    with r = L/x0 and A = x0/a
    and a = x0/A
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
