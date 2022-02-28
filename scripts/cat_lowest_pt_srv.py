#!/usr/bin/env python
# license removed for brevity

import math
from resource import prlimit
import rospy
from rospy import topics
from catenaries.msg import drone_pose
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray, InteractiveMarkerFeedback
from scipy.spatial import distance
from tf import transformations
from scipy.spatial.transform import Rotation
import tf

from catenary import catenaries
from catenaries.srv import CatLowestPoint, CatLowestPointResponse

ROPES_NUMBER = 1


def handle_cat_lowest_point(req):
    start = req.start
    end = req.end
    L = req.L

    print("start:", start)
    print("end", end)
    print("L:", L)
    points = catenaries.getCatenaryCurve3D(start, end, L)

    # find the lowest point based on the z coordinate
    min_z_index = np.argmin(points, axis=0)[2]

    print(points[0])
    print(points[-1])

    print(points[min_z_index])
    print("============================================================")
    # return lowest point
    return CatLowestPointResponse(points[min_z_index][:-1])


def server():
    rospy.init_node('cat_lowest_pt_srv')
    s = rospy.Service('catenary_lowest_point',
                      CatLowestPoint, handle_cat_lowest_point)

    print("Ready to add two ints.")
    rospy.spin()


if __name__ == "__main__":
    server()
