#!/usr/bin/env python
# license removed for brevity

import math
import rospy
from rospy import topics
from catenaries.msg import drone_pose
from visualization_msgs.msg import Marker, MarkerArray, InteractiveMarkerFeedback
from scipy.spatial import distance
from tf import transformations
from scipy.spatial.transform import Rotation
import tf
from catenary import catenaries
from catenaries.srv import CatLowestPoint, CatLowestPointResponse

ROPES_NUMBER = 1


def add_two_ints_client():
    start = [0, 0, 0]
    end = [2, 0, 0]
    L = 3.5

    rospy.wait_for_service('catenary_lowest_point')
    try:
        catenary_lowest = rospy.ServiceProxy(
            'catenary_lowest_point', CatLowestPoint)

        return catenary_lowest(start, end, L)

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


if __name__ == "__main__":
    print(add_two_ints_client())
