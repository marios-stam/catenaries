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
from classes import Catenaries

catenary_mark_array_pub = rospy.Publisher(
    'catenaries_array',  MarkerArray, queue_size=10)

start_point = [0, 0, 0]
end_point = [2, 0, 0]
length = 3.5

start_end_points_and_lenghts = [
    [start_point, end_point, length]
]

ROPES_NUMBER = 1


def main():
    node_name = 'catenaries_listener'
    rospy.init_node(node_name, anonymous=False)

    tf_listener = tf.TransformListener()

    cat_handler = Catenaries.Catenaries_Handler(
        catenary_mark_array_pub, start_end_points_and_lenghts)

    transform_template = "/drone"
    points = [0, 0]

    rate = rospy.Rate(30.0)

    while not rospy.is_shutdown():
        for i in range(ROPES_NUMBER*2):
            transform_name = transform_template + \
                str(i+1)  # the transform name to look for
            try:
                (trans, rot) = tf_listener.lookupTransform(
                    '/world', transform_name, rospy.Time(0))
                point = trans

                points[i] = point
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        if points[0] == 0 and points[1] == 0:
            continue

        if type(points[0]) == int or type(points[1]) == int:
            continue

        if points[0][0] > points[0][1]:
            points = points[::-1]

        start_end_points_and_lenghts[0][0], start_end_points_and_lenghts[0][1] = points[0], points[1]

        # print("start_end_points_and_lenghts: ", start_end_points_and_lenghts)

        cat_handler.update(0, start_end_points_and_lenghts)
        cat_handler.visusalise()

        rate.sleep()


def listener():
    node_name = 'catenaries'
    rospy.init_node(node_name, anonymous=False)

    cat_handler = Catenaries.Catenaries_Handler(
        catenary_mark_array_pub, start_end_points_and_lenghts)

    topic_name = '/inter_marker/feedback'
    rospy.Subscriber(topic_name, InteractiveMarkerFeedback,
                     cat_handler.handleNewInterMarker)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    main()
