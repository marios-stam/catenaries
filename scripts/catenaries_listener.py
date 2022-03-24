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
import sys

catenary_mark_array_pub = rospy.Publisher(
    'catenaries_array',  MarkerArray, queue_size=10)

start_point = [0, 0, 0]
end_point = [1, 0, 0]
length = 1.5  # TODO: make this as ROS parameter

start_end_points_and_lenghts = [
    [start_point, end_point, length]
]

ROPES_NUMBER = 1


def get_leader_follower_topics():
    leader_topic = rospy.get_param("/cf_leader_name")

    follower_topic = rospy.get_param("/cf_follower_name")

    leader_topic = "/{}/{}".format(leader_topic, leader_topic)
    follower_topic = "/{}/{}".format(follower_topic, follower_topic)

    drone_topics = [leader_topic, follower_topic]
    print("leader_topic:", leader_topic)
    print("follower_topic:", follower_topic)

    return drone_topics


def get_points_planning(tf_listener, points):
    """
    This functions is used during path planning.
    """
    transform_template = "/drone"
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

    return points


def get_points_realtime(tf_listener, points):
    """
    This functions is used during experiments or replaying rosbags in order
    to visualise the shape of the rope and do visual checking before enetring the real one.
    """

    for i, topic in enumerate(leader_follower_topics):
        try:
            (trans, rot) = tf_listener.lookupTransform(
                '/world', topic, rospy.Time(0))
            point = trans

            points[i] = point
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

    print("points:", points)
    return points


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
    node_name = 'catenaries_listener'
    rospy.init_node(node_name, anonymous=False)

    tf_listener = tf.TransformListener()

    cat_handler = Catenaries.Catenaries_Handler(
        catenary_mark_array_pub, start_end_points_and_lenghts)

    points = [0, 0]

    try:
        leader_follower_topics = get_leader_follower_topics()
    except:
        print("No leader follower topics found")

    realtime = 0 if len(sys.argv) == 1 else sys.argv[1] == "1"
    print("realtime:", realtime)
    get_points = get_points_realtime if realtime else get_points_planning

    rate = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        points = get_points(tf_listener, points)
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
