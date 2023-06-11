#!/usr/bin/env python3

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist

def publish_odometry():
    rospy.init_node('odometry_publisher', anonymous=True)
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rate = rospy.Rate(10)  # 10 Hz publishing rate

    while not rospy.is_shutdown():
        try:
            # Get the transform from map to base_footprint
            transform = tf_buffer.lookup_transform("map", "base_footprint", rospy.Time(0), rospy.Duration(1.0))

            # Create and populate the odometry message
            odom = Odometry()
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = "map"
            odom.child_frame_id = "base_footprint"

            # Set the position from the transform
            odom.pose.pose.position = transform.transform.translation

            # Set the orientation from the transform
            odom.pose.pose.orientation = transform.transform.rotation

            # Set the linear and angular velocities
            odom.twist.twist = Twist()

            # Publish the odometry message
            odom_pub.publish(odom)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Transform lookup failed")

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_odometry()
    except rospy.ROSInterruptException:
        pass