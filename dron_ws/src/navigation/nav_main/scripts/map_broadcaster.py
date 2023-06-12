#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped, Pose


class TFBroadcaster:
    def __init__(self):
        rospy.init_node('tf_broadcaster')

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.initial_pose_subscriber = rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.initial_pose_callback)

        # Set the frame IDs for the transform
        self.parent_frame = 'map'
        self.child_frame = 'odom'

        self.initial_pose = Pose()
        self.initial_pose.position.x = 0.0
        self.initial_pose.position.y = 0.0
        self.initial_pose.position.z = 0.0
        self.initial_pose.orientation.x = 0.0
        self.initial_pose.orientation.y = 0.0
        self.initial_pose.orientation.z = 0.0
        self.initial_pose.orientation.w = 1.0

    def initial_pose_callback(self, msg):
        self.initial_pose = msg.pose.pose

    def update_tf(self):
        rate = rospy.Rate(30)  # 30 Hz

        while not rospy.is_shutdown():
            if self.initial_pose is not None:
                try:
                    # Create a transform message
                    transform = TransformStamped()
                    transform.header.stamp = rospy.Time.now()
                    transform.header.frame_id = self.parent_frame
                    transform.child_frame_id = self.child_frame
                    transform.transform.translation.x = self.initial_pose.position.x
                    transform.transform.translation.y = self.initial_pose.position.y
                    transform.transform.translation.z = self.initial_pose.position.z
                    transform.transform.rotation = self.initial_pose.orientation

                    # Broadcast the transform
                    self.tf_broadcaster.sendTransform(transform)

                except rospy.ROSException:
                    rospy.logwarn('Failed to update tf')

            rate.sleep()

if __name__ == '__main__':
    try:
        broadcaster = TFBroadcaster()
        broadcaster.update_tf()
    except rospy.ROSInterruptException:
        pass
