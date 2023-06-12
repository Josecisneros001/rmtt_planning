#!/usr/bin/env python3

import rospy
from tf import transformations
from apriltag_ros.msg import AprilTagDetectionArray
import tf2_ros
from geometry_msgs.msg import TransformStamped
import tf2_geometry_msgs
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped
from std_msgs.msg import Header
import numpy as np

def create_pose_stamped(x, y, z, r_x, r_y, r_z, w, frame_id):
  return PoseStamped(
    header=Header(
      seq=0,
      stamp=rospy.Time(0),
      frame_id=frame_id
    ),
    pose=Pose(
      position=Point(
        x=x,
        y=y,
        z=z,
      ),
      orientation=Quaternion(
        x=r_x,
        y=r_y,
        z=r_z,
        w=w
      )
    )
  )


class RobotPosePublisher:
    def __init__(self):
        rospy.init_node('robot_pose_publisher')
        
        self.map_origin = None
        self.detected_tag_id = None
        self.detected_tag_pose = None
        
        # Subscribe to AprilTag detections
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_detection_callback)
        
        # Create a publisher for robot pose
        self.pose_publisher = rospy.Publisher('robot_pose', PoseStamped, queue_size=10)
        
        # Create a transform broadcaster and listener
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.last_pose = create_pose_stamped(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 'map').pose

        self.apriltag_poses = [
          create_pose_stamped(0.0, 0.59, 1.32, 0.0, 0.0, 0.0, 1.0, 'map'),  # 0  
          create_pose_stamped(0.0, 1.1, 0.61, 0.0, 0.0, 0.0, 1.0, 'map'),  # 1
        ]

        rospy.loginfo('Robot pose publisher initialized')
        rospy.spin()
        
    def tag_detection_callback(self, data):
        if len(data.detections) > 0:
            detection = data.detections[0]  # Get the first detected tag
            
            self.detected_tag_id = detection.id[0]
            self.detected_tag_pose = detection.pose.pose.pose
            self.detected_tag_pose = create_pose_stamped(
              self.detected_tag_pose.position.x,
              self.detected_tag_pose.position.y,
              self.detected_tag_pose.position.z,
              self.detected_tag_pose.orientation.x,
              self.detected_tag_pose.orientation.y,
              self.detected_tag_pose.orientation.z,
              self.detected_tag_pose.orientation.w,
              'camera_link'
            )
        else:
            # If no tag is detected, remain in the same position
            self.detected_tag_id = None
        
        # Publish the robot pose
        robot_pose = PoseStamped()
        robot_pose.header.stamp = rospy.Time.now()
        robot_pose.header.frame_id = 'map'
        robot_pose.pose = self.get_robot_pose()
        self.pose_publisher.publish(robot_pose)
        self.publish_tf(robot_pose.pose)
    
    def get_robot_pose(self):
        if self.detected_tag_id is None:
            # No tag detected or map origin not set yet, return an identity pose
            return self.last_pose
        
        # Calculate the transform between the detected tag and the map origin
        return self.calculate_transform(self.detected_tag_pose, self.apriltag_poses[self.detected_tag_id])
        
    
    def calculate_transform(self, detected_pose, apriltag_pose):
        # Convert detected_pose frame from 'camera_link' to 'base_footprint'
        transform = self.tf_buffer.lookup_transform('base_footprint', 'camera_link', rospy.Time(0), rospy.Duration(1.0))
        detected_pose = tf2_geometry_msgs.do_transform_pose(detected_pose, transform)

        # Right now the detected pose is in the 'base_footprint' frame
        # We need to add the pose of the april tag to the origin of the map and i will get the distance between the robot and the origin
        resultant_pose = Pose()
        resultant_pose.position.x = (detected_pose.pose.position.x) + (apriltag_pose.pose.position.x)
        resultant_pose.position.y = (detected_pose.pose.position.y) * -1 + (apriltag_pose.pose.position.y)
        resultant_pose.position.z = (detected_pose.pose.position.z) + (apriltag_pose.pose.position.z) 
        # Add Euler Angles
        # quaternion1 = ( detected_pose.pose.orientation.x, detected_pose.pose.orientation.y, detected_pose.pose.orientation.z, detected_pose.pose.orientation.w)
        # quaternion2 = ( apriltag_pose.pose.orientation.x, apriltag_pose.pose.orientation.y, apriltag_pose.pose.orientation.z, apriltag_pose.pose.orientation.w)
        # euler1 = transformations.euler_from_quaternion(quaternion1)
        # euler2 = transformations.euler_from_quaternion(quaternion2)
        # euler = [sum(x) for x in zip(euler1, euler2)]
        # quaternion = transformations.quaternion_from_euler(euler[0], euler[1], euler[2])
        # resultant_pose.orientation.x = quaternion[0]
        # resultant_pose.orientation.y = quaternion[1]
        # resultant_pose.orientation.z = quaternion[2]
        # resultant_pose.orientation.w = quaternion[3]
        resultant_pose.orientation.x = 0.0
        resultant_pose.orientation.y = 0.0
        resultant_pose.orientation.z = 0.0
        resultant_pose.orientation.w = 1.0

        quat = transformations.quaternion_from_euler(np.deg2rad(180), 0, np.deg2rad(180))
        resultant_pose.orientation.x = quat[0]
        resultant_pose.orientation.y = quat[1]
        resultant_pose.orientation.z = quat[2]
        resultant_pose.orientation.w = quat[3]
        self.last_pose = resultant_pose

        rospy.loginfo('Robot pose: {}'.format(resultant_pose))

        return resultant_pose

    def publish_tf(self, robot_pose):
      try:
          # robot_pose to transform
          transform = TransformStamped()
          transform.header.stamp = rospy.Time.now()
          transform.header.frame_id = "odom"
          transform.child_frame_id = "base_footprint"
          transform.transform.translation.x = robot_pose.position.x
          transform.transform.translation.y = robot_pose.position.y
          transform.transform.translation.z = robot_pose.position.z
          transform.transform.rotation = robot_pose.orientation
          
          # Publish the transform
          self.tf_broadcaster.sendTransform(transform)

      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
          rospy.logwarn('Failed to publish TF')

if __name__ == '__main__':
    RobotPosePublisher()
