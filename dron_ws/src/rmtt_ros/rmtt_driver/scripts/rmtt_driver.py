#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import robomaster
from robomaster import robot
import rospy
import numpy as np
from std_msgs.msg import Int8, Float32, Empty, ColorRGBA
from geometry_msgs.msg import Vector3, Quaternion, Twist, Pose
from tf.broadcaster import TransformBroadcaster
from tf.transformations import quaternion_from_euler
import cv2
from sensor_msgs.msg import Range, Imu
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo


class RMTTDriver(object):
    IP_ADDRESS_STR = "192.168.10.2"
    ROBOT_ADDRESS_STR = "192.168.10.1"

    V_XY_MAX = 3.0
    V_Z_MAX = 3.0
    V_YAW_RATE_MAX = 3.0

    V_XY_SAFETY = 1.2
    V_Z_SAFETY = 1.8
    V_YAW_RATE_SAFETY = 1.5

    ACTIVE_FRONT_CAM = True
    FRONT_CAM_FREQ = 50.0

    def __init__(self):
        # Node Init
        rospy.init_node('rmtt_driver')
        
        # Cleanup when termniating the node
        rospy.on_shutdown(self.shutdown)
        
        # Load parameters
        RMTTDriver.IP_ADDRESS_STR = rospy.get_param("IP_ADDRESS_STR", RMTTDriver.IP_ADDRESS_STR)
        RMTTDriver.ROBOT_ADDRESS_STR = rospy.get_param("ROBOT_ADDRESS_STR", RMTTDriver.ROBOT_ADDRESS_STR)
        RMTTDriver.V_XY_MAX = rospy.get_param("V_XY_MAX", RMTTDriver.V_XY_MAX)
        RMTTDriver.V_Z_MAX = rospy.get_param("V_Z_MAX", RMTTDriver.V_Z_MAX)
        RMTTDriver.V_YAW_RATE_MAX = rospy.get_param("V_YAW_RATE_MAX", RMTTDriver.V_YAW_RATE_MAX)
        RMTTDriver.ACTIVE_FRONT_CAM = rospy.get_param("ACTIVE_FRONT_CAM", RMTTDriver.ACTIVE_FRONT_CAM)
        RMTTDriver.FRONT_CAM_FREQ = rospy.get_param("FRONT_CAM_FREQ", RMTTDriver.FRONT_CAM_FREQ)

        # Variables Init
        self.drone = robot.Drone()
        self.frequency = 100.0
        self.Ts = 1.0/self.frequency
        self.node_rate = rospy.Rate(self.frequency)
        self.drone_state = "LANDED"
        self.battery_state = "NA"
        self.bridge = CvBridge()
        self.shutdown_flag = False
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0

        # Publishers
        self.pubBtmRange = rospy.Publisher('btm_range', Float32, queue_size=10)
        self.pubFwdRange = rospy.Publisher('fwd_range', Float32, queue_size=10)
        self.pubImu = rospy.Publisher('imu', Imu, queue_size=5)
        self.pubImuAngle = rospy.Publisher('imu_angle', Float32, queue_size=5)
        self.pubBattery = rospy.Publisher('battery', Float32, queue_size=10)
        self.pubFrontCam = rospy.Publisher('front_cam/image_raw', Image, queue_size=10)
        self.pubFrontCamInfo = rospy.Publisher('front_cam/camera_info', CameraInfo, queue_size=10)



        # Subscribers
        rospy.Subscriber("takeoff", Empty, self.callBackTakeOff)
        rospy.Subscriber("land", Empty, self.callBackLand)
        rospy.Subscriber("shutdown", Empty, self.callBackShutdown)
        rospy.Subscriber("cmd_vel", Twist, self.callBackCmdVel)
        rospy.Subscriber("rgb_led", ColorRGBA, self.callBackRGBLed)

    def callBackShutdown(self, msg):
        self.shutdown()
        self.shutdown_flag = True

    def shutdown(self):
        # Stop the robot
        try:
            rospy.loginfo("Stopping the drone...")
            self.drone.unsub_tof()
            self.drone.flight.unsub_attitude()
            self.drone.flight.unsub_imu()
            self.drone.battery.unsub_battery_info()  

            if (RMTTDriver.ACTIVE_FRONT_CAM):
                self.drone.camera.stop_video_stream()
            
            self.drone.close()
            rospy.sleep(2)
        except:
            pass
        rospy.loginfo("Shutting down RMTTDriver Node...")


    # ROS Callbacks

    def callBackTakeOff(self, data):
        if (self.drone_state=="LANDED"):
            self.drone.led.set_led_breath(freq=2, r=255, g=0, b=0)    
            self.drone.flight.takeoff().wait_for_completed()
            self.drone_state="FLYING"
            self.drone.led.set_led(r=0, g=255, b=0)    

    def callBackLand(self, data):
        if (self.drone_state=="FLYING"):
            self.drone.led.set_led_breath(freq=2, r=255, g=0, b=0)    
            self.drone.flight.land().wait_for_completed()
            self.drone_state="LANDED"
            self.drone.led.set_led(r=0, g=0, b=0)    
        
    def callBackCmdVel(self, data):
        def map(x, in_min, in_max, out_min, out_max):
            return np.rint((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

        # roll, pitch, accelerate, yaw:  a,b,c,d [-100,100]
        vx = map(data.linear.x, -RMTTDriver.V_XY_MAX, RMTTDriver.V_XY_MAX, -100.0, 100.0)
        vy = map(data.linear.y, -RMTTDriver.V_XY_MAX, RMTTDriver.V_XY_MAX, -100.0, 100.0)
        vz = map(data.linear.z, -RMTTDriver.V_Z_MAX, RMTTDriver.V_Z_MAX, -100.0, 100.0)
        v_yaw_rate = map(data.angular.z, -RMTTDriver.V_YAW_RATE_MAX, RMTTDriver.V_YAW_RATE_MAX, -100.0, 100.0)
        
        # Saturate for safety.
        vx = np.clip(vx, map(-RMTTDriver.V_XY_SAFETY, -RMTTDriver.V_XY_MAX, RMTTDriver.V_XY_MAX, -100.0, 100.0), map(RMTTDriver.V_XY_SAFETY, -RMTTDriver.V_XY_MAX, RMTTDriver.V_XY_MAX, -100.0, 100.0))
        vy = np.clip(vy, map(-RMTTDriver.V_XY_SAFETY, -RMTTDriver.V_XY_MAX, RMTTDriver.V_XY_MAX, -100.0, 100.0), map(RMTTDriver.V_XY_SAFETY, -RMTTDriver.V_XY_MAX, RMTTDriver.V_XY_MAX, -100.0, 100.0))
        vz = np.clip(vz, map(-RMTTDriver.V_Z_SAFETY, -RMTTDriver.V_Z_MAX, RMTTDriver.V_Z_MAX, -100.0, 100.0), map(RMTTDriver.V_Z_SAFETY, -RMTTDriver.V_Z_MAX, RMTTDriver.V_Z_MAX, -100.0, 100.0)) 
        
        if (self.drone_state=="FLYING"):
            self.drone.flight.rc(a=-vy, b=vx, c=vz, d=-v_yaw_rate)

    def callBackRGBLed(self, data):
        self.drone.led.set_led(r=data.r, g=data.g, b=data.b)

    def readFrontCamera(self, timer):
        try:
            img = self.drone.camera.read_cv2_image()
            image_message = self.bridge.cv2_to_imgmsg(img, "bgr8")
            image_message.header.frame_id = "camera_link"
            image_message.header.stamp = rospy.Time.now()
            self.pubFrontCam.publish(image_message)

            # Publish camera info
            self.camera_info = CameraInfo()

            self.camera_info = CameraInfo()
            self.camera_info.header.frame_id = "camera_link"
            self.camera_info.header.stamp = rospy.Time.now()
            self.camera_info.height = 720
            self.camera_info.width = 960
            self.camera_info.distortion_model = "plumb_bob"
            self.camera_info.D = [-0.016272, 0.093492, 0.000093, 0.002999, 0.000000]
            self.camera_info.K = [929.562627, 0.000000, 487.474037, 0.000000, 928.604856, 363.165223, 0.000000, 0.000000, 1.000000]
            self.camera_info.R = [1.000000, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000, 0.000000, 1.000000]
            self.camera_info.P = [937.878723, 0.000000, 489.753885, 0.000000, 0.000000, 939.156738, 363.172139, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000]
            self.camera_info.binning_x = 0
            self.camera_info.binning_y = 0
            self.pubFrontCamInfo.publish(self.camera_info)
        except:
            pass

    def subTof(self, tof_cm):        
        tof_fwd_cm = self.drone.sensor.get_ext_tof()
        
        if (tof_cm>0):
            self.pubBtmRange.publish(Float32(tof_cm/100.0))
        else:
            self.pubBtmRange.publish(Float32(0.0))

        if (tof_fwd_cm==None):
            self.pubFwdRange.publish(Float32(np.nan))
        else:
            if (tof_fwd_cm>0):
                self.pubFwdRange.publish(Float32(tof_fwd_cm/100.0))
            else:
                self.pubFwdRange.publish(Float32(0.0))
    
    def subAttitude(self, attitude_angles):
        self.yaw, self.pitch, self.roll = attitude_angles

    
    def subImu(self, imu_info):
        vgx, vgy, vgz, agx, agy, agz = imu_info
        # agx = 0.01*agx
        # agy = 0.01*agy
        # agz = 0.01*agz
        imu_data = Imu()  
        imu_data.header.stamp = rospy.Time.now()
        imu_data.header.frame_id = "imu_link" 
        imu_data.orientation_covariance[0] = 1000000
        imu_data.orientation_covariance[1] = 0
        imu_data.orientation_covariance[2] = 0
        imu_data.orientation_covariance[3] = 0
        imu_data.orientation_covariance[4] = 1000000
        imu_data.orientation_covariance[5] = 0
        imu_data.orientation_covariance[6] = 0
        imu_data.orientation_covariance[7] = 0
        imu_data.orientation_covariance[8] = 0.000001

        newquat = quaternion_from_euler(self.roll, self.pitch, self.yaw)
        imu_data.orientation = Quaternion(newquat[0], newquat[1], newquat[2], newquat[3])
        imu_data.linear_acceleration_covariance[0] = -1
        imu_data.angular_velocity_covariance[0] = -1

        imu_data.linear_acceleration.x = agx
        imu_data.linear_acceleration.y = agy
        imu_data.linear_acceleration.z = agz

        imu_data.angular_velocity.x = vgx
        imu_data.angular_velocity.y = vgy
        imu_data.angular_velocity.z = vgz
        self.pubImu.publish(imu_data)
        self.pubImuAngle.publish(Float32(self.yaw))
        
    def subBatteryInfo(self, battery_info):
        battery_soc = battery_info
        self.pubBattery.publish(Float32(battery_soc))
        
        # warnings for different levels of battery state of charge
        if (self.battery_state=="NA"):  # Not Available
            if (battery_soc<30):
                self.battery_state = "MEDIUM"
                print("  battery: {0}".format(battery_soc))
        
        if (self.battery_state=="MEDIUM"):
            if (battery_soc<20):
                self.battery_state = "POOR"
                print("  [WARNING]  battery: {0}".format(battery_soc))
                
        if (self.battery_state=="POOR"):
            if (battery_soc<10):
                self.battery_state = "CRITICAL" 
                print("  [ALERT]  battery: {0}".format(battery_soc))

    def run(self):
        robomaster.config.LOCAL_IP_STR = RMTTDriver.IP_ADDRESS_STR
        robomaster.config.ROBOT_IP_STR = RMTTDriver.ROBOT_ADDRESS_STR
    
        print("\n**** RMTT ROS DRIVER ****")
        self.drone.initialize(conn_type="sta")
        print("  connexion to "+RMTTDriver.ROBOT_ADDRESS_STR+" ..... ok")
        drone_version = self.drone.get_sdk_version()
        print("  drone sdk version: {0}".format(drone_version))
        print("  Ready to fly!")
    
        self.drone.sub_tof(freq=10, callback=self.subTof)
        self.drone.flight.sub_attitude(10, self.subAttitude)
        self.drone.flight.sub_imu(10, self.subImu)
        self.drone.battery.sub_battery_info(freq=1, callback=self.subBatteryInfo)
        
    
        if (RMTTDriver.ACTIVE_FRONT_CAM):
            self.drone.camera.start_video_stream(display=False)
            self.drone.camera.set_fps("high")
            self.drone.camera.set_resolution("high")
            self.drone.camera.set_bitrate(6)  
            rospy.Timer(rospy.Duration(1.0 / RMTTDriver.FRONT_CAM_FREQ), self.readFrontCamera)
    
        rospy.spin()


if __name__ == '__main__':
    try:
        driver = RMTTDriver()
        driver.run()
    except rospy.ROSInterruptException:
        pass
