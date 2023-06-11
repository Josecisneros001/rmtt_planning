#!/usr/bin/env python3

import roslib
import sys
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import torch
import torchvision.transforms as transforms
import numpy as np
import cv2

import os
import glob
import torch
from imutils.video import VideoStream
import time

import sys
filepath = os.path.dirname(os.path.abspath(__file__))
sys.path.append(filepath + "/../MiDaS/")
import utils
from midas.model_loader import default_models, load_model

VERBOSE = True

class Midas:
    def __init__(self):
        rospy.init_node('midas')
        
        self.input_topic = rospy.get_param('~input_topic', '/front_cam/image_raw')
        self.output_topic = rospy.get_param('~output_topic', '/front_cam/image_depth')
        self.model_name = rospy.get_param('~model_name', 'dpt_swin2_large_384')        
        
        print("input_topic =", self.input_topic)
        print("output_topic =", self.output_topic)
        print("model_name =", self.model_name)

        self.model_type = self.model_name
        self.model_path = filepath+"/../MiDaS/" + default_models[self.model_type]
        self.optimize = False
        self.height = None
        self.square = False
        self.grayscale = True

        self.bridge = CvBridge()
        
        self.image_pub = rospy.Publisher(self.output_topic, Image, queue_size=1)
        self.image_sub = rospy.Subscriber(self.input_topic, Image, self.image_callback)
        
        self.first_execution = True

        self.model = None
        self.transform = None
        self.net_w = None
        self.net_h = None
        self.frame = None
        self.new_frame = False
        self.init()
        self.run()

    def init(self):
        model_type = self.model_type
        model_path = self.model_path
        optimize = self.optimize
        height = self.height
        square = self.square
        grayscale = self.grayscale

        print("Initialize")
        # select device
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        print("Device: %s" % self.device)

        self.model, self.transform, self.net_w, self.net_h = load_model(self.device, model_path, model_type, optimize, height, square)

    def run(self):
        model_type = self.model_name
        optimize = self.optimize
        grayscale = self.grayscale

        side = False
        model = self.model
        transform = self.transform
        net_w = self.net_w
        net_h = self.net_h
        device = self.device
        with torch.no_grad():
            fps = 1
            time_start = time.time()
            frame_index = 0
            while True and not rospy.is_shutdown():
                while self.new_frame is False and not rospy.is_shutdown():
                    time.sleep(0.001)
                self.new_frame = False
                frame = self.frame
                if frame is not None:
                    original_image_rgb = np.flip(frame, 2)  # in [0, 255] (flip required to get RGB)
                    image = transform({"image": original_image_rgb/255})["image"]

                    prediction = self.process(device, model, model_type, image, (net_w, net_h),
                                         original_image_rgb.shape[1::-1], optimize, True)

                    original_image_bgr = np.flip(original_image_rgb, 2) if side else None
                    content = self.create_side_by_side(original_image_bgr, prediction, grayscale)
                    
                    if VERBOSE:
                        cv2.imshow('MiDaS Depth Estimation - Press Escape to close window ', content/255)

                        if cv2.waitKey(1) == 27:  # Escape key
                            break
                    
                    alpha = 0.1
                    if time.time()-time_start > 0:
                        fps = (1 - alpha) * fps + alpha * 1 / (time.time()-time_start)  # exponential moving average
                        time_start = time.time()
                    print(f"\rFPS: {round(fps,2)}", end="")

                    try:
                        if grayscale:
                            # 32FC3 to  32FC1
                            content = cv2.cvtColor(content/255, cv2.COLOR_BGR2GRAY)
                            msg = self.bridge.cv2_to_imgmsg(content, encoding="passthrough")
                            msg.header.stamp = rospy.Time.now()
                            msg.header.frame_id = "camera_link"
                            self.image_pub.publish(msg)
                        else:
                            msg = self.bridge.cv2_to_imgmsg(content, encoding="rgb8")
                            msg.header.stamp = rospy.Time.now()
                            msg.header.frame_id = "camera_link"
                            self.image_pub.publish(msg)

                    except CvBridgeError as e:
                        print(e)

                    frame_index += 1
        

    def image_callback(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        self.new_frame = True

    def process(self, device, model, model_type, image, input_size, target_size, optimize, use_camera):
        """
        Run the inference and interpolate.

        Args:
            device (torch.device): the torch device used
            model: the model used for inference
            model_type: the type of the model
            image: the image fed into the neural network
            input_size: the size (width, height) of the neural network input (for OpenVINO)
            target_size: the size (width, height) the neural network output is interpolated to
            optimize: optimize the model to half-floats on CUDA?
            use_camera: is the camera used?

        Returns:
            the prediction
        """
        first_execution = self.first_execution

        if "openvino" in model_type:
            if first_execution or not use_camera:
                print(f"    Input resized to {input_size[0]}x{input_size[1]} before entering the encoder")
                first_execution = False

            sample = [np.reshape(image, (1, 3, *input_size))]
            prediction = model(sample)[model.output(0)][0]
            prediction = cv2.resize(prediction, dsize=target_size,
                                    interpolation=cv2.INTER_CUBIC)
        else:
            sample = torch.from_numpy(image).to(device).unsqueeze(0)

            if optimize and device == torch.device("cuda"):
                if first_execution:
                    print("  Optimization to half-floats activated. Use with caution, because models like Swin require\n"
                        "  float precision to work properly and may yield non-finite depth values to some extent for\n"
                        "  half-floats.")
                sample = sample.to(memory_format=torch.channels_last)
                sample = sample.half()

            if first_execution or not use_camera:
                height, width = sample.shape[2:]
                print(f"    Input resized to {width}x{height} before entering the encoder")
                first_execution = False

            prediction = model.forward(sample)
            prediction = (
                torch.nn.functional.interpolate(
                    prediction.unsqueeze(1),
                    size=target_size[::-1],
                    mode="bicubic",
                    align_corners=False,
                )
                .squeeze()
                .cpu()
                .numpy()
            )

        return prediction

    def create_side_by_side(self, image, depth, grayscale):
        """
        Take an RGB image and depth map and place them side by side. This includes a proper normalization of the depth map
        for better visibility.

        Args:
            image: the RGB image
            depth: the depth map
            grayscale: use a grayscale colormap?

        Returns:
            the image and depth map place side by side
        """
        depth_min = depth.min()
        depth_max = depth.max()
        normalized_depth = 255 * (depth - depth_min) / (depth_max - depth_min)
        normalized_depth *= 3

        right_side = np.repeat(np.expand_dims(normalized_depth, 2), 3, axis=2) / 3
        if not grayscale:
            right_side = cv2.applyColorMap(np.uint8(right_side), cv2.COLORMAP_INFERNO)

        if image is None:
            return right_side
        else:
            return np.concatenate((image, right_side), axis=1)



if __name__ == '__main__':
    midas = Midas()
    rospy.spin()
