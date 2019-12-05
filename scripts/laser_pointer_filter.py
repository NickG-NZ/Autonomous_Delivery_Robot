#!/usr/bin/env python

import numpy as np
import cv2
import rospy
import math

from sensor_msgs.msg import CompressedImage, Image, CameraInfo, LaserScan
from geometry_msgs.msg import Pose2D, PoseStamped
import tf
from scipy import ndimage

from cv_bridge import CvBridge, CvBridgeError


class LaserPointerFilter():
    def __init__(self):
        """ ROS Publishers and Subscribers """

        rospy.init_node('LaserPointerFilter')
        rospy.Subscriber('/camera_relay/image/compressed', CompressedImage, self.compressed_camera_callback,
                         queue_size=1, buff_size=2 ** 24)
        rospy.Subscriber('/raspicam_node/camera_info', CameraInfo, self.camera_info_callback)
        self.laser_pointer_cmd_publisher = rospy.Publisher('/laser_pointer_cmd', Pose2D, queue_size=1)

        self.bridge = CvBridge()
        self.trans_listener = tf.TransformListener()

        # camera and laser parameters that get updated
        self.cx = 198.4 #0.
        self.cy = 155.5 #0.
        self.fx = 329.2 # 0.00304 # 1.
        self.fy = 329.1 #0.00304 # 1

        # filter attributes
        self.image_cache_rgb = None
        self.image_cache_hsv = None
        self.image_cache_filtered = None
        self.applicable_horizon = [200, 300]
        self.hue_minimum_threshold = [128, 138]
        self.val_minimum_threshold = 200

        self.minimum_time_between_publish = 1
        self.laser_pointer_cmd_cache = Pose2D()
        self.last_laser_pointer_cmd_publish = rospy.Time.now()
        self.angle_multiplier = 1
        self.distance_forward = 0.2
        self.camera_matrix = None

    def compressed_camera_callback(self, msg):
        """ callback for camera images """

        try:
            self.applicable_horizon
        except:
            print("No self.applicable_horizon yet")
            return

        try:
            self.image_cache_rgb = self.bridge.compressed_imgmsg_to_cv2(msg, "passthrough")
        except CvBridgeError as e:
            print(e)

        self.filter_cache()
        self.publish_laser_pointer_cmd()

    def camera_info_callback(self, msg):
        """ extracts relevant camera intrinsic parameters from the camera_info message.
        cx, cy are the center of the image in pixel (the principal point), fx and fy are
        the focal lengths. Stores the result in the class itself as self.cx, self.cy,
        self.fx and self.fy """
        print("Here**************************************")

        self.cx = msg.P[2]
        self.cy = msg.P[6]
        self.fx = msg.P[0]
        self.fy = msg.P[5]

    def filter_cache(self):

        self.image_cache_hsv = np.zeros(self.image_cache_rgb.shape)
        self.image_cache_hsv[self.applicable_horizon[0]:self.applicable_horizon[1], :, :] = \
            cv2.cvtColor(self.image_cache_rgb[self.applicable_horizon[0]:self.applicable_horizon[1], :, :],
                         cv2.COLOR_RGB2HSV)

        hue_filter = np.logical_and( \
            self.image_cache_hsv[:, :, 0] > self.hue_minimum_threshold[0], \
            self.image_cache_hsv[:, :, 0] < self.hue_minimum_threshold[1])
        val_filter = self.image_cache_hsv[:, :, 2] > self.val_minimum_threshold

        self.image_cache_filtered = np.logical_and(hue_filter, val_filter)

        if np.sum(np.sum(self.image_cache_filtered)) < 5:
            self.laser_pointer_cmd_cache = None
            return

        # code to help tune filter #
        #import matplotlib.pyplot as plt
        #plt.imshow(self.image_cache_filtered)
        #plt.show()
        # fig, axs =plt.subplots(3)
        # axs[0].imshow(self.image_cache_rgb)
        # axs[1].imshow(self.image_cache_hsv)
        # axs[2].imshow(self.image_cache_filtered)
        # fig.show()
        # input("Press Enter to continue...")
        # fig.close()

        (v, u) = ndimage.measurements.center_of_mass(self.image_cache_filtered)

        x, y, theta = self.camera_pixel_to_world(u, v)

        self.laser_pointer_cmd_cache = Pose2D()
        self.laser_pointer_cmd_cache.x = x
        self.laser_pointer_cmd_cache.y = y
        self.laser_pointer_cmd_cache.theta = theta

    def project_pixel_to_ray(self, u, v):
        """ takes in a pixel coordinate (u,v) and returns a tuple (x,y,z)
        that is a unit vector in the direction of the pixel, in the camera frame.
        This function access self.fx, self.fy, self.cx and self.cy """

        x = (u - self.cx) / self.fx
        y = (v - self.cy) / self.fy
        norm = math.sqrt(x * x + y * y + 1)
        x /= norm
        y /= norm
        z = 1.0 / norm

        return x, y, z

    def camera_pixel_to_world(self, u, v):

        x, y, z = self.project_pixel_to_ray(u, v)

        theta_cam = math.atan2(-x, z)

        if theta_cam < 0:
            theta_cam += 2. * math.pi

        (translation, rotation) = self.trans_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        x_cam_map = translation[0]
        y_cam_map = translation[1]
        euler = tf.transformations.euler_from_quaternion(rotation)
        theta_cam_map = euler[2]

        t = self.angle_multiplier*(theta_cam + theta_cam_map)

        x_goal_map = self.distance_forward * np.cos(t) + x_cam_map
        y_goal_map = self.distance_forward * np.sin(t) + y_cam_map
        theta_goal_map = t

        # goal_cam = PoseStamped()

        # goal_cam.header.frame_id = "raspicam"

        # goal_cam.pose.position.x = y
        # goal_cam.pose.position.y = x
        # goal_cam.pose.position.z = z

        # euler_angle = math.atan2(-x, z) + math.pi/2  # might be backwards
        # goal_cam.pose.orientation.w = math.cos(euler_angle / 2)
        # goal_cam.pose.orientation.x = 0
        # goal_cam.pose.orientation.y = math.sin(euler_angle / 2)
        # goal_cam.pose.orientation.z = 0

        # origin_frame = "/map"
        # pose_origin = self.trans_listener.transformPose(origin_frame, goal_cam)
        # x_goal_map = pose_origin.pose.position.x
        # y_goal_map = pose_origin.pose.position.y
        # quaternion = (
        #     pose_origin.pose.orientation.x,
        #     pose_origin.pose.orientation.y,
        #     pose_origin.pose.orientation.z,
        #     pose_origin.pose.orientation.w)
        # euler = tf.transformations.euler_from_quaternion(quaternion)
        # theta_goal_map = euler[2]

        return x_goal_map, y_goal_map, theta_goal_map

    def publish_laser_pointer_cmd(self):
        if 1 or rospy.Time.now().secs - self.last_laser_pointer_cmd_publish.secs > \
                        (self.minimum_time_between_publish - 1):
            self.last_laser_pointer_cmd_publish = rospy.Time.now()
            if self.laser_pointer_cmd_cache is not None:
                #print(self.laser_pointer_cmd_cache)
                self.laser_pointer_cmd_publisher.publish(self.laser_pointer_cmd_cache)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    lpf = LaserPointerFilter()
    lpf.run()
