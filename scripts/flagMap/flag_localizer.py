#!/usr/bin/env python

import rospy
import tf
import numpy as np
from asl_turtlebot.msg import DetectedObject, DetectedObjectList, FlagMap
from std_msgs.msg import Int64
from geometry_msgs.msg import Pose2D



class FlagLocalizer():
    def __init__(self):

        self.detected_flags = {}  # {(int)id: np.array([x, y])}
        self.flag_counts = {}  # {(int) id: int num_times_seen}
        self.oracle_flags = {}
        self.flag_move_threshold = 0.03
        self.mapping_done = False

    def object_detected(self, msg, robot_pose):
        map_changed = false
        flags_detected = msg.ob_msgs  # list of DetectedObject messages

        for flag in flags_detected:
            # Check if oracle has placed the object somewhere (don't move it until game starts)
            if flag.id in self.oracle_flags.keys() and not self.mapping_done:
                continue

            flag_pos = calc_flag_position(flag, robot_pose)
            if flag.id in self.detected_flags.keys():

                # If new location is far from old location, move flag
                if np.linalg.norm(flag_pos - self.detected_flags[flag.id]) > flag_move_threshold
                    self.detected_flags[flag.id] = flag_pos
                    self.flag_counts[flag.id] = 1

                # Else average the position with the old position
                else:
                    self.detected_flags[flag.id] = (self.detected_flags[flag.id] * \
                        self.flag_counts[flag.id] + flag_pos) / (self.flag_counts[flag.id] + 1)
                    self.flag_counts[flag.id] += 1

            # Else add flag to detected flags
            else:
                self.detected_flags


        if map_changed:
            return true
        return false


    def calc_flag_position(flag_object, robot_pose):
        # Angles from detector_mobile_net are in range (0, 2pi]
        theta_left = wrap2pi(flag_object.theta_left)
        theta_right = wrap2pi(fla_object.theta_right)

        if theta_left > 0.0 > theta_right:
            theta_avg = theta_left + theta_right + robot_pose[2]
        else:
            theta_avg = (theta_left + theta_right) / 2.0 + robot_pose[2]

        flag_position = np.zeros(2)
        flag_position[0] = robot_pose[0] + flag_object.distance*np.cos(theta_avg)
        flag_position[1] = robot_pose[1] + flag_object.distance*np.sin(theta_avg)
        return flag_position

    def wrap2pi(angle):
        # Wraps any angle to the range (-pi, pi]
        wrapped_angle = (angle + np.pi) % (2*np.pi) - np.pi
        return wrapped_angle

    def oracle_correction(self, msg):
        map_changed = false
        
        # Correction code here


        if map_changed:
            return true
        return false


    def flag_query(self, msg):
        pass



class FlagLocalizerNode():
    def __init__(self):
        rospy.init_node("flag_localizer", anonymous=True)

        # Modularize functionality
        self.flag_localizer = FlagLocalizer()
        self.tfListener = tf.TransformListener()

        # Publishers
        rospy.Publisher("flagmap/response", Pose2D, queue_size=10)
        rospy.Publisher("flag_visualization", FlagMap, queue_size=10)

        # Subscribers
        rospy.Subscriber("detector/objects", DetectedObjectList, self.object_detected_callback)
        rospy.Subscriber("oracle/flag_correction", DetectedObjectList, self.oracle_correction_callback)
        rospy.Subscriber("flagmap/query", Int64, self.flag_query_callback)
        rospy.Subscriber("state_correction", String, self.mapping_done_callback)


    def object_detected_callback(self, msg):
        # Get robot's pose
        robot_pose = np.zeros(3)
        try:
            (trans, rot) = self.tfListener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            robot_pose[0] = trans.x
            robot_pose[1] = trans.y
            robot_pose[2] = tf.transformations.euler_from_quaternion(rot)[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.loginfo("Couldn't place detected flag because robot pose is unknown")
            print e
            return

        # Place flag on map (or update it's location)
        map_changed = self.flag_localizer.object_detected(msg, robot_pose)
        if map_changed:
            self.publish_map()
            rospy.loginfo("Updated the flag map")


    def oracle_correction_callback(self, msg):
        map_changed = self.flag_localizer.oracle_correction(msg)
        if map_changed:
            self.publish_map()

    def flag_query_callback(self, msg):
        self.flag_localizer.flag_query(msg)

    def mapping_done_callback(self, msg):
        # If message is true, change the mapping_done_parameter
        if msg == "go_to_opponent":
            self.flag_localizer.mapping_done = True

    def publish_map(self):
        pass

    def run(self):
        if not rospy.is_shutdown():
            rospy.Spin()






if __name__ == '__main__':
    flg = FlagLocalizerNode()
    try:
        flg.run()
    except rospy.ROSInterruptException:
        pass

