#!/usr/bin/env python

import rospy
import tf
import numpy as np
from asl_turtlebot.msg import DetectedObject, DetectedObjectList, FlagMap
from std_msgs.msg import Int64
from geometry_msgs.msg import Pose2D


class FlagLocalizer():
    """Parent class which FlagLocalizerNode inherits from"""
    def __init__(self):

        self.detected_flags = {}  # {(int)id: np.array([x, y])}
        self.flag_counts = {}  # {(int) id: int num_times_seen}
        self.oracle_flags = {}
        self.flag_move_threshold = 0.03
        self.mapping_done = False

    def object_detected(self, msg, robot_pose):
        map_changed = False
        flags_detected = msg.ob_msgs  # list of DetectedObject messages

        for flag in flags_detected:
            # Check if oracle has placed the object somewhere (don't move it until game starts)
            if flag.id in self.oracle_flags.keys() and not self.mapping_done:
                continue

            flag_pos = self.calc_flag_position(flag, robot_pose)
            if flag.id in self.detected_flags.keys():

                # If new location is far from old location, move flag
                if np.linalg.norm(flag_pos - self.detected_flags[flag.id]) > flag_move_threshold
                    self.detected_flags[flag.id] = flag_pos
                    self.flag_counts[flag.id] = 1

                # Else average the position with the old position
                else:
                    self.detected_flags[flag.id] = (self.detected_flags[flag.id] *
                        self.flag_counts[flag.id] + flag_pos) / (self.flag_counts[flag.id] + 1)
                    self.flag_counts[flag.id] += 1

            # Else add flag to detected flags
            else:
                self.detected_flags[flag.id] = flag_pos
                self.flag_counts[flag.id] = 1
            map_changed = True

        if map_changed:
            return True
        return False

    def calc_flag_position(flag_object, robot_pose):
        # Angles from detector_mobile_net are in range (0, 2pi]
        theta_left = self.wrap2pi(flag_object.theta_left)
        theta_right = self.wrap2pi(fla_object.theta_right)

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
        map_changed = False
        flag_id = int(msg.theta)  # used the theta slot of Pose2D for the flag id
        flag_pos = np.array([msg.x, msg.y])

        if flag_id in self.detected_flags.keys():
            map_changed = True
            
        self.detected_flags[flag_id] = flag_pos
        self.flag_counts[flag_id] = 1
        return map_changed


class FlagLocalizerNode(FlagLocalizer):
    def __init__(self):
        super(FlagLocalizerNode, self).__init__()
        rospy.init_node("flag_localizer", anonymous=True)

        # Modularize functionality
        self.tfListener = tf.TransformListener()

        # Publishers
        self.query_response_pub = rospy.Publisher("flagmap/response", Pose2D, queue_size=10)
        self.flag_map_pub = rospy.Publisher("flag_map", FlagMap, queue_size=10)

        # Subscribers
        rospy.Subscriber("detector/objects", DetectedObjectList, self.object_detected_callback)
        rospy.Subscriber("oracle/flag_correction", Pose2D, self.oracle_correction_callback)
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
        map_changed = self.object_detected(msg, robot_pose)
        if map_changed:
            self.publish_map()
            rospy.loginfo("Updated the flag map")

    def oracle_correction_callback(self, msg):
        map_changed = self.oracle_correction(msg)
        if map_changed:
            self.publish_map()
            rospy.loginfo("Oracle moved a flag")
        else:
            rospy.loginfo("Oracle placed a new flag on the map")

    def flag_query_callback(self, msg):
        flag_id = msg
        pose = Pose2D()
        pose.x = self.detected_flags[flag_id][0]
        pose.y = self.detected_flags[flag_id][1]
        pose.theta = float(flag_id)
        self.query_response_pub.publish(pose)
        rospy.loginfo("Responded to query about flag: %d", flag_id)
        
    def mapping_done_callback(self, msg):
        if msg == "go_to_opponent":
            self.mapping_done = True

    def publish_map(self):
        flag_map = FlagMap()
        for flag in self.detected_flags:
            pose = Pose2D()
            pose.x = self.detected_flags[flag][0]
            pose.y = self.detected_flags[flag][1]
            pose.theta = 0
            flag_map.objects.append(str(flag))
            flag_map.coordinates.append(pose)
        self.flag_map_pub.publish(flag_map)

    def run(self):
        if not rospy.is_shutdown():
            rospy.Spin()


if __name__ == '__main__':
    flg = FlagLocalizerNode()
    try:
        flg.run()
    except rospy.ROSInterruptException:
        pass

