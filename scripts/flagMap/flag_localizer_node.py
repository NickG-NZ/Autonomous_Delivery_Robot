#!/usr/bin/env python

import rospy
import tf
import numpy as np
from asl_turtlebot.msg import DetectedObject, DetectedObjectList, FlagMap
from std_msgs.msg import Int64, String
from geometry_msgs.msg import Pose2D
from flag_localizer import FlagLocalizer


class FlagLocalizerNode(FlagLocalizer):
    def __init__(self):
        super(FlagLocalizerNode, self).__init__()

        rospy.init_node("flag_localizer", anonymous=True)
        self.tfListener = tf.TransformListener()

        # Publishers
        self.query_response_pub = rospy.Publisher("/flagmap/response", Pose2D, queue_size=10)
        self.flag_map_pub = rospy.Publisher("/flag_map", FlagMap, queue_size=10)
        self.opponent_flag_pub = rospy.Publisher("/opponent_flag", Int64, queue_size=10)

        # Subscribers
        rospy.Subscriber("/detector/objects", DetectedObjectList, self.object_detected_callback)
        rospy.Subscriber("/oracle/flag_correction", Pose2D, self.oracle_correction_callback)
        rospy.Subscriber("/flagmap/query", Int64, self.flag_query_callback)
        rospy.Subscriber("/state_correction", String, self.game_started_callback)
        rospy.Subscriber("/opponent_location", Pose2D, self.opponent_pose_callback)

    def object_detected_callback(self, msg):
        # Get robot's pose
        robot_pose = np.zeros(3)
        try:
            (trans, rot) = self.tfListener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            robot_pose[0] = trans.x
            robot_pose[1] = trans.y
            robot_pose[2] = tf.transformations.euler_from_quaternion(rot)[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.loginfo("Couldn't place detected flag(s) because robot pose is unknown")
            print e
            return

        # Place flags on map (or update their locations)
        map_changed, opponent_detected = self.object_detected(msg, robot_pose)

        if opponent_detected:
            self.opponent_flag_pub.publish(self.opponent_flag)
            rospy.loginfo("Opponent flag is: %d", self.opponent_flag)
        if map_changed:
            self.publish_map()
            rospy.loginfo("Updated the flag map")

    def oracle_correction_callback(self, msg):
        moved_flag = self.oracle_correction(msg)
        if moved_flag:
            rospy.loginfo("Oracle moved a flag")
        else:
            rospy.loginfo("Oracle placed a new flag on the map")
        self.publish_map()

    def flag_query_callback(self, msg):
        flag_id = msg
        pose = Pose2D()
        pose.x = self.detected_flags[flag_id][0]
        pose.y = self.detected_flags[flag_id][1]
        pose.theta = float(flag_id)
        self.query_response_pub.publish(pose)
        rospy.loginfo("Responded to query about flag: %d", flag_id)

    def game_started_callback(self, msg):
        if msg == "go_to_opponent":
            self.game_started = True
            self.save_flag_map()

    def opponent_pose_callback(self, msg):
        self.opponent_pose = np.array([msg.x, msg.y, msg.theta])

    def publish_map(self):
        flag_map = FlagMap()
        for flag in self.detected_flags:
            pose = Pose2D()
            pose.x = self.detected_flags[flag][0]
            pose.y = self.detected_flags[flag][1]
            pose.theta = 0
            flag_map.objects.append(str(flag).zfill(3))
            flag_map.coordinates.append(pose)
        self.flag_map_pub.publish(flag_map)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    flg = FlagLocalizerNode()
    try:
        flg.run()
    except rospy.ROSInterruptException:
        pass
