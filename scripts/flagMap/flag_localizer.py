#!/usr/bin/env python

import rospy
from asl_turtlebot.msg import DetectedObject, DetectedObjectList


class FlagLocalizer():
    def __init__(self):

        rospy.init_node("flag_localizer", anonymous='true')

        self.detected_flags = {}
        self.oracle_flags = {}
        self.flag_moved_threshold = 0.01
        self.mapping_done = False

        rospy.Subscriber("detector/object", DetectedObjectList, object_detected_callback)
        rospy.Subscriber("oracle/flag_correction", DetectedObjectList, oracle_correction_callback)

    def object_detected_callback(self, msg):
        flag_list = msg.objects
        flag_state = msg.ob_msgs

        for flag in flag_list:
            # Check if oracle has placed the object somewhere (don't move it until game starts)
            if flag.id in self.oracle_flags.keys() and not self.mapping_done:
                continue
            elif flag.id in self.detected_flags.keys():
                # If new location is far from old location, move flag


                # If the new object location is close, take the average





