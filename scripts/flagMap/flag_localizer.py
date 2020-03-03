#!/usr/bin/env python

import rospy
from asl_turtlebot.msg import DetectedObject, DetectedObjectList
from std_msgs.msg import Int64
from geometry_msgs.msg import Pose2D


class FlagLocalizer():
    def __init__(self):

        self.detected_flags = {}
        self.oracle_flags = {}
        self.flag_moved_threshold = 0.01
        self.mapping_done = False


    def object_detected(self, msg):
        flag_list = msg.objects
        flag_state = msg.ob_msgs

        for flag in flag_list:
            # Check if oracle has placed the object somewhere (don't move it until game starts)
            if flag.id in self.oracle_flags.keys() and not self.mapping_done:
                continue
            elif flag.id in self.detected_flags.keys():
                # If new location is far from old location, move flag


                # If the new object location is close, take the average


    def oracle_correction(self, msg):
        pass

    def flag_query(self, msg):
        pass

    def publish_map(self, msg):
        pass


class FlagLocalizerNode():
    def __init__(self):
        rospy.init_node("flag_localizer", anonymous='true')

        # Modularize functionality
        self.flag_localizer = FlagLocalizer()

        # Publishers
        rospy.Publisher("flagmap/response", Pose2D, queue_size=10)
        rospy.Publisher("")

        # Subscribers
        rospy.Subscriber("detector/object", DetectedObjectList, self.object_detected_callback)
        rospy.Subscriber("oracle/flag_correction", DetectedObjectList, self.oracle_correction_callback)
        rospy.Subscriber("flagmap/query", Int64, self.flag_query_callback)


    def object_detected_callback(self, msg):
        self.flag_localizer.object_detected(msg)

    def oracle_correction_callback(self, msg):
        self.flag_localizer.oracle_correction(msg)

    def flag_query_callback(self, msg):
        self.flag_localizer.flag_query(msg)

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

