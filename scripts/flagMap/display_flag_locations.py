#!/usr/bin/env python

from aslturtlebot.msg import FlagMap
from visualization_msgs.msg import Marker


class FlagVisualizer():
	def __init__(self):



class FlagVisualizerNode():
    def __init__(self):
        rospy.init_node("flag_visualizer", anonnymous=True)

        # Publishers
        self.marker_publisher = rospy.Publisher("flag_marker", Marker, queue_size=10)

        #Subscribers
        rospy.Subscriber("flag_visualization", FlagMap, self.publish_map_callback)


    def publish_map_callback():
    	

