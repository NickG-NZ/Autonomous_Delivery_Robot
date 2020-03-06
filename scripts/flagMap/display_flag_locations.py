#!/usr/bin/env python
import rospy
from asl_turtlebot.msg import FlagMap
from visualization_msgs.msg import Marker


class FlagVisualizerNode(object):
    def __init__(self):
        rospy.init_node("flag_visualizer", anonymous=True)

        # Publishers
        self.marker_pub = rospy.Publisher("flag_visualization", Marker, queue_size=10)

        # Subscribers
        rospy.Subscriber("flag_map", FlagMap, self.publish_map_callback)
        rospy.loginfo("init done")

    def publish_map_callback(self, msg):
        for flag, idx in enumerate(msg.objects):
            pose = msg.coordinates[idx]  # pose2D
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time()
            marker.id = int(flag)
            # view-oriented text type
            marker.type = 9
            marker.pose.position.x = pose.x
            marker.pose.position.y = pose.y
            # scale.z sets the height of text
            marker.scale.z = 0.1
            # solid blue color
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            # set displayed text to object name
            marker.text = msg.objects[idx]
            self.marker_pub.publish(marker)

    def run(self):
        if not rospy.is_shutdown():
            rospy.spin()


if __name__ == "__main__":
    f = FlagVisualizerNode()
    try:
        f.run()
    except rospy.ROSInterruptException:
        pass
