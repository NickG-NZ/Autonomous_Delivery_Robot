#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose2D
import tf


class Goal_Location_Publisher:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0
        rospy.init_node('goal_marker', anonymous=True)
        self.marker_pub = rospy.Publisher('/goal_marker', Marker, queue_size=10)
        rospy.sleep(1)
        rospy.Subscriber('/cmd_nav', Pose2D, self.goal_callback)

    def goal_callback(self, msg):
        # get location of object
        x = msg.x
        y = msg.y
        theta = msg.theta
        # create marker
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.id = 0
        # arrow
        marker.type = 0
        # set location to goal location
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        # direction to match command theta
        q = tf.transformations.quaternion_about_axis(theta, (0, 0, 1))
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]
        # size
        marker.scale.x = 0.5
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        # solid red color
        marker.color.a = 1.0  # Don't forget to set the alpha!
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        # publish marker
        self.marker_pub.publish(marker)
        print('published marker at {0}, {1}, {2}'.format(x, y, marker.pose.orientation.w))

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    flp = Goal_Location_Publisher()
    flp.run()
