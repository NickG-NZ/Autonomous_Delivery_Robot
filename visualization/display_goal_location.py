#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose2D, PoseStamped
import tf

X = 0
Y = 0
Z = 0
THETA = 0

def callback(msg):
    global X, Y
    X = msg.x
    Y = msg.y
    THETA = msg.theta
    # print('goal update {0}, {1}, {2}'.format(X, Y, THETA))
    return

def publisher():
    vis_pub = rospy.Publisher('/goal_marker', Marker, queue_size=10)
    rospy.init_node('goal_marker', anonymous=True)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        marker = Marker()

        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time()

        rospy.Subscriber('/cmd_nav', Pose2D, callback)


        # IMPORTANT: If you're creating multiple markers, 
        #            each need to have a separate marker ID.
        marker.id = 0

        marker.type = 0 # arrow

        marker.pose.position.x = X
        marker.pose.position.y = Y
        marker.pose.position.z = 0

        q = tf.transformations.quaternion_about_axis(THETA, (0, 0, 1))
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]

        marker.scale.x = 0.2
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        marker.color.a = 1.0 # Don't forget to set the alpha!
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        
        vis_pub.publish(marker)
        # print('Published marker at {0},{1}!'.format(X, Y))
        
        rate.sleep()


if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
