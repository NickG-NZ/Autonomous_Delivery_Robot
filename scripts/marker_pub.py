#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose2D, PoseStamped
X = 0
Y = 0
Z = 0

def callback(msg):
    global X, Y
    X = msg.x
    Y = msg.y
    print('goal update {0}, {1}'.format(X, Y))
    return

def publisher():
    vis_pub = rospy.Publisher('marker_topic', Marker, queue_size=10)
    rospy.init_node('marker_node', anonymous=True)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        marker = Marker()

        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time()

        rospy.Subscriber('/cmd_nav', Pose2D, callback)


        # IMPORTANT: If you're creating multiple markers, 
        #            each need to have a separate marker ID.
        marker.id = 0

        marker.type = 2 # sphere

        marker.pose.position.x = X
        marker.pose.position.y = Y
        marker.pose.position.z = 0

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        marker.color.a = 0.5 # Don't forget to set the alpha!
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        
        vis_pub.publish(marker)
        print('Published marker at {0},{1}!'.format(X, Y))
        
        rate.sleep()


if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
