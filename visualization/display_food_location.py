import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose2D
from asl_turtlebot.msg import FoodMap


class Food_Location_Publisher:
    def __init__(self):
        rospy.init_node('goal_marker', anonymous=True)
        self.marker_pub = rospy.Publisher('/food_marker', Marker, queue_size=10)
        rospy.Subscriber('/foodmap', FoodMap, self.map_callback)

    def map_callback(self, msg):
        for ind in range(len(msg.objects)):
            # get location of object
            coord = msg.coordinates[ind]
            x = coord.x
            y = coord.y
            # create marker
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time()
            marker.id = ind
            # view-oriented text type
            marker.type = 9
            marker.pose.position.x = x
            marker.pose.position.y = y
            # scale.z sets the height of text
            marker.scale.z = 0.1
            # solid blue color
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            # set displayed text to object name
            marker.text = msg.objects[ind]
            # publish marker
            self.marker_pub.publish(marker)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    flp = Food_Location_Publisher()
    flp.run()
