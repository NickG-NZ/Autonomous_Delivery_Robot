#!/usr/bin/env python

import rospy
from asl_turtlebot.msg import DetectedObjectList, FoodMap
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool, String
import tf
import numpy as np


class Food_Localizer:
    def __init__(self):
        rospy.init_node('food_localizer', anonymous=True)
        # whether the map is frozen
        # updates to True once vending starts to freeze food map
        self.freeze_map = False
        # if distance from current average and new measurement is more than threshold, new measurement is discarded
        self.threshold = 0.2
        # coordinates of detected objects, key is object name, value is coordinate
        self.foodmap = {}
        self.count = {}
        self.trans_listener = tf.TransformListener()
        self.foodmap_pub = rospy.Publisher('/foodmap', FoodMap, queue_size=10)
        rospy.Subscriber('/request_vending_cmd', Bool, self.request_callback)
        rospy.Subscriber('/detector/objects', DetectedObjectList, self.detector_callback)

    def request_callback(self):
        self.freeze_map = True

    def detector_callback(self, msg):
        # if map is not frozen
        if not self.freeze_map:
            # get robot's current location and heading
            (translation, rotation) = self.trans_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            robot_x = translation[0]
            robot_y = translation[1]
            euler = tf.transformations.euler_from_quaternion(rotation)
            robot_theta = euler[2]

            objects = msg.objects
            data_list = msg.ob_msgs
            for ind in range(len(objects)):
                # get location of the object
                obj = objects[ind]
                data = data_list[ind]
                obj_theta = (data.thetaleft + data.thetaright) / 2.0
                obj_dist = data.distance
                obj_x = robot_x + obj_dist * np.cos(robot_theta + obj_theta)
                obj_y = robot_y + obj_dist * np.sin(robot_theta + obj_theta)
                obj_coord = np.array([obj_x, obj_y])
                # update foodmap
                if obj not in self.foodmap.keys():
                    self.foodmap[obj] = obj_coord
                    self.count[obj] = 0
                else:
                    if np.linalg.norm(self.foodmap[obj] - obj_coord) < self.threshold:
                        self.foodmap[obj] = (self.count[obj] * self.foodmap[obj] + obj_coord) \
                                            / float(self.count[obj] + 1)
                        self.count[obj] += 1
            # publish new food map
            map_msg = FoodMap()
            for obj in self.foodmap.keys():
                obj_name = String()
                obj_name.data = obj
                map_msg.objects.append(obj_name)
                coord = Pose2D()
                coord.x = self.foodmap[obj][0]
                coord.y = self.foodmap[obj][1]
                coord.theta = 0
                map_msg.coordinates.append(coord)
            self.foodmap_pub.publish(map_msg)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    fl = Food_Localizer()
    fl.run()
