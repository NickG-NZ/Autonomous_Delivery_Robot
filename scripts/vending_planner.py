#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import OccupancyGrid, MapMetaData
from asl_turtlebot.msg import FoodMap
from std_msgs.msg import String, Bool
import numpy as np
from grids import StochOccupancyGrid2D


class Vending_Planner:
    def __init__(self):
        rospy.init_node('vending_planner', anonymous=True)
        # waypoint queue, list of np arrays
        self.queue = []
        # threshold on collision probability when finding waypoint corresponding to food
        self.occupancy_thres = 0.05
        # map parameters
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0
        self.map_origin = [0, 0]
        self.map_probs = []
        # will be updatd to StochOccupancyGrid2D in map call back
        self.occupancy = None
        # location of food, dictionary with food name (string) as key and coordinates (np array) as values
        self.foodmap = {}
        # waypoints for picking up food, dictionary with food name (string) as key and coordinates (np array) as values
        self.food_waypoint = {}
        # distances between any two object, element ij is the distance between food i and food j
        self.distance_mat = np.array([])
        # list of food, to get index for distance_mat
        self.food_list = []
        # coordinate of home, where food will be delivered to
        self.home_coord = [0.0, 0.0]
        # TODO: add dynamic parameter for home coordinate
        # food ordered, list of list of strings, each inner list contains one order
        self.food_reqest = []
        self.waypoint_pub = rospy.Publisher('/vending_cmd', Pose2D, queue_size=10)
        rospy.Subscriber('/map_metadata', MapMetaData, self.map_md_callback)
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/foodmap', FoodMap, self.foodmap_callback)
        rospy.Subscriber('/delivery_request', String, self.request_callback)
        rospy.Subscriber('/request_vending_cmd', Bool, self.cmd_callback)
        rospy.Subscriber('/request_vending_replan', Bool, self.replan_callback)

    def map_md_callback(self, msg):
        """
        receives maps meta data and stores it
        """
        self.map_width = msg.width
        self.map_height = msg.height
        self.map_resolution = msg.resolution
        self.map_origin = (msg.origin.position.x, msg.origin.position.y)

    def map_callback(self, msg):
        """
        receives new map info and updates the map
        """
        # data is a list of int, occupancy probability of each grid, row major indexing, range (0, 100), unknown -1
        self.map_probs = msg.data
        # if we've received the map metadata and have a way to update it:
        if self.map_width > 0 and self.map_height > 0 and len(self.map_probs) > 0:
            # init probability based occupancy checker with window size 8
            self.occupancy = StochOccupancyGrid2D(self.map_resolution,
                                                  self.map_width,
                                                  self.map_height,
                                                  self.map_origin[0],
                                                  self.map_origin[1],
                                                  8,
                                                  self.map_probs)
            if self.foodmap:
                # find waypoints for each food
                self.find_waypoints()
                # compute distances between food
                self.build_connected_graph()

    def foodmap_callback(self, msg):
        # updates foodmap
        self.foodmap = {}
        for ind in range(len(msg.objects)):
            obj = msg.objects[ind]
            coord = msg.coordinates[ind]
            self.foodmap[obj.data] = np.array([coord.x, coord.y])
        if self.occupancy:
            # find waypoints for each food
            self.find_waypoints()
            # compute distances between food
            self.build_connected_graph()

    def find_waypoints(self):
        return

    def build_connected_graph(self):
        return

    def request_callback(self, msg):
        order = msg.data.split(',')
        # add order to list of food request if new order
        if order not in self.food_reqest:
            self.food_reqest.append(order)
        self.replan()

    def replan(self):
        return

    def cmd_callback(self):

