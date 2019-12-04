#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import OccupancyGrid, MapMetaData
from asl_turtlebot.msg import FoodMap
from std_msgs.msg import String, Bool
import numpy as np
from grids import StochOccupancyGrid2D
from planners import AStar
import itertools
import tf


class Vending_Planner:
    def __init__(self):
        rospy.init_node('vending_planner', anonymous=True)
        # waypoint queue, list of tuples, (order#, item#)
        self.queue = []
        # threshold on collision probability when finding waypoint corresponding to food
        self.occupancy_thres = 0.1
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
        self.started_vending = False
        self.planning = False
        self.current_waypoint = []
        self.trans_listener = tf.TransformListener()
        self.waypoint_pub = rospy.Publisher('/vending_cmd', Pose2D, queue_size=10)
        rospy.Subscriber('/map_metadata', MapMetaData, self.map_md_callback)
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/foodmap', FoodMap, self.foodmap_callback)
        rospy.Subscriber('/delivery_request', String, self.request_callback)
        rospy.Subscriber('/request_vending_cmd', Bool, self.cmd_callback)
        # rospy.Subscriber('/request_vending_replan', Bool, self.replan_callback)
        rospy.Subscriber('/resume_vending', Bool, self.resume_vending_callback)

    def resume_vending_callback(self):
        # freeze map and food location when in vending mode
        rospy.loginfo('started vending: freezing map and food location')
        self.started_vending = True
        # since last waypoint was not reached, place current waypoint back on queue
        # replan since we are probably at a new location now
        if self.current_waypoint:
            self.queue.insert(0, self.current_waypoint)
        rospy.loginfo('replanning')
        self.replan()

    def map_md_callback(self, msg):
        """
        receives maps meta data and stores it
        """
        if not self.started_vending:
            self.map_width = msg.width
            self.map_height = msg.height
            self.map_resolution = msg.resolution
            self.map_origin = (msg.origin.position.x, msg.origin.position.y)

    def map_callback(self, msg):
        """
        receives new map info and updates the map
        """
        if not self.started_vending:
            rospy.loginfo('vending: updating map')
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
                                                      self.map_probs, self.occupancy_thres)

    def foodmap_callback(self, msg):
        if not self.started_vending:
            # updates foodmap
            rospy.loginfo('vending: updating food location')
            self.foodmap = {}
            for ind in range(len(msg.objects)):
                obj = msg.objects[ind]
                coord = msg.coordinates[ind]
                self.foodmap[obj.data] = np.array([coord.x, coord.y])
            self.foodmap['home'] = self.home_coord

    def request_callback(self, msg):
        order = msg.data.split(',')
        # add order to list of food request if new order
        if order not in self.food_reqest:
            rospy.loginfo('vending: received new food request')
            self.food_reqest.append(order)
            order_num = len(self.food_reqest)
            # add all items to queue, the out of bound index represents home
            for i in range(len(order) + 1):
                self.queue.append([order_num, i])
        self.replan()

    def cmd_callback(self):
        """
        publish next waypoint
        :return:
        """
        if not self.queue:
            rospy.loginfo('vending: queue empty and there is no new request, sending home coordinate')
            coord = self.home_coord
        else:
            # remove first waypoint from queue and publish it
            obj = self.queue.pop(0)
            # record current waypoint label
            self.current_waypoint = obj
            order_num, item_num = obj
            # out of bound index represents home
            if item_num >= len(self.food_reqest[order_num]):
                coord = self.home_coord
            else:
                coord = self.food_waypoint[self.food_reqest[order_num][item_num]]
        waypoint = Pose2D()
        waypoint.x = coord[0]
        waypoint.y = coord[1]
        waypoint.theta = 0
        self.waypoint_pub.publish(waypoint)

    # def replan_callback(self):
    #     if not self.planning:
    #         self.replan()

    def find_waypoints(self):
        """
        find waypoint (points that are close to food and is collision free) corresponding to each food
        :return: None. Updates self.food_waypoint
        """
        # get all the grid points in the map
        x_coords = self.occupancy.resolution * np.array(range(self.occupancy.width)) \
                   + self.occupancy.origin_x * np.ones(self.occupancy.width)
        y_coords = self.occupancy.resolution * np.array(range(self.occupancy.height)) \
                   + self.occupancy.origin_y * np.ones(self.occupancy.height)
        x_coords, y_coords = np.meshgrid(x_coords, y_coords)
        x_coords_flat = np.ndarray.flatten(x_coords)
        y_coords_flat = np.ndarray.flatten(y_coords)
        # sort the grid points based on distances from food
        for obj, coord in self.foodmap.items():
            dist = np.sqrt((x_coords - coord[0] * np.ones(x_coords.shape))**2
                           + (y_coords - coord[1] * np.ones(y_coords.shape))**2)
            inds = np.argsort(np.ndarray.flatten(dist))
            waypoint = np.array([self.occupancy.origin_x, self.occupancy.origin_y])
            # find the closest point that is collision free
            for ind in inds:
                waypoint = np.array([x_coords_flat[ind], y_coords_flat[ind]])
                if self.occupancy.is_free(waypoint):
                    break
            self.food_waypoint[obj] = waypoint
        return

    def build_connected_graph(self):
        """
        calculate the distance between current location, each food object, and home
        :return: None. Update self.distance_mat, such that element i, j is the distance from food i to food j,
        where i and j are the index in self.food_list
        """
        # add home location to food_waypoints
        self.food_waypoint['home'] = self.home_coord
        # add current location to food_waypoints
        (translation, rotation) = self.trans_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        x = translation[0]
        y = translation[1]
        self.food_waypoint['self'] = [x, y]
        # create list of food so index of each objects is fixed
        self.food_list = self.food_waypoint.keys()
        nfood = len(self.food_list)
        # bounds for Astar search
        state_min = np.array([self.occupancy.origin_x, self.occupancy.origin_y])
        state_max = np.array([self.occupancy.origin_x + self.occupancy.resolution * self.occupancy.width,
                              self.occupancy.origin_y + self.occupancy.resolution * self.occupancy.height])
        # init distance_mat
        self.distance_mat = np.full([nfood, nfood], np.inf)
        for i in range(nfood):
            foodi = self.food_list[i]
            coordi = self.food_waypoint[foodi]
            # distance to self is zero
            self.distance_mat[i, i] = 0
            for j in range(i):
                foodj = self.food_list[j]
                coordj = self.food_waypoint[foodj]
                # solve Astar
                problem = AStar(state_min, state_max, coordi, coordj, self.occupancy, self.occupancy.resolution)
                success = problem.solve()
                if success:
                    # get total distance
                    cost = problem.est_cost_through[problem.path[-2]]
                    # assume symmetry, set both i to j and j to i to distance calculated
                    self.distance_mat[i, j] = cost
                    self.distance_mat[j, i] = cost
        return

    def replan(self):
        rospy.loginfo('vending: planning waypoints')
        self.planning = True
        self.find_waypoints()
        self.build_connected_graph()
        # generate all possible paths, and calculate distances
        paths = []
        distances = []
        order_numbers = len(self.food_reqest)
        total_item_numbers = [len(self.food_reqest[i]) for i in range(order_numbers)]
        waypoint_number = sum(total_item_numbers) + order_numbers
        temp = []
        # complete the earliest order first
        for order_iter in range(order_numbers - 1, -1, -1):
            item_numbers = len(self.food_reqest[order_iter])
            item_order = itertools.permutations(list(range(waypoint_number - sum(total_item_numbers[(order_iter + 1):]) - (order_numbers - order_iter))), item_numbers)
