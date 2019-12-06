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
from collections import defaultdict


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
        self.mat_index = {}
        # coordinate of home, where food will be delivered to
        self.home_coord = [0.0, 0.0]
        # TODO: add dynamic parameter for home coordinate
        # food ordered, list of list of strings, each inner list contains one order
        self.food_request = []
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

    def resume_vending_callback(self, msg):
        # freeze map and food location when in vending mode
        rospy.loginfo('started vending: freezing map and food location')
        self.started_vending = True
        # since last waypoint was not reached, place current waypoint back on queue
        # also replan since we are probably at a new location now
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
        """
        updates food location
        """
        if not self.started_vending:
            # updates foodmap
            rospy.loginfo('vending: updating food location')
            self.foodmap = {}
            for ind in range(len(msg.objects)):
                obj = msg.objects[ind]
                coord = msg.coordinates[ind]
                self.foodmap[obj] = np.array([coord.x, coord.y])
            self.foodmap['home'] = self.home_coord

    def request_callback(self, msg):
        """
        add new order to queue and replan
        :param msg: new food request, detail see request_publisher.py
        :return: None, updates self.queue
        """
        order = msg.data.split(',')
        for i in range(len(order)):
            if order[i] not in self.foodmap.keys():
                rospy.loginfo('vending: requested food {0} does not exist'.format(order[i]))
                rospy.loginfo('vending: food available {0}'.format(self.food_list))
                del order[i]
        # add order to list of food request if new order
        if order:
            if order not in self.food_request:
                rospy.loginfo('vending: received new food request')
                self.food_request.append(order)
                order_num = len(self.food_request)
                # add all items to queue, the out of bound index represents home
                for i in range(len(order) + 1):
                    self.queue.append([order_num - 1, i])
        self.replan()

    def cmd_callback(self, msg):
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
            if item_num >= len(self.food_request[order_num]):
                coord = self.home_coord
            else:
                coord = self.food_waypoint[self.food_request[order_num][item_num]]
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

    def build_connected_graph(self, pickups_in_orders):
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
        food_calc = []
        for order in pickups_in_orders:
            for pickup in order:
                food_calc.append(self.food_request[pickup[0]][pickup[1]])
        food_calc.append('self')
        food_calc.append('home')
        nfood = len(food_calc)
        # index in distance_mat for each waypoint
        self.mat_index = {}
        for ind in range(len(food_calc)):
            self.mat_index[food_calc[ind]] = ind
        # bounds for Astar search
        state_min = np.array([self.occupancy.origin_x, self.occupancy.origin_y])
        state_max = np.array([self.occupancy.origin_x + self.occupancy.resolution * self.occupancy.width,
                              self.occupancy.origin_y + self.occupancy.resolution * self.occupancy.height])
        # init distance_mat
        self.distance_mat = np.full([nfood, nfood], np.inf)
        for i in range(nfood):
            foodi = food_calc[i]
            coordi = self.food_waypoint[foodi]
            # distance to self is zero
            self.distance_mat[i, i] = 0
            for j in range(i):
                foodj = food_calc[j]
                coordj = self.food_waypoint[foodj]
                # solve Astar
                problem = AStar(state_min, state_max, tuple(coordi), tuple(coordj), self.occupancy, self.occupancy.resolution)
                success = problem.solve()
                if success:
                    # get total distance
                    rospy.loginfo('vending: planned path length is {0}'.format(len(problem.path)))
                    cost = problem.est_cost_through[problem.path[-2]]
                    # assume symmetry, set both i to j and j to i to distance calculated
                    self.distance_mat[i, j] = cost
                    self.distance_mat[j, i] = cost
        return

    def replan(self):
        """
        replane vending path by calculating distance for every possible order of pick up and drop off
        :return: None, updates self.queue
        """
        rospy.loginfo('vending: planning waypoints')
        self.planning = True
        # items that still need to be picked up in each order
        pickups_in_orders = self.get_pickups()
        self.find_waypoints()
        self.build_connected_graph(pickups_in_orders)
        # generate all possible paths, and calculate distances
        paths = []
        distances = []
        order_numbers = len(pickups_in_orders)
        # number of pickups in each order
        num_pickups_order = [len(pickups_in_orders[i]) for i in range(order_numbers)]
        # waypoint required for each item pickup and each order drop off
        waypoint_number = sum(num_pickups_order) + order_numbers
        # generate paths through permutation
        temp = []
        # pickups_in_orders is sorted with earliest orders last, generate permutations starting with latest orders
        for order_iter in range(order_numbers):
            item_number = num_pickups_order[order_iter]
            # the open spots in waypoint sequences are those not taken up by previous order pick ups, their drop offs,
            # and drop off for current order
            open_spots = waypoint_number - sum(num_pickups_order[:order_iter]) - (order_iter + 1)
            # possible order of pick ups for this order
            item_order = list(itertools.permutations(list(range(open_spots)), item_number))
            temp.append(item_order)
        temp = list(itertools.product(*temp))
        for perm in temp:
            # combine the permutation for each orders to form the paths
            path = [[] for _ in range(waypoint_number)]
            for order_iter in range(order_numbers):
                # spots in path not taken up by waypoints
                open_spots = [i for i, x in enumerate(path) if not x]
                # the last open spot is for drop off
                path[open_spots[-1]] = [0, len(pickups_in_orders[0])]
                for item_iter in range(num_pickups_order[order_iter]):
                    # put the item label into corresponding spot in the path
                    # perm[order_iter][item_iter] indicate which of the open spots is selected for the item
                    path[open_spots[perm[order_iter][item_iter]]] = pickups_in_orders[order_iter][item_iter]
            paths.append(path)
            # calculate total distance for the path
            distance = 0
            # head of path segment
            head = self.mat_index['self']
            for waypoint in path:
                order_iter, item_iter = waypoint
                # tail of path segment
                if item_iter >= len(self.food_request[order_iter]):
                    tail = self.mat_index['home']
                else:
                    tail = self.mat_index[self.food_request[order_iter][item_iter]]
                distance += self.distance_mat[head, tail]
                head = tail
            distances.append(distance)
        # find min distance and corresponding path
        optim_ind = distances.index(min(distances))
        self.queue = paths[optim_ind]

    def get_pickups(self):
        """
        Find items that still needs to be picked up in each order
        :return: pickups_per_order: nested list, each inner list contains items to be picked up for that order,
        each item has the form [order#, item#], order and item numbers refer to self.food_request. earliest order last
        """
        pickups_per_order = defaultdict(list)
        for it in self.queue:
            pickups_per_order[it[0]].append(it)
        # sort based on order number, ascending order
        temp = sorted(pickups_per_order.items(), key=lambda s: s[0])
        # reverse, so the earlier order is last
        temp = temp[::-1]
        # get items in each order
        pickups_per_order = [x[1] for x in temp]
        # remove home from each order

        rospy.loginfo(self.food_request)
        rospy.loginfo(pickups_per_order)

        for order_iter in range(len(pickups_per_order)):
            pickups_per_order[order_iter] = [x for x in pickups_per_order[order_iter]
                                             if x[1] < len(self.food_request[x[0]])]
        return pickups_per_order

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    vp = Vending_Planner()
    vp.run()
