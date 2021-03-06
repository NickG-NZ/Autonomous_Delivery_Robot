#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData, Path
from geometry_msgs.msg import Twist, Pose2D, PoseStamped
from std_msgs.msg import String, Bool
import tf
import numpy as np
from numpy import linalg
from utils import wrapToPi
from planners import AStar, compute_smoothed_traj
from grids import StochOccupancyGrid2D
import scipy.interpolate
import matplotlib.pyplot as plt
from controllers import PoseController, TrajectoryTracker, HeadingController
from enum import Enum
import copy
import cPickle as pickle

from dynamic_reconfigure.server import Server
from asl_turtlebot.cfg import NavigatorConfig
from asl_turtlebot.msg import DetectedObject

# state machine modes, not all implemented
class Mode(Enum):
    IDLE = 0
    ALIGN = 1
    TRACK = 2
    PARK = 3
    STOPPING = 4
    STOPPED = 5
    CROSS = 6

class Navigator:
    """
    This node handles point to point turtlebot motion, avoiding obstacles.
    It is the sole node that should publish to cmd_vel
    """
    def __init__(self):
        rospy.init_node('turtlebot_navigator', anonymous=True)
        self.mode = Mode.IDLE

        # current state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # goal state
        self.x_g = None
        self.y_g = None
        self.theta_g = None
        self.goal_updated = False

        self.th_init = 0.0

        # map parameters
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0
        self.map_origin = [0, 0]
        self.occupancy = None
        self.prev_occupancy = None
        self.occupancy_updated = False
        self.collision_thresh = 0.3
        self.map_diff_thresh = 0.1

        # plan parameters
        self.plan_resolution =  0.1
        # state space bounds used in Astar, same unit as plan_resolution
        self.plan_horizon = 15

        # time when we started following the plan
        # current_plan_start time is updated to end of align / start of tracking
        self.current_plan_start_time = rospy.get_rostime()
        self.current_plan_duration = 0
        self.plan_start = [0., 0.]
        
        # Robot limits
        # self.v_max = 0.2    # maximum velocity
        # self.om_max = 0.4   # maximum angular velocity
        self.v_max = rospy.get_param('/navigator/v_max', 0.2)
        self.om_max = rospy.get_param('/navigator/om_max', 0.4)

        self.v_des = 0.12   # desired cruising velocity, used for path smoothing
        self.theta_start_thresh = 0.05   # threshold in theta to start moving forward when path-following
        self.start_pos_thresh = 0.2     # threshold to be far enough into the plan to recompute it

        # threshold at which navigator switches from trajectory to pose control
        self.near_thresh = 0.2
        # threshold at which navigator switches from park to idle
        self.at_thresh = 0.02
        self.at_thresh_theta = 0.05

        # Stop sign maneuver parameters
        self.stopped = False
        self.stop_maneuver_start_time = None
        self.time_till_stop = None
        self.stop_time_start = None  # Time when robot first stops
        self.stop_time = 2.  # How long to stop for
        self.stop_max_dist = 0.3  # Maximum distance from a stop sign to obey it
        self.crossing_time = 3.  # Time taken to cross an intersection (ignore stop signs)
        self.cross_start_time = None

        # trajectory smoothing
        self.spline_alpha = 0.15
        self.traj_dt = 0.1

        # trajectory tracking controller parameters
        self.kpx = 0.5
        self.kpy = 0.5
        self.kdx = 1.5
        self.kdy = 1.5

        # heading controller parameters
        self.kp_th = 2.

        self.traj_controller = TrajectoryTracker(self.kpx, self.kpy, self.kdx, self.kdy, self.v_max, self.om_max)
        # k1, k2, k3 initialized as zeros, later updated from dynamic parameters. line 120
        self.pose_controller = PoseController(0., 0., 0., self.v_max, self.om_max)
        self.heading_controller = HeadingController(self.kp_th, self.om_max)

        # path has field header and poses, which is a list of PoseStamped
        self.nav_planned_path_pub = rospy.Publisher('/planned_path', Path, queue_size=10)
        self.nav_smoothed_path_pub = rospy.Publisher('/cmd_smoothed_path', Path, queue_size=10)
        self.nav_smoothed_path_rej_pub = rospy.Publisher('/cmd_smoothed_path_rejected', Path, queue_size=10)
        self.nav_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.goal_reached_pub = rospy.Publisher('/reached_goal', Bool, queue_size=10)

        self.trans_listener = tf.TransformListener()
        self.cfg_srv = Server(NavigatorConfig, self.dyn_cfg_callback)

        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/map_metadata', MapMetaData, self.map_md_callback)
        rospy.Subscriber('/cmd_nav', Pose2D, self.cmd_nav_callback)
        rospy.Subscriber("/state_correction", String, self.game_started_callback)

        print "finished init"

    def game_started_callback(self, msg):
        if msg == "go_to_opponent":
            self.save_map()
            rospy.loginfo("Saved Occupancy Grid")

    def save_map(self):
        # Called when the game starts (after mapping has finished)
        try:
            os.makedirs('OccupancyGrids')
        except OSError:
            pass
        filename = 'OccupancyGrids/saved_ocupancy_grid.p'
        with open(filename, 'wb') as fp:
            pickle.dump(self.detected_flags, fp, protocol=pickle.HIGHEST_PROTOCOL)

    def load_map(self):
        # Not automatically called, can be used if needed
        filename = 'OccupancyGrids/saved_ocupancy_grid.p'
        with open(filename, 'rb') as fp:
            self.occupancy = pickle.load(fp)

    def reached_goal(self):
        """
        Called when robot reaches goal.
        Forgets goal, and publishes to /reached_goal
        """
        msg = Bool()
        msg.data = True
        self.goal_reached_pub.publish(msg)
        # Forget current goal
        self.x_g = None
        self.y_g = None
        self.theta_g = None
        self.switch_mode(Mode.IDLE)

    def dyn_cfg_callback(self, config, level):
        rospy.loginfo("Reconfigure Request: k1:{k1}, k2:{k2}, k3:{k3}".format(**config))
        self.pose_controller.k1 = config["k1"]
        self.pose_controller.k2 = config["k2"]
        self.pose_controller.k3 = config["k3"]
        return config

    def cmd_nav_callback(self, data):
        """
        loads in goal if different from current goal, and replans
        """
        if data.x != self.x_g or data.y != self.y_g or data.theta != self.theta_g:
            self.goal_updated = True
            self.x_g = data.x
            self.y_g = data.y
            self.theta_g = data.theta
            self.replan()
            self.goal_updated = False

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
        msg type: OccupancyGrid
        """
        # data is a list of int, occupancy probability of each grid, row major indexing, range (0, 100), unknown -1
        map_probs = msg.data

        # if we've received the map metadata and have a way to update it:
        if self.map_width>0 and self.map_height>0 and len(map_probs)>0:
            # init probability based occupancy checker with window size 8
            if self.occupancy not None:
                self.prev_occupancy = copy.deepcopy(self.occupancy)
                self.occupancy = StochOccupancyGrid2D(self.map_resolution,
                                                      self.map_width,
                                                      self.map_height,
                                                      self.map_origin[0],
                                                      self.map_origin[1],
                                                      8,
                                                      map_probs,
                                                      self.collision_thresh)

                if self.x_g is not None and self.map_difference_check():
                    # if we have a goal to plan to and map changed significantly, re-plan
                    rospy.loginfo("REPLANNING BECAUSE OF NEW MAP")
                    self.occupancy_updated = True
                    self.replan()  # new map, need to re-plan
            self.occupancy_updated = False

    def map_difference_check(self):
        """
        Checks similarity between current and new map.
        If difference greater than threshold, replans.
        """
        def snap_to_grid_resolution(x, resolution):
            return int(round(x / resolution))
        
        d_range = 1.5
        range_new = int(round(d_range/self.map_resolution/2.0))
        range_old = int(round(d_range/self.prev_occupancy.resolution/2.0))

        x_old = snap_to_grid_resolution(self.x - self.prev_occupancy.origin_x, self.prev_occupancy.resolution)
        y_old = snap_to_grid_resolution(self.y - self.prev_occupancy.origin_y, self.prev_occupancy.resolution)
        x_new = snap_to_grid_resolution(self.x - self.map_origin[0], self.map_resolution)
        y_new = snap_to_grid_resolution(self.y - self.map_origin[1], self.map_resolution)

        old_grid = np.array([self.prev_occupancy.probs]).reshape(self.prev_occupancy.width, self.prev_occupancy.height)
        new_grid = np.array([self.occupancy.probs]).reshape(self.occupancy.width, self.occupancy.height)

        window_old = [max(0, x_old-range_old),
                      min(int(self.prev_occupancy.width), x_old+range_old),
                      max(0, y_old-range_old),
                      min(int(self.prev_occupancy.height), y_old+range_old)]

        window_new = [max(0, x_new-range_new),
                      min(int(self.map_width), x_new+range_new),
                      max(0, y_new-range_new),
                      min(int(self.map_height), y_new+range_new)]

        sub_grid_old = old_grid[window_old[0]:window_old[1], window_old[2]:window_old[3]]
        sub_grid_new = new_grid[window_new[0]:window_new[1], window_new[2]:window_new[3]]

        diff = np.linalg.norm(sub_grid_new - sub_grid_old)/sub_grid_old.shape[0]

        if diff > self.map_diff_thresh:
            return True
        rospy.loginfo("MAP DIFFERENCE IS INSIGNIFICANT")
        return False

    def shutdown_callback(self):
        """
        publishes zero velocities upon rospy shutdown
        """
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.nav_vel_pub.publish(cmd_vel)

    def near_goal(self):
        """
        returns whether the robot is close enough in position to the goal to
        start using the pose controller
        """
        return linalg.norm(np.array([self.x-self.x_g, self.y-self.y_g])) < self.near_thresh

    def at_goal(self):
        """
        returns whether the robot has reached the goal position with enough
        accuracy to return to idle state
        """
        return (linalg.norm(np.array([self.x-self.x_g, self.y-self.y_g])) < self.at_thresh and abs(wrapToPi(self.theta - self.theta_g)) < self.at_thresh_theta)

    def aligned(self):
        """
        returns whether robot is aligned with starting direction of path
        (enough to switch to tracking controller)
        """
        return (abs(wrapToPi(self.theta - self.th_init)) < self.theta_start_thresh)

    def close_to_plan_start(self):
        return (abs(self.x - self.plan_start[0]) < self.start_pos_thresh and abs(self.y - self.plan_start[1]) < self.start_pos_thresh)

    def snap_to_grid(self, x):
        return (self.plan_resolution*round(x[0]/self.plan_resolution), self.plan_resolution*round(x[1]/self.plan_resolution))

    def switch_mode(self, new_mode):
        rospy.loginfo("Switching from %s -> %s", self.mode, new_mode)
        self.mode = new_mode

    def publish_planned_path(self, path, publisher):
        # publish planned plan for visualization
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        for state in path:
            pose_st = PoseStamped()
            pose_st.pose.position.x = state[0]
            pose_st.pose.position.y = state[1]
            pose_st.pose.orientation.w = 1
            pose_st.header.frame_id = 'map'
            path_msg.poses.append(pose_st)
        publisher.publish(path_msg)

    def publish_smoothed_path(self, traj, publisher):
        # publish planned plan for visualization
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        for i in range(traj.shape[0]):
            pose_st = PoseStamped()
            pose_st.pose.position.x = traj[i, 0]
            pose_st.pose.position.y = traj[i, 1]
            pose_st.pose.orientation.w = 1
            pose_st.header.frame_id = 'map'
            path_msg.poses.append(pose_st)
        publisher.publish(path_msg)

    def publish_control(self):
        """
        Runs appropriate controller depending on the mode. Assumes all controllers
        are all properly set up / with the correct goals loaded
        """
        t = self.get_current_plan_time()

        if self.mode == Mode.PARK:
            V, om = self.pose_controller.compute_control(self.x, self.y, self.theta, t)
        elif self.mode == Mode.TRACK:
            V, om = self.traj_controller.compute_control(self.x, self.y, self.theta, t)
        elif self.mode == Mode.ALIGN:
            V, om = self.heading_controller.compute_control(self.x, self.y, self.theta, t)
        elif self.mode == Mode.STOPPING or self.mode == Mode.CROSS:
            V, om = self.traj_controller.compute_control(self.x, self.y, self.theta, t)
        else:
            V = 0.
            om = 0.

        cmd_vel = Twist()
        cmd_vel.linear.x = V
        cmd_vel.angular.z = om
        self.nav_vel_pub.publish(cmd_vel)

    def get_current_plan_time(self):
        t = (rospy.get_rostime()-self.current_plan_start_time).to_sec()
        return max(0.0, t)  # clip negative time to 0

    def replan(self):
        """
        runs planner based on current pose
        if plan long enough to track:
            smooths resulting traj
            if new plan takes more time than current one and goal not changed:
                reject new plan
            else:
                loads it into traj_controller
                loads goal into pose controller
                sets self.current_plan_start_time
                if not aligned:
                    sets mode to ALIGN
                else:
                    sets mode to TRACK
        else:
            sets mode to PARK
        """
        # Make sure we have a map
        if not self.occupancy:
            rospy.loginfo("Navigator: re-planning cancelled, waiting for occupancy map.")
            self.switch_mode(Mode.IDLE)
            return

        # Attempt to plan a path
        state_min = self.snap_to_grid((-self.plan_horizon, -self.plan_horizon))
        state_max = self.snap_to_grid((self.plan_horizon, self.plan_horizon))
        x_init = self.snap_to_grid((self.x, self.y))
        self.plan_start = x_init
        x_goal = self.snap_to_grid((self.x_g, self.y_g))
        problem = AStar(state_min, state_max, x_init, x_goal, self.occupancy, self.plan_resolution)

        rospy.loginfo("Navigator: computing navigation plan")
        success = problem.solve()
        if not success:
            rospy.loginfo("Planning Failed")
            return
        rospy.loginfo("Planning Succeeded")

        planned_path = problem.path

        # Check whether path is too short
        if len(planned_path) < 4:
            rospy.loginfo("Path too short to track")
            self.pose_controller.load_goal(self.x_g, self.y_g, self.theta_g)
            self.switch_mode(Mode.PARK)
            return

        # Smooth and generate a trajectory
        traj_new, t_new = compute_smoothed_traj(planned_path, self.v_des, self.spline_alpha, self.traj_dt)

        # If currently tracking a trajectory, check whether new trajectory will take more time to follow
        if self.mode == Mode.TRACK:
            t_remaining_curr = self.current_plan_duration - self.get_current_plan_time()

            # Estimate duration of new trajectory
            th_init_new = traj_new[0, 2]
            th_err = wrapToPi(th_init_new - self.theta)
            t_init_align = abs(th_err/self.om_max)
            t_remaining_new = t_init_align + t_new[-1]

            if t_remaining_new > t_remaining_curr and not self.goal_updated and not self.occupancy_updated:
                rospy.loginfo("New plan rejected (longer duration than current plan without changes in goal or map)")
                self.publish_smoothed_path(traj_new, self.nav_smoothed_path_rej_pub)
                return

        # Otherwise follow the new plan
        self.publish_planned_path(planned_path, self.nav_planned_path_pub)
        self.publish_smoothed_path(traj_new, self.nav_smoothed_path_pub)

        self.pose_controller.load_goal(self.x_g, self.y_g, self.theta_g)
        self.traj_controller.load_traj(t_new, traj_new)

        self.current_plan_start_time = rospy.get_rostime()
        self.current_plan_duration = t_new[-1]

        self.th_init = traj_new[0, 2]
        self.heading_controller.load_goal(self.th_init)

        if not self.aligned():
            rospy.loginfo("Not aligned with start direction")
            self.switch_mode(Mode.ALIGN)
            return

        rospy.loginfo("Ready to track")
        self.switch_mode(Mode.TRACK)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # try to get state information to update self.x, self.y, self.theta
            try:
                (translation, rotation) = self.trans_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
                self.x = translation[0]
                self.y = translation[1]
                euler = tf.transformations.euler_from_quaternion(rotation)
                self.theta = euler[2]
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                # self.current_plan = []
                rospy.loginfo("Navigator: waiting for state info")
                self.switch_mode(Mode.IDLE)
                print e
                pass

            # STATE MACHINE LOGIC
            # some transitions handled by callbacks
            if self.mode == Mode.IDLE:
                pass
            elif self.mode == Mode.ALIGN:
                if self.aligned():
                    self.current_plan_start_time = rospy.get_rostime()
                    self.switch_mode(Mode.TRACK)
            elif self.mode == Mode.TRACK:
                if self.near_goal():
                    self.switch_mode(Mode.PARK)
                elif not self.close_to_plan_start():
                    rospy.loginfo("replanning because far from start")
                    self.replan()
                elif (rospy.get_rostime() - self.current_plan_start_time).to_sec() > self.current_plan_duration:
                    rospy.loginfo("replanning because out of time")
                    self.replan()  # we aren't near the goal but we thought we should have been, so replan
            elif self.mode == Mode.PARK:
                if self.at_goal():
                    # forget about goal:
                    self.reached_goal()

            self.publish_control()
            rate.sleep()

if __name__ == '__main__':    
    nav = Navigator()
    rospy.on_shutdown(nav.shutdown_callback)
    nav.run()
