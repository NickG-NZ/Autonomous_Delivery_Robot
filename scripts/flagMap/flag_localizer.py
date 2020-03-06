#!/usr/bin/env python
import numpy as np
import json


class FlagLocalizer(object):
    """Parent class which FlagLocalizerNode inherits from"""
    def __init__(self):
        self.detected_flags = {}  # {(int)id: np.array([x, y])}
        self.flag_counts = {}  # {(int) id: int num_times_seen}
        self.oracle_flags = {}
        self.flag_move_threshold = 0.05
        self.game_started = False
        self.opponent_pose = None  # numpy array
        self.flag_is_opponent_tol = 0.01
        self.opponent_flag = None  # int
        self.map_saves_count = 0

    def object_detected(self, msg, robot_pose):
        map_changed = False
        opponent_detected = False
        flags_detected = msg.ob_msgs  # list of DetectedObject messages

        for flag in flags_detected:
            # Check if oracle has placed this flag somewhere (don't move it until game starts)
            if flag.id in self.oracle_flags.keys() and not self.game_started:
                continue

            flag_pos = self.calc_flag_position(flag, robot_pose)

            # Check if the flag is the opponent
            if not self.opponent_flag and self.game_started and self.opponent_pose and \
                    (np.linalg.norm(self.opponent_pose[:2] - flag_pos) < self.flag_is_opponent_tol):
                opponent_detected = True
                self.opponent_flag = flag.id

            elif flag.id in self.detected_flags.keys():
                # If new location is far from old location, move flag
                if np.linalg.norm(flag_pos - self.detected_flags[flag.id]) > self.flag_move_threshold:
                    self.detected_flags[flag.id] = flag_pos
                    self.flag_counts[flag.id] = 1
                    map_changed = True

                # Else average the position with the old position
                else:
                    self.detected_flags[flag.id] = (self.detected_flags[flag.id] *
                                                    self.flag_counts[flag.id] + flag_pos) / (self.flag_counts[flag.id] + 1)
                    self.flag_counts[flag.id] += 1
                    map_changed = True

            # If never seen before, add the flag to detected flags
            else:
                self.detected_flags[flag.id] = flag_pos
                self.flag_counts[flag.id] = 1
                map_changed = True

        return map_changed, opponent_detected

    def calc_flag_position(self, flag_object, robot_pose):
        # Angles from detector_mobile_net are in range (0, 2pi]
        theta_left = self.wrap2pi(flag_object.thetaleft)
        theta_right = self.wrap2pi(flag_object.thetaright)
        theta_avg = (theta_left + theta_right) / 2.0 + robot_pose[2]  # doesn't need to be wrapped to [-pi, pi)

        flag_position = np.zeros(2)
        flag_position[0] = robot_pose[0] + flag_object.distance*np.cos(theta_avg)
        flag_position[1] = robot_pose[1] + flag_object.distance*np.sin(theta_avg)
        return flag_position

    @staticmethod
    def wrap2pi(angle):
        # Wraps any angle to the range (-pi, pi]
        wrapped_angle = (angle + np.pi) % (2*np.pi) - np.pi
        return wrapped_angle

    def oracle_correction(self, msg):
        moved_flag = False
        flag_id = int(msg.theta)  # used the theta slot of Pose2D for the flag id
        flag_pos = np.array([msg.x, msg.y])
        if flag_id in self.detected_flags.keys():
            moved_flag = True
        self.detected_flags[flag_id] = flag_pos
        self.flag_counts[flag_id] = 1
        return moved_flag

    def save_food_map(self):
        self.maybe_make_dir('FoodMaps')
        filename = 'FoodMaps/foodMap_{}.json'.format(self.map_saves_count)
        with open(filename, 'w') as fp:
            json.dump(self.detected_flags, fp, indent=4)
        self.map_saves_count += 1

    @staticmethod
    def maybe_make_dir(dir_name):
        





