#!/usr/bin/env python
import numpy as np


class FlagLocalizer(object):
    """Parent class which FlagLocalizerNode inherits from"""
    def __init__(self):
        self.detected_flags = {}  # {(int)id: np.array([x, y])}
        self.flag_counts = {}  # {(int) id: int num_times_seen}
        self.oracle_flags = {}
        self.flag_move_threshold = 0.03
        self.mapping_done = False

    def object_detected(self, msg, robot_pose):
        map_changed = False
        flags_detected = msg.ob_msgs  # list of DetectedObject messages

        for flag in flags_detected:
            # Check if oracle has placed the object somewhere (don't move it until game starts)
            if flag.id in self.oracle_flags.keys() and not self.mapping_done:
                continue

            flag_pos = self.calc_flag_position(flag, robot_pose)
            if flag.id in self.detected_flags.keys():

                # If new location is far from old location, move flag
                if np.linalg.norm(flag_pos - self.detected_flags[flag.id]) > self.flag_move_threshold:
                    self.detected_flags[flag.id] = flag_pos
                    self.flag_counts[flag.id] = 1

                # Else average the position with the old position
                else:
                    self.detected_flags[flag.id] = (self.detected_flags[flag.id] *
                                                    self.flag_counts[flag.id] + flag_pos) / (self.flag_counts[flag.id] + 1)
                    self.flag_counts[flag.id] += 1

            # Else add flag to detected flags
            else:
                self.detected_flags[flag.id] = flag_pos
                self.flag_counts[flag.id] = 1
            map_changed = True

        if map_changed:
            return True
        return False

    def calc_flag_position(self, flag_object, robot_pose):
        # Angles from detector_mobile_net are in range (0, 2pi]
        theta_left = self.wrap2pi(flag_object.thetaleft)
        theta_right = self.wrap2pi(flag_object.thetaright)

        if theta_left > 0.0 > theta_right:
            theta_avg = theta_left + theta_right + robot_pose[2]
        else:
            theta_avg = (theta_left + theta_right) / 2.0 + robot_pose[2]

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
        map_changed = False
        flag_id = int(msg.theta)  # used the theta slot of Pose2D for the flag id
        flag_pos = np.array([msg.x, msg.y])

        if flag_id in self.detected_flags.keys():
            map_changed = True
            
        self.detected_flags[flag_id] = flag_pos
        self.flag_counts[flag_id] = 1
        return map_changed



