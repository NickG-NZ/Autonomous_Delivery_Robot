#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D, PoseStamped
from std_msgs.msg import Bool


class GoalSupervisor:
    
    def __init__(self):

        # state machine space
        self.vending_control = False
        self.laser_pointer_control = False
        self.rviz_control = False

        # update flag for self.main_loop()
        self.publish_cmd = False

        rospy.init_node('Goal_Supervisor')
        """
        ROS Subscribers
        """
        # Goal Topic Subscriptions
        self.vending_cmd_cache = None
        self.vending_subscriber = \
            rospy.Subscriber('/vending_cmd', Pose2D, self.vending_callback)

        self.laser_pointer_cmd_cache = None
        self.laser_pointer_subscriber = \
            rospy.Subscriber('/laser_pointer_cmd', Pose2D, self.laser_pointer_callback)

        self.rviz_cmd_cache = None
        self.rviz_subscriber = \
            rospy.Subscriber('rviz_cmd', Pose2D, self.rviz_callback)

        # State Machine Subscriptions
        rospy.Subscriber('/reached_goal', Bool, self.reached_goal_callback)
        rospy.Subscriber('/resume_vending', Bool, self.resume_vending_callback)

        """
        ROS Publishers
        """
        self.cmd_nav_publisher = rospy.Publisher('/cmd_nav', Pose2D, queue_size=1)
        self.request_vending_cmd_publisher = rospy.Publisher('/request_vending_cmd', Bool, queue_size=1)

    def main_loop(self):

        if self.publish_cmd: # if update flag was activated
            self.publish_cmd = False  # ensures each command is sent once

            if self.rviz_control:

                print("rviz mode: ", self.rviz_cmd_cache)
                self.publish_cmd_nav(self.rviz_cmd_cache)

            elif self.laser_pointer_control:

                print("laser pointer mode: ", self.laser_pointer_cmd_cache)
                self.publish_cmd_nav(self.laser_pointer_cmd_cache)

            elif self.vending_control:

                if self.vending_cmd_cache is None:

                    self.request_vending_cmd()

                else:

                    print("vending mode: ", self.vending_cmd_cache)
                    self.publish_cmd_nav(self.vending_cmd_cache)

            else:
                print("idle mode")


    def clear_cmd_caches(self):
    	# clears all command caches so that previous commands don't haunt the supervisor

        self.vending_cmd_cache = None
        self.laser_pointer_cmd_cache = None
        self.rviz_cmd_cache = None

    def idle(self):

        self.clear_cmd_caches()

        self.vending_control = False
        self.laser_pointer_control = False
        self.rviz_control = False

    def laser_pointer_callback(self, msg):
        # laser_pointer has priority over vending but no priority over rviz

        print("GoalSupervisor : laser_pointer mode goal updated")
        if not self.rviz_control:
            self.laser_pointer_control = True
            self.vending_control = False
            self.rviz_control = False
            self.publish_cmd = True # update flag updated

        self.laser_pointer_cmd_cache = msg

    def vending_callback(self, msg):
        print("GoalSupervisor : vending mode goal updated")
        self.vending_cmd_cache = msg
        if self.vending_control:
            self.publish_cmd = True

    def rviz_callback(self, msg):
        print("GoalSupervisor : rviz mode goal updated")
        # rviz has first priority over laser_pointer_control and vending
        
        self.vending_control = False
        self.laser_pointer_control = False
        self.rviz_control = True

        self.rviz_cmd_cache = msg
        self.publish_cmd = True

    def reached_goal_callback(self, msg):

        if self.rviz_control:
            self.idle()
        elif self.laser_pointer_control:
            self.idle()
        elif self.vending_control:
            self.request_vending_cmd()

    def resume_vending_callback(self, msg):

        self.clear_cmd_caches()

        print("Resuming vending mode")

        self.vending_control = True
        self.laser_pointer_control = False
        self.rviz_control = False
        self.publish_cmd = True

    def request_vending_cmd(self):
        msg = Bool()
        self.request_vending_cmd_publisher(msg)

    def publish_cmd_nav(self, msg):
        self.cmd_nav_publisher.publish(msg)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.main_loop()
            rate.sleep()


if __name__ == '__main__':
    gs = GoalSupervisor()
    gs.run()
