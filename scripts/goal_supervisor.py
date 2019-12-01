
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
		self.rviz_subscriber =  \
		rospy.Subscriber('rviz_cmd', Pose2D, self.rviz_callback)

		# State Machine Subscriptions
		rospy.Subscriber('/reached_goal', Bool, self.reached_goal_callback)
		rospy.Subscriber('/resume_vending', Bool, self.resume_vending_callback)

		"""
		ROS Publishers
		"""
		self.cmd_nav_publisher = rospy.Publisher('/cmd_nav', Pose2D, queue_size=1)
		self.request_vending_cmd = rospy.Publisher('/request_vending_cmd', Bool, queue_size=1)
		self.request_vending_replan = rospy.Publisher('/request_vending_replan', Bool, queue_size=1)

	def main_loop(self):

		if self.publish_cmd:
			if self.rviz_control:
				self.publish(self.rviz_cmd_cache)
			elif self.laser_pointer_control:
				self.publish(self.laser_pointer_control)
			elif self.vending_control:
				self.publish(self.vending_cmd_cache)

			self.publish_cmd = False
		

	def idle(self):
		self.vending_control = False
		self.laser_pointer_control = False
		self.rviz_control = False

	def laser_pointer_callback(self, msg):
		# laser_pointer has priority over vending but no priority over rviz
		if not self.rviz_control:
			self.laser_pointer_control = True
			self.vending_control = False
			self.rviz_control = False
			self.publish_cmd = True

		self.laser_pointer_cmd_cache = msg.pose
		self.laser_pointer_cmd_time = msg.header.stamp

	def vending_callback(self, msg):
		self.laser_pointer_cmd_cache = msg.pose
		if self.vending_control:
			self.publish_cmd = True

	def rviz_callback(self, msg):
		# rviz has first priority over laser_pointer_control and vending
		self.vending_control = False
		self.laser_pointer_control = False
		self.rviz_control = True

		self.rviz_cmd_cache = msg.pose
		self.publish_cmd = True

	def reached_goal_callback(self, msg):

		if self.rviz_control:
			self.idle()
		elif self.laser_pointer_control:
			self.idle()
		elif self.vending_control:
			msg = Bool()
			self.request_vending_cmd(msg)

	def resume_vending_callback(self, msg):
		self.vending_control = True
		self.laser_pointer_control = False
		self.rviz_control = False
		self.publish_cmd = True

		msg = Bool()
		self.request_vending_replan(msg)

	def publish_cmd_nav(self, msg):
		self.cmd_nav_publisher.publish(msg)

	def run(self):
		self.main_loop()
		rospy.spin()


if __name__ == '__main__':
	gs = GoalSupervisor()
	gs.run()
