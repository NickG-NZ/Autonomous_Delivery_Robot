import unittest
import numpy as np
from numpy import pi, cos, sin
from flag_localizer import FlagLocalizer
from utils import DetectedObject, Pose2D


class FlagLocalizerTest(unittest.TestCase):

	def test_wrap2pi(self):
		flg = FlagLocalizer()
		self.assertEqual(flg.wrap2pi(0), 0)  # zero
		self.assertEqual(abs(flg.wrap2pi(-pi)), pi)  # -pi
		self.assertEqual(abs(flg.wrap2pi(pi)), pi)  # pi
		self.assertEqual(flg.wrap2pi(-pi/4), -pi/4)  # small negative
		self.assertAlmostEqual(flg.wrap2pi(7 * pi / 4), -pi/4, places=6)  # angle in 4th quadrant
		self.assertAlmostEqual(flg.wrap2pi(5 * pi / 4), -3*pi/4, places=6)  # angle in 3rd quadrant
		self.assertEqual(flg.wrap2pi(3*pi/4), 3*pi/4)  # angle in 2nd quadrant
		self.assertAlmostEqual(flg.wrap2pi(pi/3), pi/3, places=6)  # angle in 1st quadrant
		self.assertAlmostEqual(flg.wrap2pi(2*pi), 0, places=6)  # 2pi
		self.assertAlmostEqual(flg.wrap2pi(3.5*pi), -0.5*pi, places=6)  # angle with extra 2pi added to it

	def test_calc_flag_position(self):
		flg = FlagLocalizer()
		# Set: Robot(x,y,th), Flag(d,th_l,th_r)
		# ---------------------------------
		# R(0,0,0), F(0,0,0)
		robot_pose = [0.0, 0.0, 0.0]
		flag = DetectedObject(distance=0.0, thetaleft=0.0, thetaright=0.0)
		flg_pos = flg.calc_flag_position(flag, robot_pose)
		self.assertAlmostEqual(flg_pos[0], 0.0, places=6)
		self.assertAlmostEqual(flg_pos[1], 0.0, places=6)

		# R(0,0,0), F(5,0,0)
		# Flag directly in front of robot
		flag = DetectedObject(distance=5.0, thetaleft=0.0, thetaright=0.0)
		flg_pos = flg.calc_flag_position(flag, robot_pose)
		self.assertAlmostEqual(flg_pos[0], 5.0, places=6)
		self.assertAlmostEqual(flg_pos[1], 0.0, places=6)

		# Test 3 different cases for theta_l, theta_r of flag relative to robot:
		# ==========================================================================
		# 1. a) Either side of x-axis (biased +ve) R(0,0,0), F(1,3*pi/8,-pi/8)
		flag = DetectedObject(distance=1.0, thetaleft=3*pi/8, thetaright=-pi/8)
		flg_pos = flg.calc_flag_position(flag, robot_pose)
		self.assertAlmostEqual(flg_pos[0], 1.0*cos(pi/8), places=6)
		self.assertAlmostEqual(flg_pos[1], 1.0*sin(pi/8), places=6)

		# 1. b) Either side of x-axis (biased -ve) R(0,0,0), F(1,pi/8,-3*pi/8)
		flag = DetectedObject(distance=1.0, thetaleft=pi/8, thetaright=-3*pi/8)
		flg_pos = flg.calc_flag_position(flag, robot_pose)
		self.assertAlmostEqual(flg_pos[0], 1.0*cos(-pi/8), places=6)
		self.assertAlmostEqual(flg_pos[1], 1.0*sin(-pi/8), places=6)

		# 2. Both positive R(0,0,0), F(1,pi/2,pi/4)
		flag = DetectedObject(distance=1.0, thetaleft=pi/2, thetaright=pi/4)
		flg_pos = flg.calc_flag_position(flag, robot_pose)
		self.assertAlmostEqual(flg_pos[0], 1.0*cos(3*pi/8), places=6)
		self.assertAlmostEqual(flg_pos[1], 1.0*sin(3*pi/8), places=6)

		# 3. Both negative R(0,0,0), F(1,-pi/4,-pi/2)
		flag = DetectedObject(distance=1.0, thetaleft=-pi/4, thetaright=-pi/2)
		flg_pos = flg.calc_flag_position(flag, robot_pose)
		self.assertAlmostEqual(flg_pos[0], 1.0*cos(-3*pi/8), places=6)
		self.assertAlmostEqual(flg_pos[1], 1.0*sin(-3*pi/8), places=6)
		# =========================================================================

		# Test robot and flag positions/poses in different quadrants
		# =========================================================================
		# R(2.5,1,-3*pi/4), F(0.5,-pi/6,-3pi/6)
		# Robot in 1st quadrant, robot angle in 3rd quadrant, flag in 2nd quadrant
		robot_pose = (2.5, 1.0, -3*pi/4)
		flag = DetectedObject(distance=0.5, thetaleft=-pi/6, thetaright=-pi/2)
		flg_pos = flg.calc_flag_position(flag, robot_pose)
		self.assertAlmostEqual(flg_pos[0], 2.0170370868554657, places=6)
		self.assertAlmostEqual(flg_pos[1], 1.1294095225512604, places=6)

		# R(-1,0,3*pi/4), F(1.2,1.0,0.2)
		# Robot on border of 2nd and 3rd quadrant, angle in 2nd, flag in 2nd
		robot_pose = [-1.0, 0.0, 3*pi/4]
		flag = DetectedObject(distance=1.2, thetaleft=1.0, thetaright=0.2)
		flg_pos = flg.calc_flag_position(flag, robot_pose)
		self.assertAlmostEqual(flg_pos[0], -2.1794355183291723, places=6)
		self.assertAlmostEqual(flg_pos[1], 0.22120546580859393, places=6)

		# R(1,-1,-pi/4), F(0.3,3pi/8,-pi/8)
		# Robot in 4th quadrant, angle in 4th quadrant, flag in 1st quadrant (theta either side of x)
		robot_pose = [1.0, -1.0, -pi/4]
		flag = DetectedObject(distance=0.3, thetaleft=3*pi/8, thetaright=-pi/8)
		flg_pos = flg.calc_flag_position(flag, robot_pose)
		self.assertAlmostEqual(flg_pos[0], 1.277163859753386, places=6)
		self.assertAlmostEqual(flg_pos[1], -1.1148050297095269, places=6)

	def test_oracle_correction(self):
		flg = FlagLocalizer()

		# Add some flags to detected_flags and some counts to flag_counts
		for i in range(14, 30, 2):
			flg.detected_flags[i] = np.random.rand(2)
			flg.flag_counts[i] = np.random.randint(1, 5, 1)[0]

		# Flag already in detected flags
		ID = 201
		flg.detected_flags[ID] = np.array([0.1, -0.5])
		flg.flag_counts[ID] = 5

		msg = Pose2D(0.5, -1.2, float(ID))
		moved_flag = flg.oracle_correction(msg)
		self.assertTrue(moved_flag)
		self.assertEqual(flg.detected_flags[ID][0], msg.x)
		self.assertEqual(flg.detected_flags[ID][1], msg.y)
		self.assertEqual(len(flg.detected_flags.keys()), 9)
		self.assertEqual(flg.flag_counts[ID], 1)

		# Flag not seen before
		ID = 65
		msg = Pose2D(-0.8, 0.2, float(ID))
		moved_flag = flg.oracle_correction(msg)
		self.assertFalse(moved_flag)
		self.assertEqual(flg.detected_flags[ID][0], msg.x)
		self.assertEqual(flg.detected_flags[ID][1], msg.y)
		self.assertEqual(len(flg.detected_flags.keys()), 9)
		self.assertEqual(flg.flag_counts[ID], 1)

