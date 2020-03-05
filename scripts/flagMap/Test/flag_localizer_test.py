import unittest
import numpy as np
from flag_localizer import FlagLocalizer
from utils import DetectedObject


class FlagLocalizerTest(unittest.TestCase):

	def test_wrap2pi(self):
		flg = FlagLocalizer()
		self.assertEqual(flg.wrap2pi(0), 0)  # zero
		self.assertEqual(abs(flg.wrap2pi(-np.pi)), np.pi)  # -pi
		self.assertEqual(abs(flg.wrap2pi(np.pi)), np.pi)  # pi
		self.assertEqual(flg.wrap2pi(-np.pi/4), -np.pi/4)  # small negative
		self.assertAlmostEqual(flg.wrap2pi(7 * np.pi / 4), -np.pi/4, places=6)  # angle in 4th quadrant
		self.assertAlmostEqual(flg.wrap2pi(5 * np.pi / 4), -3*np.pi/4, places=6)  # angle in 3rd quadrant
		self.assertEqual(flg.wrap2pi(3*np.pi/4), 3*np.pi/4)  # angle in 2nd quadrant
		self.assertAlmostEqual(flg.wrap2pi(np.pi/3), np.pi/3, places=6)  # angle in 1st quadrant
		self.assertAlmostEqual(flg.wrap2pi(2*np.pi), 0, places=6)  # 2pi
		self.assertAlmostEqual(flg.wrap2pi(3.5*np.pi), -0.5*np.pi, places=6)  # angle with extra 2pi added to it

	def test_calc_flag_position(self):
		flg = FlagLocalizer()
		# Set: Robot(x,y,th), Flag(d,th_l,th_r)
		# ---------------------------------
		# R(0,0,0), F(0,0,0)
		robot_pose = [0.0, 0.0, 0.0]
		flag = DetectedObject(distance=0, thetaleft=0, thetaright=0)
		flg_pos = flg.calc_flag_position(flag, robot_pose)
		self.assertAlmostEqual(flg_pos[0], 0.0, places=6)
		self.assertAlmostEqual(flg_pos[1], 0.0, places=6)

		# R(0,0,0), F(5,0,0)
		robot_pose = [0.0, 0.0, 0.0]
		flg = FlagLocalizer()
		flag = DetectedObject(distance=5, thetaleft=0, thetaright=0)
		flg_pos = flg.calc_flag_position(flag, robot_pose)
		self.assertAlmostEqual(flg_pos[0], 5.0, places=6)
		self.assertAlmostEqual(flg_pos[1], 0.0, places=6)

		# R
