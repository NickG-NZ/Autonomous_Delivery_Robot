import unittest
import numpy as np
from numpy import pi, cos, sin
from flag_localizer import FlagLocalizer
from utils import DetectedObject, Pose2D, DetectedObjectList


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
		robot_pose = np.array([0.0, 0.0, 0.0])
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
		robot_pose = np.array([2.5, 1.0, -3*pi/4])
		flag = DetectedObject(distance=0.5, thetaleft=-pi/6, thetaright=-pi/2)
		flg_pos = flg.calc_flag_position(flag, robot_pose)
		self.assertAlmostEqual(flg_pos[0], 2.0170370868554657, places=6)
		self.assertAlmostEqual(flg_pos[1], 1.1294095225512604, places=6)

		# R(-1,0,3*pi/4), F(1.2,1.0,0.2)
		# Robot on border of 2nd and 3rd quadrant, angle in 2nd, flag in 2nd
		robot_pose = np.array([-1.0, 0.0, 3*pi/4])
		flag = DetectedObject(distance=1.2, thetaleft=1.0, thetaright=0.2)
		flg_pos = flg.calc_flag_position(flag, robot_pose)
		self.assertAlmostEqual(flg_pos[0], -2.1794355183291723, places=6)
		self.assertAlmostEqual(flg_pos[1], 0.22120546580859393, places=6)

		# R(1,-1,-pi/4), F(0.3,3pi/8,-pi/8)
		# Robot in 4th quadrant, angle in 4th quadrant, flag in 1st quadrant (theta either side of x)
		robot_pose = np.array([1.0, -1.0, -pi/4])
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
		self.assertEqual(flg.detected_flags[ID].shape, (2,))
		self.assertEqual(len(flg.detected_flags.keys()), 9)
		self.assertEqual(flg.flag_counts[ID], 1)
		self.assertTrue(ID in flg.oracle_flags)

		# Flag not seen before
		ID = 65
		msg = Pose2D(-0.8, 0.2, float(ID))
		moved_flag = flg.oracle_correction(msg)
		self.assertFalse(moved_flag)
		self.assertEqual(flg.detected_flags[ID][0], msg.x)
		self.assertEqual(flg.detected_flags[ID][1], msg.y)
		self.assertEqual(len(flg.detected_flags.keys()), 10)
		self.assertEqual(flg.flag_counts[ID], 1)
		self.assertTrue(ID in flg.oracle_flags)

	def test_update_detected_flags(self):
		flg = FlagLocalizer()

		# Check adding new flag
		# ====================================
		# 1) Empty detected_flags dict
		ID = 3
		flag = DetectedObject(id=ID)
		flag_pos = np.array([3, -2])
		flg.update_detected_flags(flag, flag_pos)
		self.assertTrue(ID in flg.detected_flags)
		assert np.array_equal(flg.detected_flags[ID], flag_pos), "Incorrectly added new flag to empty dict"
		self.assertEqual(flg.flag_counts[ID], 1)

		# 2) Non-empty detected_flags dict
		# Add some flags to detected_flags and some counts to flag_counts
		for i in range(14, 30, 2):
			flg.detected_flags[i] = np.random.rand(2)
			flg.flag_counts[i] = np.random.randint(1, 5, 1)[0]
		ID = 987
		flag = DetectedObject(id=ID)
		flag_pos = np.array([0, -1])
		flg.update_detected_flags(flag, flag_pos)
		self.assertTrue(ID in flg.detected_flags)
		assert np.array_equal(flg.detected_flags[ID], flag_pos), "Incorrectly added new flag to dict with other flags"
		self.assertEqual(flg.flag_counts[ID], 1)

		# Check moving flag to new location (if position change greater than threshold)
		# ===================================================================
		flag_pos = flag_pos + 1.1*flg.flag_move_threshold   # Move it slightly more than allowable tolerance
		flg.flag_counts[ID] = 10  # This should be reset to 1
		flg.update_detected_flags(flag, flag_pos)
		self.assertTrue(ID in flg.detected_flags)
		assert np.array_equal(flg.detected_flags[ID], flag_pos), "Didn't move flag to new location correctly"
		self.assertEqual(flg.flag_counts[ID], 1)

		ID = 3
		flag = DetectedObject(id=ID)
		flag_pos = np.array([2, 0])
		flg.update_detected_flags(flag, flag_pos)
		self.assertTrue(ID in flg.detected_flags)
		assert np.array_equal(flg.detected_flags[ID], flag_pos)
		self.assertEqual(flg.flag_counts[ID], 1)

		# Check averaging flag location (if position change within threshold)
		# ==================================================================
		# Average of two positions
		new_pos = flag_pos + flg.flag_move_threshold*0.5  # Move it slightly less than tolerance
		pos_avg = (flag_pos + new_pos) / 2.0
		flg.update_detected_flags(flag, new_pos)
		self.assertTrue(ID in flg.detected_flags)
		assert np.allclose(flg.detected_flags[ID], pos_avg)
		self.assertEqual(flg.flag_counts[ID], 2)

		# Running average
		ID = 5
		flag = DetectedObject(id=ID)
		flag_pos = np.array([0, 4])
		flg.detected_flags[ID] = flag_pos
		flg.flag_counts[ID] = 4
		new_pos = flag_pos + np.array([0, 1])
		pos_avg = (4*flag_pos + new_pos)/5
		flg.update_detected_flags(flag, flag_pos)
		self.assertTrue(ID in flg.detected_flags)
		assert np.array_equal(flg.detected_flags[ID], pos_avg)
		self.assertEqual(flg.flag_counts[ID], 5)

	def test_object_detected_A(self):
		# Before Game Starts
		# =======================
		flg = FlagLocalizer()

		# Check flags placed by oracle aren't moved before game starts
		# =======================================================
		robot_pose = np.array([0.0, 0.0, 0.0])
		ID = 2
		strID = str(ID).zfill(3)
		msg = DetectedObjectList()
		msg.objects = [strID]
		msg.ob_msgs = [DetectedObject(id=ID, name=strID, distance=2.0, thetaleft=pi / 2.0, thetaright=-pi / 2.0)]
		flag_pos_old = np.array([1, 0])
		flg.detected_flags[ID] = flag_pos_old
		flg.oracle_flags[ID] = flag_pos_old
		flg.flag_counts[ID] = 1

		map_changed, opponent_detected = flg.object_detected(msg, robot_pose)
		self.assertFalse(map_changed)
		self.assertFalse(opponent_detected)
		self.assertEqual(flg.opponent_flag, None)
		assert np.array_equal(flg.detected_flags[ID], flag_pos_old), "Shouldn't have moved a flag placed by oracle"
		assert np.array_equal(flg.oracle_flags[ID], flag_pos_old), "Shouldn't have changed the oracle flags"

		# Check non-oracle flags are moved correctly before game starts
		# =============================================================
		# Move flag to new location
		robot_pose = np.array([1.0, 0.0, 0.0])
		ID = 4
		strID = str(ID).zfill(3)
		msg = DetectedObjectList()
		msg.objects = [strID]
		msg.ob_msgs = [DetectedObject(id=ID, name=strID, distance=2.0, thetaleft=pi / 2.0, thetaright=-pi / 2.0)]
		flag_pos_old = np.array([1, 0])
		flag_pos_new = np.array([3, 0])
		flg.detected_flags[ID] = flag_pos_old
		flg.flag_counts[ID] = 1

		map_changed, opponent_detected = flg.object_detected(msg, robot_pose)
		self.assertTrue(map_changed)
		self.assertFalse(opponent_detected)
		self.assertEqual(flg.opponent_flag, None)
		assert np.array_equal(flg.detected_flags[ID], flag_pos_new), "Incorrectly moved a flag before game started"
		self.assertFalse(ID in flg.oracle_flags)

		# Check new flags are added correctly before game starts
		# ===========================================================
		ID = 7
		strID = str(ID).zfill(3)
		msg = DetectedObjectList()
		msg.objects = [strID]
		msg.ob_msgs = [DetectedObject(id=ID, name=strID, distance=2.0, thetaleft=pi / 2.0, thetaright=-pi / 2.0)]
		flag_pos = np.array([3, 0])

		map_changed, opponent_detected = flg.object_detected(msg, robot_pose)
		self.assertTrue(map_changed)
		self.assertFalse(opponent_detected)
		self.assertEqual(flg.opponent_flag, None)
		assert np.array_equal(flg.detected_flags[ID], flag_pos), "Incorrectly added a flag before game started"
		self.assertFalse(ID in flg.oracle_flags)

	def test_object_detected_B(self):
		# After Game Starts
		# ====================
		flg = FlagLocalizer()
		flg.game_started = True

		# Check opponent flag not set if opponent pose is unknown
		# ========================================================
		robot_pose = np.array([0.0, 0.0, 0.0])
		ID = 0
		strID = str(ID).zfill(3)
		msg = DetectedObjectList()
		msg.objects = [strID]
		msg.ob_msgs = [DetectedObject(id=ID, name=strID, distance=2.0, thetaleft=pi / 2.0, thetaright=-pi / 2.0)]

		map_changed, opponent_detected = flg.object_detected(msg, robot_pose)
		self.assertTrue(map_changed)
		self.assertFalse(opponent_detected)
		self.assertEqual(flg.opponent_flag, None)
		self.assertTrue(ID in flg.detected_flags)

		# Check flags placed by oracle can be moved after game starts
		# ========================================================
		ID = 1
		strID = str(ID).zfill(3)
		msg = DetectedObjectList()
		msg.objects = [strID]
		msg.ob_msgs = [DetectedObject(id=ID, name=strID, distance=2.0, thetaleft=pi / 2.0, thetaright=-pi / 2.0)]
		flag_pos_old = np.array([1, 0])
		flag_pos_new = np.array([2, 0])
		flg.detected_flags[ID] = flag_pos_old
		flg.oracle_flags[ID] = flag_pos_old
		flg.flag_counts[ID] = 1

		map_changed, opponent_detected = flg.object_detected(msg, robot_pose)
		self.assertTrue(map_changed)
		assert np.allclose(flg.detected_flags[ID], flag_pos_new), "Failed to move flag correctly after game started"

		# Check opponent identified correctly if flag not seen before
		# =====================================================
		# 1) Flag detection location and opponent location match exactly
		flg = FlagLocalizer()
		flg.game_started = True

		robot_pose = ([0, 0, -pi/2.0])
		ID = 12
		strID = str(ID).zfill(3)
		msg = DetectedObjectList()
		msg.objects = [strID]
		msg.ob_msgs = [DetectedObject(id=ID, name=strID, distance=1.0, thetaleft=pi / 4.0, thetaright=-pi / 4.0)]
		flg.opponent_pose = np.array([0, -1, 2])

		map_changed, opponent_detected = flg.object_detected(msg, robot_pose)
		self.assertFalse(map_changed)
		self.assertTrue(opponent_detected)
		self.assertEqual(len(flg.detected_flags.keys()), 0)
		self.assertEqual(flg.opponent_flag, ID)
		assert np.allclose(flg.opponent_pose, np.array([0, -1, 2])), "Shouldn't change opponent pose"

		# 2) Flag detection location and opponent location match within tolerance
		flg = FlagLocalizer()
		flg.game_started = True

		robot_pose = ([0, 0, 0])
		ID = 1
		strID = str(ID).zfill(3)
		msg = DetectedObjectList()
		msg.objects = [strID]
		msg.ob_msgs = [DetectedObject(id=ID, name=strID, distance=1.0, thetaleft=pi / 2.0, thetaright=pi / 2.0)]
		flg.opponent_pose = np.array([0, 1 + flg.flag_is_opponent_tol*0.95, 2])

		map_changed, opponent_detected = flg.object_detected(msg, robot_pose)
		self.assertFalse(map_changed)
		self.assertTrue(opponent_detected)
		self.assertEqual(len(flg.detected_flags.keys()), 0)
		self.assertEqual(flg.opponent_flag, ID)

		# Check opponent identified correctly if flag seen previously
		# =========================================================
		flg = FlagLocalizer()
		flg.game_started = True

		robot_pose = ([0, 0, 0])
		ID = 5
		strID = str(ID).zfill(3)
		msg = DetectedObjectList()
		msg.objects = [strID]
		msg.ob_msgs = [DetectedObject(id=ID, name=strID, distance=1.0, thetaleft=3*pi / 4.0, thetaright=pi / 4.0)]
		flg.opponent_pose = np.array([0, 1 + flg.flag_is_opponent_tol*0.95, 2])
		flg.detected_flags[ID] = np.array([10, 5])

		map_changed, opponent_detected = flg.object_detected(msg, robot_pose)
		self.assertFalse(map_changed)
		self.assertTrue(opponent_detected)
		self.assertEqual(len(flg.detected_flags.keys()), 1)
		self.assertEqual(flg.opponent_flag, ID)
		assert np.allclose(flg.detected_flags[ID], np.array([10, 5])), "Detected flags shouldn't have changed"

		# Check opponent flag not overwritten if already set (and handles opponent flag ID == 0)
		# ==============================================================
		flg = FlagLocalizer()
		flg.game_started = True

		robot_pose = ([0, 0, 0])
		ID = 0
		strID = str(ID).zfill(3)
		msg = DetectedObjectList()
		msg.objects = [strID]
		msg.ob_msgs = [DetectedObject(id=ID, name=strID, distance=1.0, thetaleft=pi / 4.0, thetaright=-pi / 4.0)]
		flg.opponent_pose = np.array([1 + flg.flag_is_opponent_tol*0.95, 0.0, 2])
		flg.opponent_flag = ID

		map_changed, opponent_detected = flg.object_detected(msg, robot_pose)
		self.assertFalse(map_changed)
		self.assertFalse(opponent_detected)
		self.assertEqual(len(flg.detected_flags.keys()), 0)
		self.assertEqual(flg.opponent_flag, ID)

	def test_object_detected_C(self):
		# Multiple objects
		# ======================================
		# Test multiple objects before game starts (opponent pose known)
		flg = FlagLocalizer()
		robot_pose = ([0, 0, 0])
		flg.opponent_pose = np.array([0.1, 0.3, 2.4])
		IDs = [3, 97, 15, 0, 6]
		msg = DetectedObjectList([], [])
		dists = np.linspace(0.1, 0.5, 5)
		for i, ID in enumerate(IDs):
			strID = str(ID).zfill(3)
			msg.objects.append(strID)
			msg.ob_msgs.append(DetectedObject(id=ID, name=strID, distance=dists[i], thetaleft=0.2, thetaright=-0.2))
			map_changed, opponent_detected = flg.object_detected(msg, robot_pose)
			self.assertTrue(map_changed)
			self.assertFalse(opponent_detected)

		self.assertTrue(len(flg.detected_flags), 5)
		self.assertEqual(flg.opponent_flag, None)
		for i, ID in enumerate(IDs):
			assert np.allclose(flg.detected_flags[ID], np.array([dists[i], 0])), "Flags placed incorrectly"

		# Test multiple objects after game starts (Opponent pose and flag known)
		# ======================================================================
		flg = FlagLocalizer()
		flg.game_started = True
		robot_pose = ([0, 0, 0])
		flg.opponent_pose = np.array([0.1, 0.3, 2.4])
		flg.opponent_flag = 97
		IDs = [3, 97, 15, 0, 6]
		msg = DetectedObjectList([], [])
		dists = np.linspace(0.1, 0.5, 5)
		for i, ID in enumerate(IDs):
			strID = str(ID).zfill(3)
			msg.objects.append(strID)
			msg.ob_msgs.append(DetectedObject(id=ID, name=strID, distance=dists[i], thetaleft=0.2, thetaright=-0.2))
		map_changed, opponent_detected = flg.object_detected(msg, robot_pose)
		self.assertTrue(map_changed)
		self.assertFalse(opponent_detected)

		self.assertTrue(len(flg.detected_flags), 5)
		self.assertEqual(flg.opponent_flag, 97)
		for i, ID in enumerate(IDs):
			assert np.allclose(flg.detected_flags[ID], np.array([dists[i], 0])), "Flags placed incorrectly"

		# Test multiple objects (including placing the opponent)
		# ==================================================
		flg = FlagLocalizer()
		flg.game_started = True
		robot_pose = ([0, 0, 0])
		flg.opponent_pose = np.array([0.2, 0, -2.4])
		IDs = [3, 97, 15, 0, 6]
		msg = DetectedObjectList([], [])
		dists = np.linspace(0.1, 0.5, 5)
		for i, ID in enumerate(IDs):
			strID = str(ID).zfill(3)
			msg.objects.append(strID)
			msg.ob_msgs.append(DetectedObject(id=ID, name=strID, distance=dists[i], thetaleft=0.2, thetaright=-0.2))
		map_changed, opponent_detected = flg.object_detected(msg, robot_pose)
		self.assertTrue(map_changed)
		self.assertTrue(opponent_detected)
		self.assertTrue(len(flg.detected_flags), 4)
		self.assertEqual(flg.opponent_flag, 97)
		for i, ID in enumerate(IDs):
			if ID == 97:
				continue
			assert np.allclose(flg.detected_flags[ID], np.array([dists[i], 0])), "Flags placed incorrectly"
