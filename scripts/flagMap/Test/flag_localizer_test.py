import unittest
import numpy as np
from flag_localizer import FlagLocalizer

class FlagLocalizerTest(unittest.TestCase):

    def test_wrap2pi(self):
    	flag_localizer = FlagLocalizer()
    	# angles = np.arange(-np.pi, 3*np.pi, np.pi/4)
    	a = 0
    	self.assertEqual(FlagLocalizer.wrap2pi(a), 0)
