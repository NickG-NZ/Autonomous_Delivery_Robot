from unittest import TestCase


class TestTemplate(TestCase):
	def test_example(self):
		a = 2
		b = 2
		self.assertEqual(a, b)  # "Assert {} == {} is false".format(a, b)

