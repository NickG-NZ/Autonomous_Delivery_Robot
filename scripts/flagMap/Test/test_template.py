from unittest import TestCase


class TestTemplate(TestCase):
	def my_test(self):
		a = 2
		b = 2
		assert a == b, "Assert {} == {} is false".format(a, b)

