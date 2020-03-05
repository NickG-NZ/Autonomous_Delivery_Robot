"""Classes emulating ROS message types for testing"""


class DetectedObject(object):
    def __init__(self, f_id=0, name="000", distance=0, thetaleft=0, thetaright=0, corners=[]):
        self.f_id = f_id  # int
        self.name = name  # string
        self.confidence = 0  # float
        self.distance = distance  # float
        self.thetaleft = thetaleft  # float
        self.thetaright = thetaright # float
        self.corners = corners  # float[]


class DetectedObjectList(object):
    def __init__(self, objects, ob_msgs):
        self.objects = objects  # string[]
        self.ob_msgs = ob_msgs  # DetectedObject[]


class FlagMap(object):
    def __init__(self, objects, coordinates):
        self.objects = objects  # string[]
        self.coordinates = coordinates  # geometry_msgs/Pose2D[]


class Pose2D(object):
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta