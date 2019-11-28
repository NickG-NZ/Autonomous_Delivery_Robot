import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches


# A 2D state space grid with a set of rectangular obstacles. The grid is fully deterministic
class DetOccupancyGrid2D(object):
    def __init__(self, width, height, obstacles):
        self.width = width
        self.height = height
        self.obstacles = obstacles

    def is_free(self, x):
        """
        determine if state x is free of obstacle
        :param x: list / np array, state to check for collision
        :return: boolean, true if collision free, false otherwise
        """
        for obs in self.obstacles:
            inside = True
            for dim in range(len(x)):
                if x[dim] < obs[0][dim] or x[dim] > obs[1][dim]:
                    inside = False
                    break
            if inside:
                return False
        return True

    def plot(self, fig_num=0):
        fig = plt.figure(fig_num)
        for obs in self.obstacles:
            ax = fig.add_subplot(111, aspect='equal')
            ax.add_patch(
            patches.Rectangle(
            obs[0],
            obs[1][0]-obs[0][0],
            obs[1][1]-obs[0][1],))

class StochOccupancyGrid2D(object):
    def __init__(self, resolution, width, height, origin_x, origin_y,
                window_size, probs, thresh=0.5):
        self.resolution = resolution
        # width and height of the map in units of resolution
        self.width = width
        self.height = height
        # map origin in robot's state space
        self.origin_x = origin_x
        self.origin_y = origin_y
        # list, probability of obstacle in particular grid, indexed as grid_y * self.width + grid_x, range (0, 100)
        self.probs = probs
        # window size use in collision checking, units of resolution
        # check for collision in window of specified size around state (x, y only)
        self.window_size = window_size
        # The state is collision free if the probability of obstacle in a window around state is less than thresh
        self.thresh = thresh

    def snap_to_grid(self, x):
        return (self.resolution*round(x[0]/self.resolution), self.resolution*round(x[1]/self.resolution))

    def is_free(self, state):
        """
        Check if state is collision free by calculating the probability that a window around the state does not contain
        any obstacles, assuming independence between each grid point
        :param state: list / np array, state of the robot, first two elements are x and y
        :return: boolean, True if state is collision free, False otherwise
        """
        # combine the probabilities of each cell by assuming independence
        # of each estimation
        # p_total is the probability that the wind around state is free of collision
        p_total = 1.0
        # will not give the right window size if window_size is odd
        lower = -int(round((self.window_size-1)/2))
        upper = int(round((self.window_size-1)/2))
        for dx in range(lower,upper+1):
            for dy in range(lower,upper+1):
                x, y = self.snap_to_grid([state[0] + dx * self.resolution, state[1] + dy * self.resolution])
                grid_x = int((x - self.origin_x) / self.resolution)
                grid_y = int((y - self.origin_y) / self.resolution)
                if grid_y>0 and grid_x>0 and grid_x<self.width and grid_y<self.height:
                    p_total *= (1.0-max(0.0,float(self.probs[grid_y * self.width + grid_x])/100.0))
        return (1.0-p_total) < self.thresh

    def plot(self, fig_num=0):
        fig = plt.figure(fig_num)
        pts = []
        for i in range(len(self.probs)):
            # convert i to (x,y)
            gy = int(i/self.width)
            gx = i % self.width
            x = gx * self.resolution + self.origin_x
            y = gy * self.resolution + self.origin_y
            if not self.is_free((x,y)):
                pts.append((x,y))
        pts_array = np.array(pts)
        plt.scatter(pts_array[:,0],pts_array[:,1],color="red",zorder=15,label='planning resolution')
