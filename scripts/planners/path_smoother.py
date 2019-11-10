import numpy as np
import scipy.interpolate

def compute_smoothed_traj(path, V_des, alpha, dt):
    """
    Fit cubic spline to a path and generate a resulting trajectory for our
    wheeled robot.

    Inputs:
        path (np.array [N,2]): Initial path
        V_des (float): Desired nominal velocity, used as a heuristic to assign nominal
            times to points in the initial path
        alpha (float): Smoothing parameter (see documentation for
            scipy.interpolate.splrep)
        dt (float): Timestep used in final smooth trajectory
    Outputs:
        traj_smoothed (np.array [N,7]): Smoothed trajectory
        t_smoothed (np.array [N]): Associated trajectory times
    Hint: Use splrep and splev from scipy.interpolate
    """
    # estimate time at each point along path
    path = np.array(path)
    N = len(path[:, 0])
    t = np.zeros(N)
    for iter in range(N - 1):
        ds = np.linalg.norm(path[iter + 1, :] - path[iter, :])
        dtime = ds / V_des
        t[iter + 1] = t[iter] + dtime
    # spline interpolation for x and y against time
    x_tck = scipy.interpolate.splrep(t, path[:, 0], k=3, s=alpha)
    y_tck = scipy.interpolate.splrep(t, path[:, 1], k=3, s=alpha)
    # get time for new trajectory
    nstep = np.ceil((t[-1] - t[0]) / dt)
    t_smoothed = np.linspace(t[0], t[-1], nstep)
    # find smooth trajectory
    x_smoothed = scipy.interpolate.splev(t_smoothed, x_tck)
    y_smoothed = scipy.interpolate.splev(t_smoothed, y_tck)
    xd_smoothed = np.gradient(x_smoothed)
    yd_smoothed = np.gradient(y_smoothed)
    xdd_smoothed = np.gradient(xd_smoothed)
    ydd_smoothed = np.gradient(yd_smoothed)
    theta = np.arctan2(yd_smoothed, xd_smoothed)
    traj_smoothed = np.transpose(
        np.vstack((x_smoothed, y_smoothed, theta, xd_smoothed, yd_smoothed, xdd_smoothed, ydd_smoothed)))
    return traj_smoothed, t_smoothed
