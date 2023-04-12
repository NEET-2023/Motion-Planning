import numpy as np
import matplotlib.pyplot as plt
import scipy.interpolate as interpolate
from math import comb

def b_spline_path(path: list, n_path_points: int, degree: int = 3, s=None) -> tuple:
    """
    Approximate points with a B-Spline path

    Parameters
    ----------
    x : array_like
        x position list of approximated points
    y : array_like
        y position list of approximated points
    n_path_points : int
        number of path points
    degree : int, optional
        B Spline curve degree. Must be 2<= k <= 5. Default: 3.
    s : int, optional
        smoothing parameter. If this value is bigger, the path will be
        smoother, but it will be less accurate. If this value is smaller,
        the path will be more accurate, but it will be less smooth.
        When `s` is 0, it is equivalent to the interpolation. Default is None,
        in this case `s` will be `len(x)`.

    Returns
    -------
    x : array
        x positions of the result path
    y : array
        y positions of the result path
    heading : array
        heading of the result path
    curvature : array
        curvature of the result path

    """
    x = [x[0] for i, x in enumerate(path)]
    y = [x[1] for i, x in enumerate(path)]
    z = [x[2] for i, x in enumerate(path)]
    distances = _calc_distance_vector(x, y, z)

    spl_i_x = interpolate.UnivariateSpline(distances, x, k=degree, s=s)
    spl_i_y = interpolate.UnivariateSpline(distances, y, k=degree, s=s)
    spl_i_z = interpolate.UnivariateSpline(distances, z, k=degree, s=s)

    sampled = np.linspace(0.0, distances[-1], n_path_points)
    rix, riy, riz = _evaluate_spline(sampled, spl_i_x, spl_i_y, spl_i_z)

    path = np.array((rix, riy, riz)).T

    # Plotting the smoothed path for debugging
    # pltx = [x[0] for i, x in enumerate(path)]
    # plty = [x[1] for i, x in enumerate(path)]
    # pltz = [x[2] for i, x in enumerate(path)]
    # ax = plt.axes(projection='3d')
    # ax.plot3D(pltx, plty, pltz, 'gray')
    # ax.scatter3D(x, y, z, cmap='Greens')
    # plt.show()

    return path

def _calc_distance_vector(x, y, z):
    dx, dy, dz = np.diff(x), np.diff(y), np.diff(z)
    distances = np.cumsum([np.sqrt(idx**2 + idy**2 + idz**2) for idx, idy, idz in zip(dx, dy, dz)])
    distances = np.concatenate(([0.0], distances))
    distances /= distances[-1]
    return distances

def _evaluate_spline(sampled, spl_i_x, spl_i_y, spl_i_z):
    x = spl_i_x(sampled)
    y = spl_i_y(sampled)
    z = spl_i_z(sampled)
    return np.array(x), np.array(y), np.array(z)


def bezier_path(control_points, n_points=100):
    """
    Compute bezier path (trajectory) given control points.

    :param control_points: (numpy array)
    :param n_points: (int) number of points in the trajectory
    :return: (numpy array)
    """
    traj = []
    for t in np.linspace(0, 1, n_points):
        traj.append(bezier(t, control_points))

    return np.array(traj)

def bezier(t, control_points):
    """
    Return one point on the bezier curve.

    :param t: (float) number in [0, 1]
    :param control_points: (numpy array)
    :return: (numpy array) Coordinates of the point
    """
    n = len(control_points) - 1
    return np.sum([bernstein_poly(n, i, t) * control_points[i] for i in range(n + 1)], axis=0)


def bernstein_poly(n, i, t):
    """
    Bernstein polynom.

    :param n: (int) polynom degree
    :param i: (int)
    :param t: (float)
    :return: (float)
    """
    return comb(n, i) * t ** i * (1 - t) ** (n - i)


if __name__ == '__main__':
    path = np.load('generated_path.npy')
    b_spline_path(path, len(path), s=5)