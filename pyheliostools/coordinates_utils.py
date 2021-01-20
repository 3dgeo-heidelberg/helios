import numpy as np


def cartesianToSpherical(coords):
    """Compute the spherical coordinates corresponding to given cartesian
    coordinates.

    Arguments:
        coords -- Numpy array (matrix) of cartesian coordinates (x, y, z)

    Return:
        Numpy array of spherical coordinates.
        x' := Distance (r)
        y' := Inclination angle (phi)
        z' := Azimuth angle (theta)
    """
    # Prepare (pre-computations)
    xySquare = np.power(coords[:, 0], 2) + np.power(coords[:, 1], 2)

    # Compute spherical coordinates
    spher = np.ndarray(np.shape(coords))
    spher[:, 0] = np.sqrt(xySquare + np.power(coords[:, 2], 2))
    spher[:, 1] = np.arctan2(coords[:, 1], coords[:, 0])
    spher[:, 2] = np.arctan2(np.sqrt(xySquare), coords[:, 2])
    return spher


def sphericalToCartesian(coords):
    """Compute the cartesian coordinates corresponding to given spherical
    coordinates.

    Arguemnts:
        coords -- Numpy array (matrix) of spherical coordinates (r, phi, theta)

    Return:
        Numpy array of cartesian coordinates.
        r' := X cartesian coordinate (x)
        phi' := Y cartesian coordinate (y)
        theta' := Z cartesian coordinate (z)
    """
    # Prepare (pre-computations)
    rsintheta = coords[:, 0] * np.sin(coords[:, 2])

    # Compute cartesian coordinates
    cart = np.ndarray(np.shape(coords))
    cart[:, 0] = rsintheta * np.cos(coords[:, 1])
    cart[:, 1] = rsintheta * np.sin(coords[:, 1])
    cart[:, 2] = coords[:, 0] * np.cos(coords[:, 2])
    return cart
