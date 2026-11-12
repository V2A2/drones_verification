"""
Geometry of a received ProbStar (mean, std) for plotting. numpy only, no ROS / Qt imports.

The star is X = mean + diag(std) alpha, alpha in [-n_sigma, n_sigma]^6, alpha ~ N(0, I) (build_probstar). Projected
on k state axes (k = 2 or 3) this is a zonotope: center + G @ alpha, G (k x 6). Its Gaussian covariance is G G^T.
"""
import itertools

import numpy as np

AXES = ('x', 'y', 'z', 'vx', 'vy', 'vz')
N_SIGMA = 3.0  # build_probstar default


def projection(mean, std, axes):
    """-> (center (k,), generators G (k, 6)) of the star on state indices `axes`."""
    mean = np.asarray(mean, float)
    G = np.diag(np.asarray(std, float))[list(axes)]
    return mean[list(axes)], G


def _hull_2d(pts):
    """Andrew monotone chain, counter-clockwise, (m, 2) -> (h, 2)"""
    pts = sorted(set(map(tuple, np.round(pts, 12))))
    if len(pts) < 3:
        return np.array(pts)

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return np.array(lower[:-1] + upper[:-1])


def zonotope_polygon(center, G, n_sigma=N_SIGMA):
    """2D support: convex polygon (h, 2), the hull of the 2^6 mapped corners of the predicate box."""
    signs = np.array(list(itertools.product((-1.0, 1.0), repeat=G.shape[1])))
    return _hull_2d(center + n_sigma * signs @ G.T)


def zonotope_faces(center, G, n_sigma=N_SIGMA):
    """
    3D support: list of parallelogram faces, each (4, 3). For every pair of generators (i, j) the extreme face in
    direction +-(g_i x g_j) is center + sum_k s_k g_k +- g_i +- g_j with s_k = sign(g_k . n); generators parallel
    to the face (g_k . n = 0) are in the face plane and are left out, parallel pairs give the same face (deduped).
    """
    g = n_sigma * G.T  # (6, 3), scaled generators
    faces, seen = [], set()
    for i, j in itertools.combinations(range(len(g)), 2):
        n = np.cross(g[i], g[j])
        norm = np.linalg.norm(n)
        if norm < 1e-9 * (np.linalg.norm(g[i]) * np.linalg.norm(g[j]) + 1e-300):
            continue
        n /= norm
        for sgn in (1.0, -1.0):
            nn = sgn * n
            dots = g @ nn
            key = tuple(np.round(nn, 6)) + (round(float(np.sum(np.abs(dots))), 6),)
            if key in seen:
                continue
            seen.add(key)
            base = center + (np.where(dots > 1e-9, 1.0, -1.0)[:, None] * g)[
                [k for k in range(len(g)) if abs(dots[k]) > 1e-9]].sum(axis=0)
            # the face spans every generator in its plane; with >2 of them it is a larger polygon
            inplane = [k for k in range(len(g)) if abs(dots[k]) <= 1e-9]
            gi = g[inplane]
            corners = np.array([base + s @ gi for s in itertools.product((-1.0, 1.0), repeat=len(gi))])
            # order the corners around the face center
            u = gi[0] / np.linalg.norm(gi[0])
            w = np.cross(nn, u)
            rel = corners - corners.mean(axis=0)
            hull = _hull_2d(np.column_stack((rel @ u, rel @ w)))
            ang = np.array([corners.mean(axis=0) + a * u + b * w for a, b in hull])
            faces.append(ang)
    return faces


def covariance(G):
    return G @ G.T


def ellipse_2d(center, G, k_sigma, n=64):
    """k_sigma-contour of the projected Gaussian, (n, 2)"""
    w, v = np.linalg.eigh(covariance(G))
    th = np.linspace(0, 2 * np.pi, n, endpoint=False)
    unit = np.stack((np.cos(th), np.sin(th)))
    return center + (v @ (np.sqrt(np.maximum(w, 0))[:, None] * unit * k_sigma)).T


def ellipsoid_circles(center, G, k_sigma, n=48):
    """3 great circles of the k_sigma-ellipsoid of the projected Gaussian, list of (n, 3)"""
    w, v = np.linalg.eigh(covariance(G))
    M = v * (np.sqrt(np.maximum(w, 0)) * k_sigma)  # columns = semi-axes
    th = np.linspace(0, 2 * np.pi, n)
    c, s, z = np.cos(th), np.sin(th), np.zeros_like(th)
    return [center + (M @ np.stack(u)).T for u in ((c, s, z), (c, z, s), (z, c, s))]
