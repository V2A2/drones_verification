import numpy as np
from StarV.set.probstar import ProbStar


def collision_constraint(r_self, r_other):
    """
    Collision set H = {X | H X <= d} of the concatenated state X = [X_self; X_other] (12,).

    Both bodies are axis-aligned boxes with half extents r_self, r_other (the state has no
    heading), so the Minkowski difference B_self - B_other = {p | C p <= d} is the box with
    half extents r_self + r_other:

        C = [I3; -I3],  d = [r; r]

    and the relative position p = pos_other - pos_self gives H = [-C 0 C 0].
    """
    r = np.asarray(r_self, dtype=float) + np.asarray(r_other, dtype=float)
    if r.shape != (3,):
        raise ValueError("half extents must have shape (3,)")

    C = np.vstack((np.eye(3), -np.eye(3)))
    Z = np.zeros((6, 3))
    H = np.hstack((-C, Z, C, Z))
    d = np.concatenate((r, r))
    return H, d


def _constrained_probability(star, H, d):
    """P(star in {X | H X <= d} and predicates in their box), 0 when the tightened predicate bounds cross (empty)."""
    S = star.addMultipleConstraints(H, d)
    if np.any(S.pred_lb > S.pred_ub):
        return 0.0
    return float(S.estimateProbability())


def collision_probability(own, other, H, d, p_G=None):
    """
    Collision probability P_G(H Z <= d) of the joint ProbStar Z = [own; other] (exact for the linear model, no
    linearization remainder):

        P = P(H Z <= d and G) / P(G)

    P_G conditions on G = all predicates in their +-n_sigma box. The constrained estimate can slightly exceed P(G)
    (StarV's auxiliary variable approximation), so it is clipped to 1.

    P(G) depends only on the predicates (an affine map keeps them), so it can be passed in instead of estimated again.
    """
    joint = own.concatenate(other)
    if p_G is None:
        p_G = float(joint.estimateProbability())
    return min(_constrained_probability(joint, H, d) / p_G, 1.0)


def collision_probability_bounds(own, other, H, d, err):
    """
    Bounds P_L <= P_G(H Z <= d) <= P_U on the collision probability of the true joint state Z = Z^L + eps
    (UAV_Linearization_ZOH_and_Error_Bounds.md §11-12), Z^L = [own; other] the linearized joint ProbStar.

    err is the joint state error bound |eps| <= [err_self; err_other] (12,) from linearization_error. With
    r = |H| err (= |W| E, W = H K2: each H row has one position of each agent, so no entries cancel)

        P_L = P_G(H Z^L <= d - r)  (inner),   P_U = P_G(H Z^L <= d + r)  (outer)

    P_G conditions on G = all predicates in their +-n_sigma box, which includes the own input domain. The other
    agent's star is rebuilt from mean/std, so its inputs are folded into its state predicates and their box
    stands in for its part of G.

    The constrained estimates can slightly exceed P(G) (StarV's auxiliary variable approximation), so both
    bounds are clipped to 1.

    Returns (P_L, P_U, r), r (6,) the margin per row of H.
    """
    joint = own.concatenate(other)
    r = np.abs(H) @ np.asarray(err, dtype=float)
    p_G = float(joint.estimateProbability())
    for _ in range(10):
        p_lower = _constrained_probability(joint, H, d - r) / p_G
        p_upper = _constrained_probability(joint, H, d + r) / p_G
        if p_lower <= p_upper:
            break

    return min(p_lower, 1.0), min(p_upper, 1.0), r


# ------------------------------------------------------------
# Attitude-driven horizontal dynamics
# ------------------------------------------------------------
#
# Fixed altitude, linearized at the measured attitude u0 (affine, not at hover) and discretized with ZOH:
#
#     X_{k+1} = Ad X_k + Bd u + cd,   u = [roll, pitch, yaw]^T  (the order of rot_to_euler)
#
# Yaw is assumed to stay 0 for the whole flight (the drone faces earth +x), so the heading frame is the
# earth frame and the nonlinear acceleration is
#
#     a(u) = g * [tan(pitch), -tan(roll) / cos(pitch)]
#
# (ENU/FLU: positive roll accelerates towards -y, the opposite sign of Tran's simplified Eq. (3)).
# yaw is a placeholder in u (zero column in B). At u = u0 the prediction equals the nonlinear one.
#
# The measured attitude is held over the whole horizon. This is a modelling assumption: future control
# inputs are not bounded, so the result is not a sound over-approximation of the nonlinear drone. The
# linearization remainder (from the attitude spread around u0) is bounded by linearization_error and enters
# the collision check through collision_probability_bounds (inner/outer constraints), not the set itself.


def rot_to_euler(R):
    """ZYX Euler angles of the rotation matrix R (body -> earth): [roll, pitch, yaw]."""
    R = np.asarray(R, dtype=float)
    roll = np.arctan2(R[2, 1], R[2, 2])
    pitch = np.arcsin(np.clip(-R[2, 0], -1.0, 1.0))
    yaw = np.arctan2(R[1, 0], R[0, 0])
    return np.array([roll, pitch, yaw])


def linear_system_matrices(dt, u0=None, g=None):
    """
    Constant velocity over dt in x, y and z, the same interface as nonlinear_system_matrices:

        X' = Ad X + Bd u + cd,   Ad = I + A dt,   Bd = 0,   cd = 0

    The attitude u0 and g are ignored (no input), so the model is exactly linear and has no linearization remainder.
    """
    #             x    y    z    vx   vy   vz
    A = np.array([
        [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],  # x
        [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],  # y
        [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],  # z
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # vx
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # vy
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   # vz
    ])
    Ad = np.eye(6) + dt * A
    return Ad, np.zeros((6, 3)), np.zeros(6)


def nonlinear_system_matrices(dt, u0, g=9.81):
    """
    ZOH discretization over dt with the input held constant (UAV_Linearization_Steps.md §5):

        X' = Ad X + Bd u + cd,   Ad (6, 6), Bd (6, 3), cd (6,)

        Ad = I + A dt,   Bd = (dt I + 1/2 dt^2 A) B0,   cd = (dt I + 1/2 dt^2 A) c0

    Linearized at u0 = [roll0, pitch0, yaw0] = [phi0, theta0, -] (yaw = 0 assumed, zero yaw column).
    ENU: the phi (roll) terms have the opposite sign of the md.
    """
    u0 = np.asarray(u0, dtype=float)
    phi0, theta0 = u0[0], u0[1]  # roll, pitch
    tan_th, sec_th = np.tan(theta0), 1.0 / np.cos(theta0)
    tan_ph, sec_ph = np.tan(phi0), 1.0 / np.cos(phi0)

    # md §4
    # cx = g * tan_th - g * sec_th**2 * theta0
    # cy = -g * tan_ph * sec_th + g * tan_ph * sec_th * tan_th * theta0 + g * sec_th * sec_ph**2 * phi0

    # md §3
    #             x    y    z    vx   vy   vz
    A = np.array([
        [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],  # x
        [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],  # y
        [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],  # z
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # vx
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # vy
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   # vz
    ])
    #     roll                          pitch                          yaw
    B0 = np.array([
        [ 0.0,                          0.0,                           0.0],  # x
        [ 0.0,                          0.0,                           0.0],  # y
        [ 0.0,                          0.0,                           0.0],  # z
        [ 0.0,                          g * sec_th**2,                 0.0],  # vx
        [-g * sec_th * sec_ph**2,      -g * tan_ph * sec_th * tan_th,  0.0],  # vy
        [ 0.0,                          0.0,                           0.0]   # vz  (fixed altitude)
    ])
    # c0 = h(u0) - B0 u0,  h(u) = [0, 0, 0, g tan(pitch), -g tan(roll) / cos(pitch), 0]  (X' = A X + h(u))
    h0 = np.array([0.0, 0.0, 0.0, g * tan_th, -g * tan_ph * sec_th, 0.0])
    c0 = h0 - B0 @ u0

    # ZOH, exact because A @ A = 0
    I = np.eye(6)
    M = dt * I + 0.5 * dt**2 * A
    Ad = I + dt * A
    Bd = M @ B0
    cd = M @ c0
    return Ad, Bd, cd


def linearization_error(dt, u0, u_std, n_sigma=3, g=9.81):
    """
    Lagrange remainder bound of the system_matrices linearization over dt, for u in the input star's box
    u0 +- r, r = n_sigma * u_std (n_sigma must match build_probstar):

        |h_i(u) - h_i(u0) - B0_i (u - u0)| <= 1/2 sum_jk M_jk r_j r_k,   M_jk = max_box |d2 h_i / du_j du_k|

    with h_vx = g tan(theta), h_vy = -g tan(phi) sec(theta). Every Hessian entry is a product of |tan|, sec
    and sec^2, all increasing in |angle|, so the maxima are at phi_m = |phi0| + r_phi, theta_m = |theta0| + r_theta.
    Held over dt the error enters like the input: e_X = (dt I + 1/2 dt^2 A) e.

    Returns the (6,) absolute error bound of X' = [x, y, z, vx, vy, vz].
    """
    u0 = np.asarray(u0, dtype=float)
    r = n_sigma * np.asarray(u_std, dtype=float)
    r_ph, r_th = r[0], r[1]  # roll, pitch (yaw has no effect)
    phi_m, theta_m = abs(u0[0]) + r_ph, abs(u0[1]) + r_th
    if max(phi_m, theta_m) >= np.pi / 2:
        raise ValueError(f'attitude box reaches +-pi/2 (roll {phi_m:.3f}, pitch {theta_m:.3f}), bound undefined')
    tan_th, sec_th = np.tan(theta_m), 1.0 / np.cos(theta_m)
    tan_ph, sec_ph = np.tan(phi_m), 1.0 / np.cos(phi_m)

    # vx: only d2/dtheta2 = 2 g sec^2 tan
    e_vx = g * sec_th**2 * tan_th * r_th**2
    # vy: Hessian maxima of -g tan(phi) sec(theta)
    M_pp = 2 * g * sec_ph**2 * tan_ph * sec_th
    M_pt = g * sec_ph**2 * sec_th * tan_th
    M_tt = g * tan_ph * sec_th * (tan_th**2 + sec_th**2)
    e_vy = 0.5 * (M_pp * r_ph**2 + 2 * M_pt * r_ph * r_th + M_tt * r_th**2)

    # (dt I + 1/2 dt^2 A) applied to [0, 0, 0, e_vx, e_vy, 0]
    return np.array([0.5 * dt**2 * e_vx, 0.5 * dt**2 * e_vy, 0.0, dt * e_vx, dt * e_vy, 0.0])


def build_probstar(mean, std, n_sigma = 3):
    """
    Build a 6D ProbStar of the state.

    State:
        X = [x, y, z, vx, vy, vz]^T = c + V @ alpha,  alpha ~ N(0, I6)

    Each state has its own independent predicate, truncated to [-n_sigma, n_sigma].
    """
    c = np.asarray(mean, dtype=float)
    V = np.diag(np.asarray(std, dtype=float))
    mu = np.zeros_like(c)
    sigma = np.eye(c.shape[0])
    # No additional constraints C @ alpha <= d
    C = np.empty((0, c.shape[0]))
    d = np.empty((0,))
    pred_lb = -n_sigma * np.ones_like(c)
    pred_ub =  n_sigma * np.ones_like(c)

    return ProbStar(
        np.column_stack((c, V)),
        C,
        d,
        mu,
        sigma,
        pred_lb,
        pred_ub
    )   


def advance_probstar(star, U, Ad, Bd, cd):
    """
    Advance the 6D ProbStar one step:  X' = Ad X + Bd U + cd  (Ad, Bd, cd from system_matrices).

    U is the 3D input ProbStar u = [roll, pitch, yaw], e.g. build_probstar(u_mean, u_std). Its predicates are
    appended to the state's (Minkowski sum). Every call adds an independent input, so advance over the whole
    horizon in one call: chained calls would model a new random attitude per step instead of the held one.
    """
    return star.affineMap(Ad, cd).minKowskiSum(U.affineMap(Bd))
