#!/usr/bin/env python3
import time

import message_filters
import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Float32MultiArray, Float64MultiArray, UInt8

from probstar_reach.msg import ProbStar as ProbStarMsg
from probstar_reach.utils import (
    advance_probstar, build_probstar, collision_constraint, collision_probability,
    rot_to_euler, linear_system_matrices)


def step_time(t0, dt, k):
    """Time of tube step k. Sender and receiver must both use it: the two pointers need equal step times."""
    return t0 + Duration(seconds=k * dt)


def quat_to_rot(x, y, z, w):
    """Rotation matrix (body -> parent frame) from a quaternion."""
    q = np.array([w, x, y, z], dtype=float)
    q /= np.linalg.norm(q)
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Verifier(Node):
    """
    Propagates a tube of nsteps ProbStars from the agent's state estimate, step k = 1..nsteps advanced from the initial
    state to t0 + k*dt, and publishes it as mean/std per step.

    After each own tube it checks the newest tube received from every other agent, step by
    step: the two stars are concatenated into X = [X_self; X_other], intersected with the collision set
    H = {X | [-C 0 C 0] X <= d} (C p <= d is the Minkowski difference of the two bounding boxes) and the probability
    of the intersection is the probability of collision, conditioned on the predicate boxes. The constant velocity
    model is exactly linear, so err is published as zeros and not used. A received tube is kept until a newer one
    replaces it or all its steps have passed.

    Time alignment (two pointers, k on the own tube, m on the received one, both with the same dt): received steps
    earlier than the first own step are skipped, then own step k is advanced by Δt = τ_m - τ_k >= 0 (the same for
    every pair) and compared with received step m, k and m moving together. The maximum of the pair probabilities
    is compared with prob_threshold, and the pairs stop at the first one reaching it.
    """

    def __init__(self):
        super().__init__('verifier')

        self.dt = self.declare_parameter('dt', 0.1).value
        self.nsteps = self.declare_parameter('nsteps', 10).value
        period = self.declare_parameter('period', 0.1).value
        if round(self.dt * 1e9) <= 0 or self.nsteps < 1:
            raise ValueError('dt must be at least 1 ns and nsteps must be positive')

        # Pitch/roll/yaw std (rad) used when the state/pose covariance has none
        self.attitude_std = self.declare_parameter('attitude_std', 0.02).value

        # Unique id of the agent, 0..255 (no sensible default, two agents with the same id ignore each other)
        self.sysid = self.declare_parameter('SystemID', 0).value
        if not 0 <= self.sysid <= 255:
            raise ValueError(f'SystemID must be a value between 0 and 255, got {self.sysid}')

        self.prob_threshold = self.declare_parameter('prob_threshold', 0.5).value
        # Half extents (x, y, z) of the bounding boxes. The msg carries no size, so the other agents get a fixed one
        half_extent_self = self.declare_parameter('half_extent_self', [0.5, 0.5, 0.5]).value
        half_extent_other = self.declare_parameter('half_extent_other', [0.5, 0.5, 0.5]).value
        self.H, self.d = collision_constraint(half_extent_self, half_extent_other)
        # Max difference (s) between the own and a received star's time for the two to be compared
        self.time_tolerance = self.declare_parameter('time_tolerance', 0.01).value

        # agent id -> (dt, [(step time, ProbStar)], decode time, t0) of the newest tube received from it, kept until a
        # newer one replaces it or it can no longer be checked (all its steps have passed or its dt differs)
        self.tubes = {}
        # [(step time, ProbStar)] of the newest own tube
        self.own_tube = None
        # P(G) of the joint star, the same for every pair (always 6 + 3 own and 6 received predicates in +-n_sigma)
        self.p_G = None

        self.x_mean = None
        self.x_std = None
        self.state_stamp = None
        self.rot = None
        self.u_std = None

        # State from px4_drone's state_estimate, in the earth frame common to all drones (like face_lifting), with
        # its covariance. Both topics are published from each PX4 odometry with the same stamp, so they are paired by
        # exact stamp.
        pose_sub = message_filters.Subscriber(
            self, PoseWithCovarianceStamped, 'state/pose', qos_profile=qos_profile_sensor_data)
        twist_sub = message_filters.Subscriber(
            self, TwistWithCovarianceStamped, 'state/twist', qos_profile=qos_profile_sensor_data)
        message_filters.TimeSynchronizer([pose_sub, twist_sub], 10).registerCallback(self.state_cb)
        # Latched in ROS 1 -> transient_local in ROS 2
        self.star_pub = self.create_publisher(
            ProbStarMsg, 'my_rs', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(ProbStarMsg, 'received_rs', self.rcvd_cb, 30)

        # px4_drone's random_walk aborts its target, retreats and holds position on this event
        self.collision_pub = self.create_publisher(UInt8, 'collision_detected', 10)
        # [agent id, max prob, min prob, latest, earliest checked time - now (s)], one per checked tube
        self.prob_pub = self.create_publisher(Float32MultiArray, 'collision_prob', 10)
        # Timing measurements (s). Publishing is never inside a timed block. The transfer time is br_time of
        # agent_comms. Float64 where t0 (s) identifies a tube
        # [agent id, decode time, steps], one per received tube (msg mean/std -> ProbStars)
        self.timing_decode_pub = self.create_publisher(Float32MultiArray, 'timing/decode', 10)
        # [t0, reach time, encode + publish time, whole compute_reach callback], one per own tube
        self.timing_cycle_pub = self.create_publisher(Float64MultiArray, 'timing/cycle', 10)
        # [agent id, its t0, check time (alignment setup + all pairs), pairs, max prob], one per checked tube
        self.timing_check_pub = self.create_publisher(Float64MultiArray, 'timing/check', 10)
        # [agent id, own step k, received step index, tau - now (s), alignment time, probability time, prob],
        # one per checked pair
        self.timing_pair_pub = self.create_publisher(Float32MultiArray, 'timing/pair', 10)
        self.create_timer(period, self.compute_reach)

    def state_cb(self, pose: PoseWithCovarianceStamped, twist: TwistWithCovarianceStamped):
        # Both in the earth frame: position, and linear velocity (not body), so no rotation is needed for x
        p = pose.pose.pose.position
        o = pose.pose.pose.orientation
        v = twist.twist.twist.linear
        # Attitude (body FLU -> earth), for the roll/pitch input u
        self.rot = quat_to_rot(o.x, o.y, o.z, o.w)

        self.x_mean = np.array([p.x, p.y, p.z, v.x, v.y, v.z])
        self.state_stamp = pose.header.stamp

        # Earth-frame position and velocity variances, the diagonals of the covariances
        pose_cov = np.array(pose.pose.covariance).reshape(6, 6)
        twist_cov = np.array(twist.twist.covariance).reshape(6, 6)
        self.x_std = np.sqrt(np.concatenate((
            np.diag(pose_cov)[:3], np.diag(twist_cov)[:3])))
        # Attitude variance, same order as u = [roll, pitch, yaw], falls back to attitude_std when unset
        att_var = np.diag(pose_cov)[3:6]
        self.u_std = np.where(att_var > 0, np.sqrt(np.maximum(att_var, 0)), self.attitude_std)

    def compute_reach(self):
        if self.x_mean is None:
            self.get_logger().warn(
                'Waiting for state/pose and state/twist', throttle_duration_sec=2.0)
            return

        start = time.perf_counter()
        state_age = (self.get_clock().now() - Time.from_msg(self.state_stamp)).nanoseconds / 1e9
        t0 = Time.from_msg(self.state_stamp)
        # x/y driven by the held roll/pitch (fixed altitude, linearized there), z constant velocity
        u_mean = rot_to_euler(self.rot)  # [roll, pitch, yaw], yaw is a placeholder (zero Bd column)
        X0 = build_probstar(self.x_mean, self.x_std)
        U = build_probstar(u_mean, self.u_std)
        # Each step advanced from X0, not from the previous step: chaining would add a new independent input per step
        tube = []
        for k in range(1, self.nsteps + 1):
            Ad, Bd, cd = linear_system_matrices(k * self.dt, u_mean)
            tube.append((step_time(t0, self.dt, k), advance_probstar(X0, U, Ad, Bd, cd)))
        # Constant velocity is exactly linear, no linearization remainder
        lin_err = np.zeros(6)
        reach_s = time.perf_counter() - start

        encode_start = time.perf_counter()
        out = ProbStarMsg()
        out.t0 = self.state_stamp
        out.dt = float(self.dt)
        out.id = self.sysid
        # The receiver rebuilds each advanced star with build_probstar(mean, std). Its generators are not diagonal
        # (x depends on alpha_x and alpha_vx), but each position axis uses its own predicates, so the row norms keep
        # the position variances the collision check needs and only drop the position/velocity correlation.
        out.mean = np.concatenate([X.V[:, 0] for _, X in tube]).tolist()
        out.std = np.concatenate([np.linalg.norm(X.V[:, 1:], axis=1) for _, X in tube]).tolist()
        out.err = np.tile(lin_err, self.nsteps).tolist()
        # Send time: agent_comms of the receivers measures the transfer time (br_time) from it
        out.header.stamp = self.get_clock().now().to_msg()
        self.star_pub.publish(out)
        encode_pub_s = time.perf_counter() - encode_start

        self.own_tube = tube
        results = self.check_tubes()
        total_s = time.perf_counter() - start
        self.timing_cycle_pub.publish(Float64MultiArray(data=[t0.nanoseconds / 1e9, reach_s, encode_pub_s, total_s]))
        # Measurement: reach, encode + publish and whole timer callback against the period, one entry per
        # kept tube
        self.get_logger().debug(
            f't+{self.nsteps * self.dt:.3f}s roll/pitch/yaw={np.round(u_mean, 3).tolist()} '
            f'center={np.round(tube[-1][1].V[:, 0], 3).tolist()} '
            f'lin_err={np.round(lin_err, 5).tolist()} | reach {reach_s * 1e3:.3f}ms, '
            f'encode+pub {encode_pub_s * 1e3:.3f}ms, total {total_s * 1e3:.3f}ms, state age {state_age:.3f}s | '
            f'{len(results)} tubes' + ''.join(f'; {r}' for r in results))

    def rcvd_cb(self, msg: ProbStarMsg):
        if msg.id == self.sysid:
            return
        decode_start = time.perf_counter()
        t0 = Time.from_msg(msg.t0)
        mean, std = np.reshape(msg.mean, (-1, 6)), np.reshape(msg.std, (-1, 6))
        tube = [(step_time(t0, msg.dt, k), build_probstar(m, s))
                for k, (m, s) in enumerate(zip(mean, std), start=1)]
        decode_s = time.perf_counter() - decode_start
        self.tubes[msg.id] = (msg.dt, tube, decode_s, t0)
        self.timing_decode_pub.publish(Float32MultiArray(data=[float(msg.id), decode_s, float(len(tube))]))

    def check_tubes(self):
        """Check the received tubes against the own tube (two pointers), returns one summary per tube for the log."""
        results = []
        own_tube = self.own_tube
        # A copy: tubes are deleted while iterating
        for other_id, (other_dt, other_tube, decode_s, other_t0) in list(self.tubes.items()):
            if other_dt != self.dt:
                # Never comparable with the own tube, dropped like an expired one
                del self.tubes[other_id]
                results.append(f'agent {other_id}: dt={other_dt} differs from the own {self.dt}, dropped')
                continue

            # Time alignment setup shared by every pair: pointer skip and Ad(Δt)
            setup_start = time.perf_counter()
            # Skip the received steps earlier than the first own step (state + dt, so while the state age is below dt
            # this also skips the ones already passed)
            m = 0
            while m < len(other_tube) and other_tube[m][0] < own_tube[0][0]:
                m += 1
            if m == len(other_tube):
                # Every step has passed, nothing left to check until a newer tube arrives
                del self.tubes[other_id]
                results.append(f'agent {other_id}: older than the own tube, dropped')
                continue
            # Own step k is advanced to received step m. Same dt, so Δt is the same for every pair. Exact for the
            # constant velocity model, Ad(Δt) Ad(k dt) = Ad(k dt + Δt)
            advance = (other_tube[m][0] - own_tube[0][0]).nanoseconds / 1e9
            Ad, _, cd = linear_system_matrices(advance)
            setup_s = time.perf_counter() - setup_start

            probs, taus, align_times, prob_times = [], [], [], []
            try:
                if self.p_G is None:
                    # Once per node lifetime, kept out of the check time. It depends only on the predicates, which an
                    # affine map keeps, so the unaligned pair gives the same value
                    t_p_G = time.perf_counter()
                    self.p_G = float(own_tube[0][1].concatenate(other_tube[m][1]).estimateProbability())
                    self.get_logger().info(f'P(G) = {self.p_G:.6f} estimated in '
                                           f'{(time.perf_counter() - t_p_G) * 1e3:.3f}ms')
                estimate_start = time.perf_counter()
                for (_, own_star), (tau, other_star) in zip(own_tube, other_tube[m:]):
                    t_align = time.perf_counter()
                    own_star = own_star.affineMap(Ad, cd)
                    t_prob = time.perf_counter()
                    align_times.append(t_prob - t_align)
                    probs.append(collision_probability(own_star, other_star, self.H, self.d, self.p_G))
                    prob_times.append(time.perf_counter() - t_prob)
                    taus.append(tau)
                    # Early stop: one pair over the threshold already decides the collision
                    if probs[-1] >= self.prob_threshold:
                        break
                estimate_s = time.perf_counter() - estimate_start
            except Exception as e:
                # Empty intersections already give 0, so this is a failed estimate
                self.get_logger().error(f'agent {other_id}: collision probability estimate failed: {e!r}')
                results.append(f'agent {other_id}: estimate failed')
                continue

            # Upper/lower bound over the pairs, and the time range they cover (taus is increasing). After an early
            # stop these cover only the pairs checked so far
            p_max, p_min = max(probs), min(probs)
            now = self.get_clock().now()
            tau_max = (taus[-1] - now).nanoseconds / 1e9
            tau_min = (taus[0] - now).nanoseconds / 1e9
            # Measurement
            results.append(
                f'agent {other_id}: skipped {m} paired {len(probs)} advance={advance:.3f}s '
                f'tau=[{tau_min:+.3f}, {tau_max:+.3f}]s estimate={estimate_s:.3f}s '
                f'prob=[{p_min:.4f}, {p_max:.4f}] | decode={decode_s * 1e3:.3f}ms setup={setup_s * 1e3:.3f}ms '
                f'align={[round(t * 1e3, 3) for t in align_times]}ms '
                f'prob_time={[round(t * 1e3, 3) for t in prob_times]}ms')
            self.prob_pub.publish(Float32MultiArray(data=[float(other_id), p_max, p_min, tau_max, tau_min]))
            self.timing_check_pub.publish(Float64MultiArray(data=[
                float(other_id), other_t0.nanoseconds / 1e9, setup_s + estimate_s, float(len(probs)), p_max]))
            # One timing message per pair, k is the own step (1-based), m + k the received one
            for k, (tau, align_s, prob_s, prob) in enumerate(zip(taus, align_times, prob_times, probs), start=1):
                self.timing_pair_pub.publish(Float32MultiArray(data=[
                    float(other_id), float(k), float(m + k), (tau - now).nanoseconds / 1e9, align_s, prob_s, prob]))

            if p_max >= self.prob_threshold:
                self.get_logger().warn(f'COLLISION DETECTED with agent {other_id}, prob={p_max:.4f}')
                self.collision_pub.publish(UInt8(data=other_id))
        return results


def main(args=None):
    rclpy.init(args=args)
    node = Verifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
