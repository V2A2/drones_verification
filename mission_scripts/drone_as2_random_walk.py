#!/usr/bin/env python3

import argparse
import math
import random
from time import sleep

from as2_msgs.msg import YawMode
from geometry_msgs.msg import Pose

from utils import Drone, GotoResult, RandomWalk

TAKE_OFF_SPEED = 1.0  # Max speed in m/s
SLEEP_TIME = 0.5  # Sleep time between behaviors in seconds
COLLISION_WAIT = 3.0  # Hover time after a collision abort before the next step, seconds
RETREAT_M = 1.0  # Distance flown back, away from the target, after a collision abort, meters
SPEED = 2.0  # Max speed in m/s
HEIGHT = 30.0  # Height in meters
# LAND_SPEED = 0.5  # Max speed in m/s
NUM_STEPS = None  # Number of random walk steps
SEGMENT_M = 10.0  # Length of each straight segment before the heading changes, meters (< half the AREA diagonal)
AREA = (-30.0, 5.0, 0.0, 30.0)  # x_min, x_max, y_min, y_max in the earth frame (m), covers the drones.mod spawns
TURN_DEG = 90.0  # Max heading change at the end of a segment, degrees


def drone_start(drone: Drone) -> bool:
    """
    Take off the drone.

    :param drone: Drone object
    :return: Bool indicating if the take off was successful
    """
    print('Start mission')

    # Arm (also switches to offboard mode, required for GPS-guided flight)
    print('Arm')
    success = drone.arm()
    print(f'Arm success: {success}')
    if not success:
        return False

    # Take Off
    print('Take Off')
    success = drone.takeoff(HEIGHT, speed=TAKE_OFF_SPEED)
    print(f'Take Off success: {success}')

    return success


def drone_run(drone: Drone, num_steps: int, segment_m: float, area: tuple, turn_deg: float,
              height: float, speed: float, sleep_time: float) -> bool:
    """
    Run the random walk mission for a single drone.

    Each step flies to a target segment_m meters from the previous target, then changes
    heading by at most turn_deg. If the next segment would leave area, the heading is
    drawn at random instead.

    :param drone: Drone object
    :param num_steps: number of random walk steps
    :param segment_m: length of each straight segment (m)
    :param area: (x_min, x_max, y_min, y_max) in the earth frame (m)
    :param turn_deg: max heading change per segment (deg)
    :param height: flight altitude (m)
    :param speed: max speed (m/s)
    :param sleep_time: sleep time between steps (s)
    :return: Bool indicating if the mission was successful
    """
    print('Run mission')

    success = True
    walk = RandomWalk(drone, num_steps, segment_m, area, turn_deg, altitude=height)
    for step, (x, y, z) in enumerate(walk):
        print(f'Step {step + 1}/{num_steps}: Go to [{x}, {y}, {z}]')
        result = drone.goto_earth(x, y, z, speed=speed)
        print(f'Go to result: {result.name}')
        if result == GotoResult.REJECTED:
            return False
        if result == GotoResult.ABORTED:
            #  Collision: retreat RETREAT_M from the current position along the line to the
            #  target, away from it. Uses the stock GoTo: the collision-avoiding one would be
            #  aborted at once by the alerts that keep coming while the drones are still close.
            ax, ay, az = drone.position
            dist = math.dist((ax, ay, az), (x, y, z))
            if dist >= 0.2:  # Closer than the behavior's go_to_threshold there is no direction
                k = RETREAT_M / dist
                px, py, pz = ax - k * (x - ax), ay - k * (y - ay), az - k * (z - az)
                print(f'Collision, retreating to [{px:.2f}, {py:.2f}, {pz:.2f}]')
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = px, py, pz
                drone._drone.go_to.start(
                    pose=pose, speed=speed, yaw_mode=YawMode.KEEP_YAW, yaw_angle=None)
            # The behavior's own hover is fire-and-forget (its result is ignored),
            # so hover explicitly and confirm it before waiting, then pick a new step. The wait
            # is fixed, so it draws nothing from the seeded random walk.
            drone.hold()
            print(f'Collision, holding for {COLLISION_WAIT:.1f} s')
            sleep(COLLISION_WAIT)
            continue
        print('Go to done')
        sleep(sleep_time)

    return success


def drone_end(drone: Drone) -> bool:
    """
    End the mission for a single drone.

    :param drone: Drone object
    :return: Bool indicating if the land was successful
    """
    print('End mission')

    # Land
    print('Land')
    success = drone.hold()
    print(f'Land success: {success}')

    return success


def main(args=None):
    """Run the random walk mission node."""
    parser = argparse.ArgumentParser(
        description='Single drone random walk mission')

    parser.add_argument('-n', '--namespace',
                        type=str,
                        default='',
                        help='ID of the drone to be used in the mission')
    parser.add_argument('-v', '--verbose',
                        action='store_true',
                        default=False,
                        help='Enable verbose output')
    parser.add_argument('-s', '--use_sim_time',
                        action='store_true',
                        default=False,
                        help='Use simulation time')
    parser.add_argument('--steps',
                        type=int,
                        default=NUM_STEPS,
                        help='Number of random walk steps')
    parser.add_argument('--segment',
                        type=float,
                        default=SEGMENT_M,
                        help='Length of each straight segment (m)')
    parser.add_argument('--turn',
                        type=float,
                        default=TURN_DEG,
                        help='Max heading change at the end of a segment (deg)')
    parser.add_argument('--seed',
                        type=int,
                        default=None,
                        help='Random seed for reproducible runs')

    parsed_args = parser.parse_args(args=args)
    drone_namespace = parsed_args.namespace
    verbosity = parsed_args.verbose
    use_sim_time = parsed_args.use_sim_time

    if parsed_args.seed is not None:
        random.seed(parsed_args.seed)

    num_steps = parsed_args.steps if parsed_args.steps else None

    print(f'Running random walk mission for drone {drone_namespace}')

    with Drone(namespace=drone_namespace, verbose=verbosity, use_sim_time=use_sim_time) as uav:
        try:
            success = drone_start(uav)
            if success:
                success = drone_run(
                    uav, num_steps, parsed_args.segment, AREA, parsed_args.turn,
                    HEIGHT, SPEED, SLEEP_TIME)
            success = drone_end(uav)
        except KeyboardInterrupt:
            print('Interrupted, holding position')
            uav.hold()

    print('Clean exit')
    exit(0)


if __name__ == '__main__':
    main()
