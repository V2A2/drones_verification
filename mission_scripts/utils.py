"""Unified interface for the vehicle backends used by the mission scripts.

Each concrete class owns its own ROS2 node lifecycle (rclpy.init/shutdown),
so a Vehicle is self-contained and can be used as a context manager:

    with Drone(namespace='drone0') as uav:
        uav.arm()
        uav.takeoff(30.0)
        uav.goto_gps(lat, lon, 30.0)
        uav.land()

Note: rclpy.init()/rclpy.shutdown() are process-global. If a script creates
more than one Vehicle, only the first constructor call actually initializes
rclpy, and shutdown() on any one instance tears it down for all of them.
"""

from abc import ABC, abstractmethod
import math
import random
from enum import Enum, auto
import rclpy

DRONE_TAKE_OFF_SPEED = 1.0  # m/s
DRONE_SPEED = 1.0  # m/s
DRONE_LAND_SPEED = 0.5  # m/s
ARM_SERVICE_TIMEOUT = 3.0  # s, wait for the platform's set_arming_state service

class GotoResult(Enum):
    """Outcome of a collision-avoiding goto."""
    SUCCESS = auto()
    ABORTED = auto()   # accepted, then aborted by the behavior (e.g. collision) -> drone hovers
    REJECTED = auto()  # goal never accepted


def _make_collision_avoidance_handler(drone):
    """
    Create a handler for the verifier's GoToCollisionAvoidanceBehavior.

    as2_python_api's GoToBehavior hardcodes the action name 'GoToBehavior', so it can't be
    pointed at the new behavior; this reuses its BehaviorHandler base instead. The action
    type is the same GoToWaypoint, so the goal is built like GoToBehavior.start() does.
    """
    from as2_msgs.action import GoToWaypoint
    from as2_msgs.msg import YawMode
    from as2_python_api.behavior_actions.behavior_handler import BehaviorHandler
    from pymap3d import geodetic2enu

    class GoToCollisionAvoidance(BehaviorHandler):
        def __init__(self):
            super().__init__(drone, GoToWaypoint, 'GoToCollisionAvoidanceBehavior')

        def go_to_gps(self, lat, lon, alt, speed) -> GotoResult:
            lat0, lon0, h0 = drone.gps.origin
            x, y, _ = geodetic2enu(lat, lon, alt, lat0, lon0, h0)
            return self.go_to_point(x, y, alt, speed)

        def go_to_point(self, x, y, z, speed) -> GotoResult:
            """Go to (x, y, z) in the earth frame (ENU, origin = the shared GPS origin)."""
            goal = GoToWaypoint.Goal()
            goal.target_pose.header.stamp = drone.get_clock().now().to_msg()
            goal.target_pose.header.frame_id = 'earth'
            goal.target_pose.point.x = float(x)
            goal.target_pose.point.y = float(y)
            goal.target_pose.point.z = float(z)  # height from origin, as in GoToBehavior
            goal.max_speed = float(speed)
            goal.yaw.mode = YawMode.PATH_FACING
            try:
                ok = self.start(goal, wait_result=True)
            except self.GoalRejected as err:
                drone.get_logger().warn(str(err))
                return GotoResult.REJECTED
            return GotoResult.SUCCESS if ok else GotoResult.ABORTED

    return GoToCollisionAvoidance()


class Vehicle(ABC):
    class State(Enum):
        INIT = auto()
        ARMED = auto()
        FLYING = auto()

    """Common interface for all vehicle backends (drone, rover, fixed-wing)."""
    def __init__(self):
        pass

    @abstractmethod
    def arm(self) -> bool:
        """Arm the vehicle (and any backend-specific pre-flight step). Blocking."""

    @abstractmethod
    def takeoff(self, altitude: float, speed: float = None) -> bool:
        """Take off to `altitude` (m). Blocking. speed=None uses the backend default."""

    @abstractmethod
    def goto_gps(self, lat: float, lon: float, alt: float, speed: float = None) -> bool:
        """Go to a GPS waypoint (deg, deg, m). Blocking. speed=None uses the backend default."""

    @abstractmethod
    def land(self, speed: float = None) -> bool:
        """Land the vehicle. Blocking. speed=None uses the backend default."""

    @property
    @abstractmethod
    def gps(self) -> tuple:
        """Current (lat, lon, alt) position, in degrees/degrees/metres."""

    @property
    @abstractmethod
    def state(self) -> State:
        """return the current state"""

    @abstractmethod
    def shutdown(self) -> None:
        """Release the ROS2 resources owned by this vehicle."""

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.shutdown()
        return False


class Drone(Vehicle):
    """Wraps as2_python_api's DroneInterfaceGPS (aerostack2)."""

    def __init__(self, namespace: str = 'drone0', verbose: bool = False,
                 use_sim_time: bool = False):
        from as2_msgs.srv import SetPlatformStateMachineEvent
        from as2_python_api.drone_interface_gps import DroneInterfaceGPS
        from rclpy.signals import SignalHandlerOptions

        if not rclpy.ok():
            # Don't let rclpy's own SIGINT handler tear down the context on
            # Ctrl+C -- mission scripts catch KeyboardInterrupt themselves to
            # hold() before exiting, which needs the context to still be
            # alive when that handler runs.
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self._drone = DroneInterfaceGPS(
            drone_id=namespace, verbose=verbose, use_sim_time=use_sim_time)
        # Needed for hold(): not loaded by DroneInterface by default.
        self._drone.load_module('motion_reference_handler')
        # Used to unstick the platform's own state machine (see takeoff()) --
        # not wrapped by as2_python_api.
        self._state_machine_event_client = self._drone.create_client(
            SetPlatformStateMachineEvent, 'platform/state_machine_event')
        # verifier's GoTo with collision avoidance (needs verifier.launch.py running)
        try:
            self._go_to_collision_avoidance = _make_collision_avoidance_handler(self._drone)
        except Exception as err:
            self._drone.shutdown()
            raise RuntimeError(
                f'GoToCollisionAvoidanceBehavior not available in namespace {namespace!r}: '
                'is verifier.launch.py running for this drone?') from err
        print(f"drone state {self._drone.info['state']}")

    def arm(self) -> bool:
        # as2_python_api's Arm only logs when set_arming_state is missing and then blocks
        # forever in call(), so check the service ourselves and fail instead.
        from std_srvs.srv import SetBool

        client = self._drone.create_client(SetBool, 'set_arming_state')
        available = client.wait_for_service(timeout_sec=ARM_SERVICE_TIMEOUT)
        self._drone.destroy_client(client)
        if not available:
            print(f'set_arming_state not available after {ARM_SERVICE_TIMEOUT} s')
            return False
        # GPS-guided go_to requires offboard mode, so treat it as part of "armed".
        # Only the arm call itself is skippable when already armed/flying --
        # offboard still needs to be (re-)checked every time.
        if self.state == Vehicle.State.INIT and not self._drone.arm():
            return False
        if self._drone.info['offboard']:
            return True
        return self._drone.offboard()

    def takeoff(self, altitude: float, speed: float = None) -> bool:
        from as2_msgs.msg import PlatformStateMachineEvent, PlatformStatus
        from as2_msgs.srv import SetPlatformStateMachineEvent

        if self._drone.info['state'] == PlatformStatus.TAKING_OFF:
            # The platform's own state machine (as2_core/platform_state_
            # machine.cpp) only leaves TAKING_OFF on an explicit TOOK_OFF
            # event, normally fired by TakeoffBehavior itself on real
            # completion. If a previous takeoff attempt never finished
            # formally (e.g. a goal rejected before this vehicle was in
            # offboard mode), that event never fires and the platform is
            # stuck believing a takeoff is still in progress -- which then
            # gets goto/land goals rejected too, even though the vehicle is
            # actually flying fine. Fire the missing event directly instead
            # of re-sending a takeoff the platform would reject anyway
            # (it's not in LANDED any more).
            request = SetPlatformStateMachineEvent.Request()
            request.event.event = PlatformStateMachineEvent.TOOK_OFF
            self._state_machine_event_client.call(request, timeout_sec=2.0)
            return True
        if self.state == Vehicle.State.FLYING:
            return True
        return self._drone.takeoff(height=altitude, speed=speed or DRONE_TAKE_OFF_SPEED)

    def goto_gps(self, lat: float, lon: float, alt: float, speed: float = None, avoid_collision: bool = True) -> GotoResult:
        if not avoid_collision:
            handler = self._drone.go_to
            ok = handler.go_to_gps_point_path_facing(
                [lat, lon, alt], speed=speed or DRONE_SPEED)
            result = GotoResult.SUCCESS if ok else GotoResult.ABORTED
        else:
            handler = self._go_to_collision_avoidance
            result = handler.go_to_gps(lat, lon, alt, speed or DRONE_SPEED)
        return result

    def goto_earth(self, x: float, y: float, z: float, speed: float = None) -> GotoResult:
        """Collision-avoiding goto to (x, y, z) in the earth frame (m)."""
        return self._go_to_collision_avoidance.go_to_point(x, y, z, speed or DRONE_SPEED)


    def hold(self) -> bool:
        """Cancel whatever motion behavior is active and hover in place."""
        # Neither go_to plugin's own_deactivate (the path stop() below
        # triggers) actually sends a hover setpoint -- it's a no-op in the
        # trajectory plugin and just logs in the position plugin -- so
        # without this the drone keeps coasting on its last velocity command.
        # Send it first since it's what actually matters for safety.
        try:
            self._drone.motion_ref_handler.hover()
        except KeyboardInterrupt:
            # hover() blocks up to 5s polling for the flight controller to
            # confirm the mode switch (basic_motion_references.__set_mode).
            # The request itself is sent synchronously before that wait, so
            # a Ctrl+C here only cuts short the confirmation -- still fall
            # through to cancel the GoTo action below instead of leaving it
            # running.
            print('Interrupted while confirming hover mode; command was already sent')
        return True

    def land(self, speed: float = None) -> bool:
        return self._drone.land(speed=speed or DRONE_LAND_SPEED)

    @property
    def gps(self) -> tuple:
        lat, lon, alt = self._drone.gps.pose
        return lat, lon, alt

    @property
    def position(self) -> tuple:
        """(x, y, z) in the earth frame (m), from self_localization/pose."""
        x, y, z = self._drone.position
        return x, y, z

    @property
    def state(self) -> Vehicle.State:
        from as2_msgs.msg import PlatformStatus

        platform_state = self._drone.info['state']
        if not self._drone.info['armed']:
            return Vehicle.State.INIT
        if platform_state == PlatformStatus.LANDED:
            return Vehicle.State.ARMED
        # TAKING_OFF, FLYING, LANDING, and EMERGENCY (which typically occurs
        # mid-flight) are all treated as FLYING.
        return Vehicle.State.FLYING

    def shutdown(self) -> None:
        self._drone.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


class Rover(Vehicle):
    """Wraps nav2_simple_commander's BasicNavigator, driven via GPS goals."""

    def __init__(self, namespace: str = ''):
        from nav2_simple_commander.robot_navigator import BasicNavigator
        from swri_transform_util.wgs84_transformer import Wgs84Transformer

        from nav2_gps_utils import (
            GOAL_TIMEOUT, get_gps_fix, interpolate_steps, make_origin, send_goal)
        from rclpy.signals import SignalHandlerOptions

        if not rclpy.ok():
            # See Drone.__init__: don't let rclpy tear down the context on
            # Ctrl+C before a script's own KeyboardInterrupt handling runs.
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self._navigator = BasicNavigator(namespace=namespace)
        self._navigator.waitUntilNav2Active(localizer='robot_localization')
        self._map_frame = f'{namespace}/map'
        self._get_gps_fix = get_gps_fix
        self._send_goal = send_goal
        self._interpolate_steps = interpolate_steps
        self._goal_timeout = GOAL_TIMEOUT
        # Origin is fixed once at startup; rebuilding it per-call would shift
        # the local coordinate frame under every subsequent goal.
        origin_fix = get_gps_fix(self._navigator)
        self._transformer = Wgs84Transformer(make_origin(origin_fix))
        print(f'GPS origin set to lat={origin_fix.latitude}, lon={origin_fix.longitude}, '
              f'alt={origin_fix.altitude}')
        # Local-frame position, tracked so goto_gps can interpolate sub-goals
        # from wherever the rover last headed rather than from the origin.
        self._local_pos = (0.0, 0.0)

    def arm(self) -> bool:
        """No-op: ground rovers have no arm concept."""
        return True

    def takeoff(self, altitude: float, speed: float = None) -> bool:
        """No-op: ground rovers do not take off."""
        return True

    def goto_gps(self, lat: float, lon: float, alt: float = None, speed: float = None,
                 max_step_m: float = None) -> bool:
        """
        Go to a GPS waypoint. alt/speed unused: Nav2's goToPose has neither a
        z nor a speed param. If max_step_m is given, the hop from the current
        local position is split into sub-goals no longer than max_step_m,
        matching how a long jump between two arbitrary waypoints should be
        driven through Nav2 rather than sent as a single distant goal.
        """
        (x, y), = self._transformer.wgs84_to_local_xy([[lat, lon]])
        target = (x, y)

        if max_step_m is None:
            success = self._send_goal(self._navigator, x, y, self._map_frame, self._goal_timeout)
            self._local_pos = target
            return success

        sub_goals = self._interpolate_steps(self._local_pos, target, max_step_m)
        success = True
        for leg, (goal_x, goal_y) in enumerate(sub_goals):
            print(f'  Leg {leg + 1}/{len(sub_goals)}: Go to local ({goal_x:.2f}, {goal_y:.2f})')
            success = self._send_goal(
                self._navigator, goal_x, goal_y, self._map_frame, self._goal_timeout)
            print(f'  Goal success: {success}')
            if not success:
                break
        self._local_pos = target
        return success

    def land(self, speed: float = None) -> bool:
        """No-op: ground rovers do not land."""
        return True

    @property
    def gps(self) -> tuple:
        fix = self._get_gps_fix(self._navigator)
        return fix.latitude, fix.longitude, fix.altitude

    @property
    def state(self) -> Vehicle.State:
        """Ground rovers have no arm/fly distinction: always ready."""
        return Vehicle.State.ARMED

    def shutdown(self) -> None:
        self._navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


class _FixedWingNode:
    """Thin ROS2 client for px4_fixedwing_control's vehicle_server node.

    Exposes only the raw arm/takeoff/goto_waypoint/position plumbing, without
    any mission-specific parameters or run loop.
    """

    def __init__(self, node, poll_interval: float = 0.05):
        from px4_fixedwing_control.action import GotoWaypoint
        from px4_fixedwing_control.srv import Takeoff
        from px4_msgs.msg import VehicleGlobalPosition, VehicleLandDetected, VehicleStatus
        from rclpy.action import ActionClient
        from rclpy.qos import qos_profile_sensor_data
        from std_srvs.srv import SetBool

        self._node = node
        self._poll_interval = poll_interval
        self._set_bool_request = SetBool.Request
        self._takeoff_request = Takeoff.Request
        self._goto_waypoint_goal = GotoWaypoint.Goal
        self._vehicle_status_cls = VehicleStatus

        self._vehicle_global_position = None
        node.create_subscription(
            VehicleGlobalPosition, 'fmu/out/vehicle_global_position',
            self._global_position_callback, qos_profile_sensor_data)

        self._vehicle_status = None
        node.create_subscription(
            VehicleStatus, 'fmu/out/vehicle_status_v1',
            self._vehicle_status_callback, qos_profile_sensor_data)

        self._vehicle_land_detected = None
        node.create_subscription(
            VehicleLandDetected, 'fmu/out/vehicle_land_detected',
            self._vehicle_land_detected_callback, qos_profile_sensor_data)

        self._arm_client = node.create_client(SetBool, 'arm')
        self._takeoff_client = node.create_client(Takeoff, 'takeoff')
        self._goto_waypoint_client = ActionClient(node, GotoWaypoint, 'goto_waypoint')

    def _global_position_callback(self, msg) -> None:
        self._vehicle_global_position = msg

    def _vehicle_status_callback(self, msg) -> None:
        self._vehicle_status = msg

    def _vehicle_land_detected_callback(self, msg) -> None:
        self._vehicle_land_detected = msg

    @property
    def state(self) -> Vehicle.State:
        # Messages not received yet: can't confirm armed, so stay INIT
        # (caller won't skip arm()).
        if self._vehicle_status is None:
            return Vehicle.State.INIT
        if self._vehicle_status.arming_state != self._vehicle_status_cls.ARMING_STATE_ARMED:
            return Vehicle.State.INIT
        if self._vehicle_land_detected is None or not self._vehicle_land_detected.landed:
            return Vehicle.State.FLYING
        return Vehicle.State.ARMED

    def _wait_future(self, future):
        import time
        while rclpy.ok() and not future.done():
            time.sleep(self._poll_interval)
        return future.result()

    def wait_for_global_position(self):
        import time
        while self._vehicle_global_position is None and rclpy.ok():
            time.sleep(0.1)
        return self._vehicle_global_position

    def arm(self, value: bool) -> bool:
        self._arm_client.wait_for_service()
        request = self._set_bool_request(data=value)
        response = self._wait_future(self._arm_client.call_async(request))
        return response.success

    def takeoff(self, altitude: float) -> bool:
        self._takeoff_client.wait_for_service()
        request = self._takeoff_request(altitude=altitude)
        response = self._wait_future(self._takeoff_client.call_async(request))
        return response.success

    def goto_waypoint(self, latitude: float, longitude: float, altitude: float) -> bool:
        self._goto_waypoint_client.wait_for_server()
        goal = self._goto_waypoint_goal(latitude=latitude, longitude=longitude, altitude=altitude)

        def feedback_callback(feedback_msg):
            self._node.get_logger().info(
                f'  distance remaining: {feedback_msg.feedback.distance_remaining:.2f} m')

        goal_handle = self._wait_future(
            self._goto_waypoint_client.send_goal_async(goal, feedback_callback=feedback_callback))
        if not goal_handle.accepted:
            return False
        result = self._wait_future(goal_handle.get_result_async())
        return result.result.success


class FixedWing(Vehicle):
    """Wraps a ROS2 client for px4_fixedwing_control's vehicle_server node."""

    def __init__(self, namespace: str = '', node_name: str = 'fixedwing_vehicle'):
        import threading

        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from rclpy.signals import SignalHandlerOptions

        if not rclpy.ok():
            # See Drone.__init__: don't let rclpy tear down the context on
            # Ctrl+C before a script's own KeyboardInterrupt handling runs.
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self._raw_node = Node(node_name, namespace=namespace or None)
        self._node = _FixedWingNode(self._raw_node)

        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._raw_node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

    def arm(self) -> bool:
        if self.state != Vehicle.State.INIT:
            return True
        return self._node.arm(True)

    def takeoff(self, altitude: float, speed: float = None) -> bool:
        # speed unused: Takeoff.srv has no speed field, the server controls climb rate.
        if self.state == Vehicle.State.FLYING:
            return True
        return self._node.takeoff(altitude)

    def goto_gps(self, lat: float, lon: float, alt: float, speed: float = None) -> bool:
        # speed unused: GotoWaypoint.action has no speed field.
        return self._node.goto_waypoint(lat, lon, alt)

    @property
    def state(self) -> Vehicle.State:
        return self._node.state

    def land(self, speed: float = None) -> bool:
        """Not supported: vehicle_server exposes no landing action/service."""
        raise NotImplementedError(
            'FixedWing.land is not supported: vehicle_server has no landing capability')

    @property
    def gps(self) -> tuple:
        msg = self._node.wait_for_global_position()
        return msg.lat, msg.lon, msg.alt

    def shutdown(self) -> None:
        self._executor.shutdown()
        self._raw_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

class Executor(ABC):
    """Generates a sequence of GPS waypoints, decoupled from the Vehicle
    that flies/drives to them."""

    @abstractmethod
    def next_gps(self) -> tuple:
        """Return the next (lat, lon, alt) waypoint, or raise StopIteration
        when the sequence is exhausted."""

    def __iter__(self):
        return self

    def __next__(self) -> tuple:
        return self.next_gps()


class RandomWalk:
    """Flies straight segments of a fixed length and picks a new heading only at the end of a segment
    (within +-turn_deg of the current one). Yields (x, y, altitude) targets in the earth frame that stay
    inside area = (x_min, x_max, y_min, y_max): if the next segment would leave the area, a new heading is
    drawn at random until it doesn't. segment_m must be shorter than half the area's diagonal, or no heading
    from the area's center stays inside. Each target starts from the previous target, not the measured position,
    so the targets depend only on the seed and the start, rounded to whole meters like the drones.mod spawns."""

    def __init__(self, drone: Drone, num_steps: int, segment_m: float, area: tuple,
                 turn_deg: float, altitude: float):
        x, y, _ = drone.position
        self._x, self._y = round(x), round(y)
        self._num_steps = num_steps
        self._segment_m = segment_m
        self._area = area
        self._turn = math.radians(turn_deg)
        self._altitude = altitude
        self._step = 0
        self._heading = random.uniform(0.0, 2.0 * math.pi)  # rad from north (y), towards east (x)

    def __iter__(self):
        return self

    def __next__(self) -> tuple:
        if self._num_steps is not None and self._step >= self._num_steps:
            raise StopIteration
        self._step += 1

        x_min, x_max, y_min, y_max = self._area
        self._heading += random.uniform(-self._turn, self._turn)
        while True:
            end_x = self._x + self._segment_m * math.sin(self._heading)
            end_y = self._y + self._segment_m * math.cos(self._heading)
            if x_min <= end_x <= x_max and y_min <= end_y <= y_max:
                self._x, self._y = end_x, end_y
                return end_x, end_y, self._altitude
            self._heading = random.uniform(0.0, 2.0 * math.pi)


class Mission(Executor):
    """Visits a static, pre-extracted list of GPS waypoints in order."""

    def __init__(self, waypoints: list, default_altitude: float = None):
        self._waypoints = waypoints
        self._default_altitude = default_altitude
        self._index = 0

    def next_gps(self) -> tuple:
        if self._index >= len(self._waypoints):
            raise StopIteration
        waypoint = self._waypoints[self._index]
        self._index += 1

        if len(waypoint) == 2:
            lat, lon = waypoint
            alt = self._default_altitude
        else:
            lat, lon, alt = waypoint
        return lat, lon, alt
