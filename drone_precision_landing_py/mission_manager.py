#!/usr/bin/env python3

import math
from enum import Enum

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Float32


class MissionState(Enum):
    WAIT_FOR_POSITION = 0
    TAKEOFF = 1
    MISSION = 2
    LANDING_SEARCH = 3
    LAND = 4
    DONE = 5


class MissionManagerNode(Node):
    def __init__(self) -> None:
        super().__init__('mission_manager_node')

        # -------------------------------------------------
        # Parameters
        # -------------------------------------------------
        self.declare_parameter('takeoff_altitude', -5.0)
        self.declare_parameter('position_tolerance', 0.5)
        self.declare_parameter('hold_count_required', 10)
        self.declare_parameter('landing_search_hold_count', 20)
        self.declare_parameter(
            'waypoints',
            [0.0, 0.0, -5.0,
             5.0, 0.0, -5.0,
             5.0, 5.0, -5.0,
             0.0, 5.0, -5.0]
        )
        self.declare_parameter('landing_staging_x', 0.0)
        self.declare_parameter('landing_staging_y', 0.0)
        self.declare_parameter('landing_staging_z', -5.0)
        self.declare_parameter('landing_staging_yaw', 0.0)
        self.declare_parameter('skip_waypoints_and_land', False)

        # -------------------------------------------------
        # Read parameters
        # -------------------------------------------------
        self.takeoff_altitude = float(self.get_parameter('takeoff_altitude').value)
        self.position_tolerance = float(self.get_parameter('position_tolerance').value)
        self.hold_count_required = int(self.get_parameter('hold_count_required').value)
        self.landing_search_hold_count = int(self.get_parameter('landing_search_hold_count').value)

        waypoint_list = self.get_parameter('waypoints').value
        self.waypoints = self.parse_waypoints(waypoint_list)

        self.landing_staging_point = Point()
        self.landing_staging_point.x = float(self.get_parameter('landing_staging_x').value)
        self.landing_staging_point.y = float(self.get_parameter('landing_staging_y').value)
        self.landing_staging_point.z = float(self.get_parameter('landing_staging_z').value)

        self.landing_staging_yaw = float(self.get_parameter('landing_staging_yaw').value)
        self.skip_waypoints_and_land = bool(self.get_parameter('skip_waypoints_and_land').value)

        self.timer_period = 0.1  # 10 Hz

        # -------------------------------------------------
        # Publishers
        # -------------------------------------------------
        self.target_position_pub = self.create_publisher(
            Point,
            '/mission/target_position',
            10
        )

        self.target_yaw_pub = self.create_publisher(
            Float32,
            '/mission/target_yaw',
            10
        )

        self.land_pub = self.create_publisher(
            Bool,
            '/mission/land',
            10
        )

        self.landing_mode_pub = self.create_publisher(
            Bool,
            '/mission/landing_mode',
            10
        )

        # -------------------------------------------------
        # Subscribers
        # -------------------------------------------------
        self.current_position_sub = self.create_subscription(
            Point,
            '/mission/current_position',
            self.current_position_callback,
            10
        )

        # -------------------------------------------------
        # Internal state
        # -------------------------------------------------
        self.state = MissionState.WAIT_FOR_POSITION

        self.current_position = Point()
        self.have_position = False

        self.current_waypoint_index = 0
        self.goal_hold_counter = 0
        self.land_command_sent = False

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # -------------------------------------------------
        # Startup logs
        # -------------------------------------------------
        self.get_logger().info('Mission manager node started')
        self.get_logger().info(f'Initial state: {self.state.name}')
        self.get_logger().info(
            f'Parameters: takeoff_altitude={self.takeoff_altitude:.2f}, '
            f'position_tolerance={self.position_tolerance:.2f}, '
            f'hold_count_required={self.hold_count_required}, '
            f'landing_search_hold_count={self.landing_search_hold_count}'
        )
        self.get_logger().info(f'Mission waypoints loaded: {len(self.waypoints)} waypoint(s)')

        for i, waypoint in enumerate(self.waypoints, start=1):
            self.get_logger().info(
                f'  Waypoint {i}: ({waypoint.x:.2f}, {waypoint.y:.2f}, {waypoint.z:.2f})'
            )

        self.get_logger().info(
            f'Landing staging point: '
            f'({self.landing_staging_point.x:.2f}, '
            f'{self.landing_staging_point.y:.2f}, '
            f'{self.landing_staging_point.z:.2f}), '
            f'yaw={self.landing_staging_yaw:.2f}'
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def parse_waypoints(self, waypoint_list) -> list[Point]:
        waypoints = []

        if not isinstance(waypoint_list, list):
            raise ValueError('Parameter "waypoints" must be a list')

        if len(waypoint_list) % 3 != 0:
            raise ValueError(
                'Parameter "waypoints" must contain a multiple of 3 values: '
                '[x1, y1, z1, x2, y2, z2, ...]'
            )

        for i in range(0, len(waypoint_list), 3):
            waypoint = Point()
            waypoint.x = float(waypoint_list[i])
            waypoint.y = float(waypoint_list[i + 1])
            waypoint.z = float(waypoint_list[i + 2])
            waypoints.append(waypoint)

        return waypoints

    def current_position_callback(self, msg: Point) -> None:
        self.current_position = msg

        if not self.have_position:
            self.have_position = True
            self.get_logger().info(
                f'First position received: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}'
            )

    def timer_callback(self) -> None:
        self.publish_landing_mode(self.state == MissionState.LANDING_SEARCH)

        if self.state == MissionState.WAIT_FOR_POSITION:
            self.handle_wait_for_position()
        elif self.state == MissionState.TAKEOFF:
            self.handle_takeoff()
        elif self.state == MissionState.MISSION:
            self.handle_mission()
        elif self.state == MissionState.LANDING_SEARCH:
            self.handle_landing_search()
        elif self.state == MissionState.LAND:
            self.handle_land()
        elif self.state == MissionState.DONE:
            self.handle_done()

    # ------------------------------------------------------------------
    # State handlers
    # ------------------------------------------------------------------
    def handle_wait_for_position(self) -> None:
        if self.have_position:
            self.transition_to(MissionState.TAKEOFF)

    def handle_takeoff(self) -> None:
        target = Point()
        target.x = 0.0
        target.y = 0.0
        target.z = self.takeoff_altitude

        self.publish_target_position(target)
        self.publish_target_yaw(self.compute_yaw_to_target(target))

        if self.is_goal_reached(target):
            self.goal_hold_counter += 1
            if self.goal_hold_counter >= self.hold_count_required:
                self.get_logger().info('Takeoff target reached and stabilized')
                self.goal_hold_counter = 0

                if self.skip_waypoints_and_land or len(self.waypoints) == 0:
                    self.transition_to(MissionState.LANDING_SEARCH)
                else:
                    self.current_waypoint_index = 0
                    self.transition_to(MissionState.MISSION)
        else:
            self.goal_hold_counter = 0

    def handle_mission(self) -> None:
        if self.current_waypoint_index >= len(self.waypoints):
            self.transition_to(MissionState.LANDING_SEARCH)
            return

        current_goal = self.waypoints[self.current_waypoint_index]

        self.publish_target_position(current_goal)
        self.publish_target_yaw(self.compute_yaw_to_target(current_goal))

        distance = self.distance_to_goal(current_goal)

        if self.is_goal_reached(current_goal):
            self.goal_hold_counter += 1

            if self.goal_hold_counter == 1:
                self.get_logger().info(
                    f'Waypoint {self.current_waypoint_index + 1} entered tolerance zone '
                    f'(distance={distance:.2f} m)'
                )

            if self.goal_hold_counter >= self.hold_count_required:
                self.get_logger().info(
                    f'Waypoint {self.current_waypoint_index + 1} reached and stabilized: '
                    f'({current_goal.x:.2f}, {current_goal.y:.2f}, {current_goal.z:.2f})'
                )
                self.goal_hold_counter = 0
                self.current_waypoint_index += 1

                if self.current_waypoint_index < len(self.waypoints):
                    next_goal = self.waypoints[self.current_waypoint_index]
                    self.get_logger().info(
                        f'Moving to waypoint {self.current_waypoint_index + 1}: '
                        f'({next_goal.x:.2f}, {next_goal.y:.2f}, {next_goal.z:.2f})'
                    )
                else:
                    self.get_logger().info('All mission waypoints completed')
        else:
            self.goal_hold_counter = 0

    def handle_landing_search(self) -> None:
        self.publish_target_position(self.landing_staging_point)
        self.publish_target_yaw(self.landing_staging_yaw)

        if self.is_goal_reached(self.landing_staging_point):
            self.goal_hold_counter += 1

            if self.goal_hold_counter >= self.landing_search_hold_count:
                self.get_logger().info(
                    'Landing staging point reached. '
                    'This is where landing_manager.py will later take over.'
                )
                # For now, until landing_manager.py is added, go to plain land.
                self.transition_to(MissionState.LAND)
        else:
            self.goal_hold_counter = 0

    def handle_land(self) -> None:
        if not self.land_command_sent:
            msg = Bool()
            msg.data = True
            self.land_pub.publish(msg)
            self.land_command_sent = True
            self.get_logger().info('Landing command published')
            self.transition_to(MissionState.DONE)

    def handle_done(self) -> None:
        pass

    # ------------------------------------------------------------------
    # Publish helpers
    # ------------------------------------------------------------------
    def publish_target_position(self, target: Point) -> None:
        self.target_position_pub.publish(target)

    def publish_target_yaw(self, yaw: float) -> None:
        msg = Float32()
        msg.data = float(yaw)
        self.target_yaw_pub.publish(msg)

    def publish_landing_mode(self, enabled: bool) -> None:
        msg = Bool()
        msg.data = enabled
        self.landing_mode_pub.publish(msg)

    # ------------------------------------------------------------------
    # Geometry helpers
    # ------------------------------------------------------------------
    def compute_yaw_to_target(self, target: Point) -> float:
        dx = target.x - self.current_position.x
        dy = target.y - self.current_position.y

        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            return 0.0

        return math.atan2(dy, dx)

    def distance_to_goal(self, goal: Point) -> float:
        dx = goal.x - self.current_position.x
        dy = goal.y - self.current_position.y
        dz = goal.z - self.current_position.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def is_goal_reached(self, goal: Point) -> bool:
        return self.distance_to_goal(goal) < self.position_tolerance

    def transition_to(self, new_state: MissionState) -> None:
        old_state = self.state
        self.state = new_state
        self.goal_hold_counter = 0

        self.get_logger().info(
            f'State transition: {old_state.name} -> {new_state.name}'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionManagerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down mission manager node')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()