#!/usr/bin/env python3

import math
from enum import Enum

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Float32, String


class MissionState(Enum):
    WAIT_FOR_POSITION = 0
    TAKEOFF = 1
    MISSION = 2
    LANDING_SEARCH = 3
    DONE = 4


class MissionManagerNode(Node):
    def __init__(self) -> None:
        super().__init__('mission_manager_node')

        self.declare_parameter('takeoff_altitude', -5.0)
        self.declare_parameter('xy_tolerance', 0.75)
        self.declare_parameter('z_tolerance', 0.50)
        self.declare_parameter('hold_count_required', 5)
        self.declare_parameter('landing_search_hold_count', 10)
        self.declare_parameter(
            'waypoints',
            [5.0, 0.0, -5.0,
             5.0, 5.0, -5.0,
             0.0, 5.0, -5.0]
        )
        self.declare_parameter('landing_staging_x', 0.0)
        self.declare_parameter('landing_staging_y', 0.0)
        self.declare_parameter('landing_staging_z', -5.0)
        self.declare_parameter('landing_staging_yaw', 0.0)
        self.declare_parameter('skip_waypoints_and_land', False)

        self.takeoff_altitude = float(self.get_parameter('takeoff_altitude').value)
        self.xy_tolerance = float(self.get_parameter('xy_tolerance').value)
        self.z_tolerance = float(self.get_parameter('z_tolerance').value)
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

        self.target_position_pub = self.create_publisher(Point, '/mission/target_position', 10)
        self.target_yaw_pub = self.create_publisher(Float32, '/mission/target_yaw', 10)
        self.landing_mode_pub = self.create_publisher(Bool, '/mission/landing_mode', 10)
        self.state_pub = self.create_publisher(String, '/mission/state', 10)

        self.current_position_sub = self.create_subscription(
            Point,
            '/mission/current_position',
            self.current_position_callback,
            10
        )

        self.state = MissionState.WAIT_FOR_POSITION
        self.current_position = Point()
        self.have_position = False
        self.current_waypoint_index = 0
        self.goal_hold_counter = 0
        self.landing_mode_locked = False

        self.last_status_log_time = 0.0

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('Mission manager node started')
        self.get_logger().info(
            f'takeoff_altitude={self.takeoff_altitude:.2f}, '
            f'xy_tolerance={self.xy_tolerance:.2f}, '
            f'z_tolerance={self.z_tolerance:.2f}, '
            f'hold_count_required={self.hold_count_required}'
        )

        for i, wp in enumerate(self.waypoints, start=1):
            self.get_logger().info(
                f'Waypoint {i}: x={wp.x:.2f}, y={wp.y:.2f}, z={wp.z:.2f}'
            )

    def parse_waypoints(self, waypoint_list) -> list[Point]:
        waypoints = []

        if len(waypoint_list) % 3 != 0:
            raise ValueError('Parameter "waypoints" must contain [x1, y1, z1, ...]')

        for i in range(0, len(waypoint_list), 3):
            wp = Point()
            wp.x = float(waypoint_list[i])
            wp.y = float(waypoint_list[i + 1])
            wp.z = float(waypoint_list[i + 2])
            waypoints.append(wp)

        return waypoints

    def current_position_callback(self, msg: Point) -> None:
        self.current_position = msg
        if not self.have_position:
            self.have_position = True
            self.get_logger().info(
                f'First position received: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}'
            )

    def publish_state(self) -> None:
        msg = String()
        msg.data = self.state.name
        self.state_pub.publish(msg)

    def maybe_log_status(self, text: str, period_sec: float = 1.0) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_status_log_time >= period_sec:
            self.get_logger().info(text)
            self.last_status_log_time = now

    def timer_callback(self) -> None:
        self.publish_state()
        self.publish_landing_mode(self.landing_mode_locked)

        if self.state == MissionState.WAIT_FOR_POSITION:
            self.handle_wait_for_position()
        elif self.state == MissionState.TAKEOFF:
            self.handle_takeoff()
        elif self.state == MissionState.MISSION:
            self.handle_mission()
        elif self.state == MissionState.LANDING_SEARCH:
            self.handle_landing_search()
        elif self.state == MissionState.DONE:
            self.handle_done()

    def handle_wait_for_position(self) -> None:
        self.maybe_log_status('State=WAIT_FOR_POSITION')
        if self.have_position:
            self.transition_to(MissionState.TAKEOFF)

    def handle_takeoff(self) -> None:
        target = Point()
        target.x = 0.0
        target.y = 0.0
        target.z = self.takeoff_altitude

        self.publish_target_position(target)
        self.publish_target_yaw(0.0)

        xy_error = self.xy_distance_to_goal(target)
        z_error = abs(target.z - self.current_position.z)

        self.maybe_log_status(
            f'State=TAKEOFF | current=({self.current_position.x:.2f}, '
            f'{self.current_position.y:.2f}, {self.current_position.z:.2f}) '
            f'| target=(0.00, 0.00, {target.z:.2f}) '
            f'| xy_error={xy_error:.2f}, z_error={z_error:.2f}, hold={self.goal_hold_counter}'
        )

        if xy_error < self.xy_tolerance and z_error < self.z_tolerance:
            self.goal_hold_counter += 1
            if self.goal_hold_counter >= self.hold_count_required:
                self.get_logger().info('Takeoff complete. Switching to mission.')
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
            self.get_logger().info('All waypoints completed. Switching to landing search.')
            self.transition_to(MissionState.LANDING_SEARCH)
            return

        target = self.waypoints[self.current_waypoint_index]

        self.publish_target_position(target)
        self.publish_target_yaw(self.compute_yaw_to_target(target))

        xy_error = self.xy_distance_to_goal(target)
        z_error = abs(target.z - self.current_position.z)

        self.maybe_log_status(
            f'State=MISSION | wp={self.current_waypoint_index + 1}/{len(self.waypoints)} '
            f'| current=({self.current_position.x:.2f}, {self.current_position.y:.2f}, {self.current_position.z:.2f}) '
            f'| target=({target.x:.2f}, {target.y:.2f}, {target.z:.2f}) '
            f'| xy_error={xy_error:.2f}, z_error={z_error:.2f}, hold={self.goal_hold_counter}'
        )

        if xy_error < self.xy_tolerance and z_error < self.z_tolerance:
            self.goal_hold_counter += 1
            if self.goal_hold_counter >= self.hold_count_required:
                self.get_logger().info(f'Waypoint {self.current_waypoint_index + 1} reached.')
                self.goal_hold_counter = 0
                self.current_waypoint_index += 1
        else:
            self.goal_hold_counter = 0

    def handle_landing_search(self) -> None:
        if not self.landing_mode_locked:
            self.publish_target_position(self.landing_staging_point)
            self.publish_target_yaw(self.landing_staging_yaw)

            xy_error = self.xy_distance_to_goal(self.landing_staging_point)
            z_error = abs(self.landing_staging_point.z - self.current_position.z)

            self.maybe_log_status(
                f'State=LANDING_SEARCH | current=({self.current_position.x:.2f}, {self.current_position.y:.2f}, {self.current_position.z:.2f}) '
                f'| staging=({self.landing_staging_point.x:.2f}, {self.landing_staging_point.y:.2f}, {self.landing_staging_point.z:.2f}) '
                f'| xy_error={xy_error:.2f}, z_error={z_error:.2f}, hold={self.goal_hold_counter}'
            )

            if xy_error < self.xy_tolerance and z_error < self.z_tolerance:
                self.goal_hold_counter += 1
                if self.goal_hold_counter >= self.landing_search_hold_count:
                    self.get_logger().info(
                        'Landing staging point reached. Handing control to landing_manager.'
                    )
                    self.landing_mode_locked = True
                    self.goal_hold_counter = 0
            else:
                self.goal_hold_counter = 0

    def handle_done(self) -> None:
        self.maybe_log_status('State=DONE', period_sec=2.0)

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

    def xy_distance_to_goal(self, goal: Point) -> float:
        dx = goal.x - self.current_position.x
        dy = goal.y - self.current_position.y
        return math.sqrt(dx * dx + dy * dy)

    def compute_yaw_to_target(self, target: Point) -> float:
        dx = target.x - self.current_position.x
        dy = target.y - self.current_position.y

        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            return 0.0

        return math.atan2(dy, dx)

    def transition_to(self, new_state: MissionState) -> None:
        old_state = self.state
        self.state = new_state
        self.goal_hold_counter = 0
        self.get_logger().info(f'State transition: {old_state.name} -> {new_state.name}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionManagerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()