#!/usr/bin/env python3

import heapq
import math
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


GridCell = Tuple[int, int]


class AStarPlannerNode(Node):
    """2D A* planner with waypoint execution for the quadrotor simulator."""

    def __init__(self) -> None:
        super().__init__("astar_planner")

        self.declare_parameter("waypoint_tolerance_m", 0.25)
        self.declare_parameter("goal_tolerance_m", 0.20)
        self.declare_parameter("command_rate_hz", 10.0)
        self.declare_parameter("default_flight_height_m", 1.5)
        self.declare_parameter("allow_diagonal", True)
        self.declare_parameter("occupancy_threshold", 50)
        self.declare_parameter("path_z_offset_m", 0.02)

        self.waypoint_tolerance_m = float(
            self.get_parameter("waypoint_tolerance_m").value
        )
        self.goal_tolerance_m = float(
            self.get_parameter("goal_tolerance_m").value
        )
        self.command_rate_hz = float(
            self.get_parameter("command_rate_hz").value
        )
        self.default_flight_height_m = float(
            self.get_parameter("default_flight_height_m").value
        )
        self.allow_diagonal = bool(
            self.get_parameter("allow_diagonal").value
        )
        self.occupancy_threshold = int(
            self.get_parameter("occupancy_threshold").value
        )
        self.path_z_offset_m = float(
            self.get_parameter("path_z_offset_m").value
        )

        self.grid: Optional[OccupancyGrid] = None
        self.current_position: Optional[Tuple[float, float, float]] = None
        self.raw_goal: Optional[PoseStamped] = None

        self.world_path: List[Tuple[float, float, float]] = []
        self.active_waypoint_index = 0
        self.plan_valid = False
        self.mission_complete = False

        self.create_subscription(
            OccupancyGrid,
            "/drone/obstacle_grid",
            self.grid_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            "/drone/odom",
            self.odom_callback,
            20,
        )
        self.create_subscription(
            PoseStamped,
            "/drone/goal_raw",
            self.goal_callback,
            10,
        )

        self.goal_pub = self.create_publisher(
            PoseStamped,
            "/drone/goal",
            20,
        )
        self.path_pub = self.create_publisher(
            Path,
            "/drone/planned_path",
            10,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            "/drone/planner_markers",
            10,
        )

        self.command_timer = self.create_timer(
            1.0 / self.command_rate_hz,
            self.command_step,
        )

        self.get_logger().info(
            "A* planner started"
        )
        self.get_logger().info(
            "Input: /drone/goal_raw, output: /drone/goal"
        )

    def grid_callback(self, message: OccupancyGrid) -> None:
        self.grid = message

        if self.raw_goal is not None and not self.plan_valid:
            self.try_plan()

    def odom_callback(self, message: Odometry) -> None:
        self.current_position = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
            float(message.pose.pose.position.z),
        )

        if self.raw_goal is not None and not self.plan_valid:
            self.try_plan()

        self.advance_waypoint_if_reached()

    def goal_callback(self, message: PoseStamped) -> None:
        self.raw_goal = message
        self.plan_valid = False
        self.mission_complete = False
        self.world_path = []
        self.active_waypoint_index = 0

        self.get_logger().info(
            "New raw goal: "
            f"x={message.pose.position.x:.2f}, "
            f"y={message.pose.position.y:.2f}, "
            f"z={message.pose.position.z:.2f}"
        )

        self.try_plan()

    def world_to_grid(
        self,
        x: float,
        y: float,
    ) -> Optional[GridCell]:
        if self.grid is None:
            return None

        resolution = self.grid.info.resolution
        origin_x = self.grid.info.origin.position.x
        origin_y = self.grid.info.origin.position.y

        column = int(math.floor((x - origin_x) / resolution))
        row = int(math.floor((y - origin_y) / resolution))

        if (
            column < 0
            or row < 0
            or column >= self.grid.info.width
            or row >= self.grid.info.height
        ):
            return None

        return (column, row)

    def grid_to_world(
        self,
        cell: GridCell,
    ) -> Tuple[float, float]:
        assert self.grid is not None

        resolution = self.grid.info.resolution
        origin_x = self.grid.info.origin.position.x
        origin_y = self.grid.info.origin.position.y

        x = origin_x + (cell[0] + 0.5) * resolution
        y = origin_y + (cell[1] + 0.5) * resolution
        return (x, y)

    def is_free(self, cell: GridCell) -> bool:
        if self.grid is None:
            return False

        column, row = cell

        if (
            column < 0
            or row < 0
            or column >= self.grid.info.width
            or row >= self.grid.info.height
        ):
            return False

        value = self.grid.data[
            row * self.grid.info.width + column
        ]

        return value >= 0 and value < self.occupancy_threshold

    def nearest_free_cell(
        self,
        start: GridCell,
        maximum_radius: int = 12,
    ) -> Optional[GridCell]:
        if self.is_free(start):
            return start

        for radius in range(1, maximum_radius + 1):
            for dx in range(-radius, radius + 1):
                for dy in (-radius, radius):
                    candidate = (
                        start[0] + dx,
                        start[1] + dy,
                    )
                    if self.is_free(candidate):
                        return candidate

            for dy in range(-radius + 1, radius):
                for dx in (-radius, radius):
                    candidate = (
                        start[0] + dx,
                        start[1] + dy,
                    )
                    if self.is_free(candidate):
                        return candidate

        return None

    def neighbours(
        self,
        cell: GridCell,
    ) -> List[Tuple[GridCell, float]]:
        output: List[Tuple[GridCell, float]] = []

        motions = [
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),
        ]

        if self.allow_diagonal:
            diagonal_cost = math.sqrt(2.0)
            motions.extend([
                (-1, -1, diagonal_cost),
                (-1, 1, diagonal_cost),
                (1, -1, diagonal_cost),
                (1, 1, diagonal_cost),
            ])

        for dx, dy, cost in motions:
            candidate = (
                cell[0] + dx,
                cell[1] + dy,
            )

            if not self.is_free(candidate):
                continue

            if dx != 0 and dy != 0:
                side_a = (cell[0] + dx, cell[1])
                side_b = (cell[0], cell[1] + dy)

                if not self.is_free(side_a) or not self.is_free(side_b):
                    continue

            output.append((candidate, cost))

        return output

    @staticmethod
    def heuristic(
        cell: GridCell,
        goal: GridCell,
    ) -> float:
        return math.hypot(
            goal[0] - cell[0],
            goal[1] - cell[1],
        )

    def astar(
        self,
        start: GridCell,
        goal: GridCell,
    ) -> Optional[List[GridCell]]:
        frontier: List[Tuple[float, GridCell]] = []
        heapq.heappush(frontier, (0.0, start))

        came_from: Dict[GridCell, Optional[GridCell]] = {
            start: None
        }
        cost_so_far: Dict[GridCell, float] = {
            start: 0.0
        }

        while frontier:
            _, current = heapq.heappop(frontier)

            if current == goal:
                break

            for neighbour, motion_cost in self.neighbours(current):
                new_cost = cost_so_far[current] + motion_cost

                if (
                    neighbour not in cost_so_far
                    or new_cost < cost_so_far[neighbour]
                ):
                    cost_so_far[neighbour] = new_cost
                    priority = (
                        new_cost
                        + self.heuristic(neighbour, goal)
                    )
                    heapq.heappush(
                        frontier,
                        (priority, neighbour),
                    )
                    came_from[neighbour] = current

        if goal not in came_from:
            return None

        path: List[GridCell] = []
        current: Optional[GridCell] = goal

        while current is not None:
            path.append(current)
            current = came_from[current]

        path.reverse()
        return path

    def line_is_free(
        self,
        start: GridCell,
        end: GridCell,
    ) -> bool:
        x0, y0 = start
        x1, y1 = end

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        step_x = 1 if x0 < x1 else -1
        step_y = 1 if y0 < y1 else -1
        error = dx - dy

        current_x = x0
        current_y = y0

        while True:
            if not self.is_free((current_x, current_y)):
                return False

            if current_x == x1 and current_y == y1:
                return True

            twice_error = 2 * error

            if twice_error > -dy:
                error -= dy
                current_x += step_x

            if twice_error < dx:
                error += dx
                current_y += step_y

    def simplify_path(
        self,
        path: List[GridCell],
    ) -> List[GridCell]:
        if len(path) <= 2:
            return path

        simplified = [path[0]]
        anchor_index = 0

        while anchor_index < len(path) - 1:
            furthest_visible = anchor_index + 1

            for candidate_index in range(
                anchor_index + 2,
                len(path),
            ):
                if self.line_is_free(
                    path[anchor_index],
                    path[candidate_index],
                ):
                    furthest_visible = candidate_index
                else:
                    break

            simplified.append(path[furthest_visible])
            anchor_index = furthest_visible

        return simplified

    def try_plan(self) -> None:
        if (
            self.grid is None
            or self.current_position is None
            or self.raw_goal is None
        ):
            return

        start = self.world_to_grid(
            self.current_position[0],
            self.current_position[1],
        )
        raw_goal_cell = self.world_to_grid(
            self.raw_goal.pose.position.x,
            self.raw_goal.pose.position.y,
        )

        if start is None or raw_goal_cell is None:
            self.get_logger().error(
                "Start or goal is outside the occupancy grid"
            )
            return

        raw_goal_is_free = self.is_free(raw_goal_cell)

        start = self.nearest_free_cell(start)
        goal = self.nearest_free_cell(raw_goal_cell)

        if start is None or goal is None:
            self.get_logger().error(
                "Could not find a nearby free start or goal cell"
            )
            return

        grid_path = self.astar(start, goal)

        if grid_path is None:
            self.get_logger().error(
                "A* failed to find a collision-free path"
            )
            return

        simplified = self.simplify_path(grid_path)

        requested_z = float(
            self.raw_goal.pose.position.z
        )
        flight_z = (
            requested_z
            if requested_z > 0.05
            else self.default_flight_height_m
        )

        self.world_path = []

        for cell in simplified:
            x, y = self.grid_to_world(cell)
            self.world_path.append((x, y, flight_z))

        if raw_goal_is_free:
            self.world_path[-1] = (
                float(self.raw_goal.pose.position.x),
                float(self.raw_goal.pose.position.y),
                flight_z,
            )
        else:
            adjusted_x, adjusted_y = self.grid_to_world(goal)
            self.world_path[-1] = (
                adjusted_x,
                adjusted_y,
                flight_z,
            )
            self.get_logger().warn(
                "Requested goal lies in an occupied safety zone; "
                "using the nearest free grid cell instead"
            )

        self.active_waypoint_index = (
            1 if len(self.world_path) > 1 else 0
        )
        self.plan_valid = True

        self.publish_path()
        self.publish_markers()

        self.get_logger().info(
            f"A* path ready: {len(grid_path)} grid cells, "
            f"{len(self.world_path)} simplified waypoints"
        )

    def advance_waypoint_if_reached(self) -> None:
        if (
            not self.plan_valid
            or self.mission_complete
            or self.current_position is None
            or not self.world_path
        ):
            return

        target = self.world_path[self.active_waypoint_index]
        error = math.sqrt(
            (target[0] - self.current_position[0]) ** 2
            + (target[1] - self.current_position[1]) ** 2
            + (target[2] - self.current_position[2]) ** 2
        )

        tolerance = (
            self.goal_tolerance_m
            if self.active_waypoint_index
            == len(self.world_path) - 1
            else self.waypoint_tolerance_m
        )

        if error > tolerance:
            return

        if self.active_waypoint_index < len(self.world_path) - 1:
            self.active_waypoint_index += 1
            self.publish_markers()
            self.get_logger().info(
                "Advancing to planner waypoint "
                f"{self.active_waypoint_index + 1}/"
                f"{len(self.world_path)}"
            )
        else:
            self.mission_complete = True
            self.get_logger().info(
                "Final planned goal reached"
            )

    def command_step(self) -> None:
        if (
            not self.plan_valid
            or self.mission_complete
            or not self.world_path
            or self.raw_goal is None
        ):
            return

        target = self.world_path[self.active_waypoint_index]

        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.pose.position.x = target[0]
        message.pose.position.y = target[1]
        message.pose.position.z = target[2]
        message.pose.orientation = (
            self.raw_goal.pose.orientation
        )

        if (
            abs(message.pose.orientation.w) < 1.0e-9
            and abs(message.pose.orientation.x) < 1.0e-9
            and abs(message.pose.orientation.y) < 1.0e-9
            and abs(message.pose.orientation.z) < 1.0e-9
        ):
            message.pose.orientation.w = 1.0

        self.goal_pub.publish(message)

    def publish_path(self) -> None:
        message = Path()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"

        for x, y, z in self.world_path:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z + self.path_z_offset_m
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)

        self.path_pub.publish(message)

    def publish_markers(self) -> None:
        markers = MarkerArray()

        waypoint_marker = Marker()
        waypoint_marker.header.frame_id = "map"
        waypoint_marker.header.stamp = self.get_clock().now().to_msg()
        waypoint_marker.ns = "planner_waypoints"
        waypoint_marker.id = 0
        waypoint_marker.type = Marker.SPHERE_LIST
        waypoint_marker.action = Marker.ADD
        waypoint_marker.scale.x = 0.14
        waypoint_marker.scale.y = 0.14
        waypoint_marker.scale.z = 0.14
        waypoint_marker.color.r = 0.15
        waypoint_marker.color.g = 0.55
        waypoint_marker.color.b = 1.0
        waypoint_marker.color.a = 0.95

        for x, y, z in self.world_path:
            point = Point()
            point.x = x
            point.y = y
            point.z = z
            waypoint_marker.points.append(point)

        markers.markers.append(waypoint_marker)

        if self.world_path:
            target = self.world_path[self.active_waypoint_index]

            active = Marker()
            active.header.frame_id = "map"
            active.header.stamp = self.get_clock().now().to_msg()
            active.ns = "active_planner_waypoint"
            active.id = 1
            active.type = Marker.SPHERE
            active.action = Marker.ADD
            active.pose.position.x = target[0]
            active.pose.position.y = target[1]
            active.pose.position.z = target[2]
            active.pose.orientation.w = 1.0
            active.scale.x = 0.24
            active.scale.y = 0.24
            active.scale.z = 0.24
            active.color.r = 1.0
            active.color.g = 0.85
            active.color.b = 0.10
            active.color.a = 0.95
            markers.markers.append(active)

        self.marker_pub.publish(markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AStarPlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
