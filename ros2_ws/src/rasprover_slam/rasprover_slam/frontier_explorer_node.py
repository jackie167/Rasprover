import math
from collections import deque
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


GridCell = Tuple[int, int]


class FrontierExplorerNode(Node):
    def __init__(self) -> None:
        super().__init__('frontier_explorer_node')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('plan_period_sec', 2.0)
        self.declare_parameter('min_frontier_cluster_size', 8)
        self.declare_parameter('min_goal_distance_m', 0.6)
        self.declare_parameter('max_goal_distance_m', 8.0)
        self.declare_parameter('blacklist_radius_m', 0.8)
        self.declare_parameter('goal_reached_radius_m', 0.7)
        self.declare_parameter('goal_timeout_sec', 45.0)
        self.declare_parameter('frontier_weight_distance', 1.0)
        self.declare_parameter('frontier_weight_size', 0.06)

        self.map_topic = self.get_parameter('map_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.plan_period_sec = float(self.get_parameter('plan_period_sec').value)
        self.min_frontier_cluster_size = int(self.get_parameter('min_frontier_cluster_size').value)
        self.min_goal_distance_m = float(self.get_parameter('min_goal_distance_m').value)
        self.max_goal_distance_m = float(self.get_parameter('max_goal_distance_m').value)
        self.blacklist_radius_m = float(self.get_parameter('blacklist_radius_m').value)
        self.goal_reached_radius_m = float(self.get_parameter('goal_reached_radius_m').value)
        self.goal_timeout_sec = float(self.get_parameter('goal_timeout_sec').value)
        self.frontier_weight_distance = float(self.get_parameter('frontier_weight_distance').value)
        self.frontier_weight_size = float(self.get_parameter('frontier_weight_size').value)

        self.map_msg: Optional[OccupancyGrid] = None
        self.current_goal: Optional[Tuple[float, float]] = None
        self.current_goal_sent_at: Optional[Time] = None
        self.goal_handle = None
        self.blacklist: List[Tuple[float, float]] = []

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.navigate_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.map_sub = self.create_subscription(OccupancyGrid, self.map_topic, self._map_callback, 10)
        self.timer = self.create_timer(self.plan_period_sec, self._tick)

        self.get_logger().info(
            'frontier_explorer_node map=%s action=/navigate_to_pose base=%s map_frame=%s',
            self.map_topic,
            self.base_frame,
            self.map_frame,
        )

    def _map_callback(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg

    def _tick(self) -> None:
        if self.map_msg is None:
            return

        robot_pose = self._lookup_robot_pose()
        if robot_pose is None:
            return

        if self._goal_active(robot_pose):
            return

        frontier_goal = self._select_frontier_goal(robot_pose)
        if frontier_goal is None:
            self.get_logger().debug('no frontier goal found')
            return

        self._send_goal(robot_pose, frontier_goal)

    def _goal_active(self, robot_pose: Tuple[float, float, float]) -> bool:
        if self.goal_handle is None or self.current_goal is None or self.current_goal_sent_at is None:
            return False

        now = self.get_clock().now()
        goal_age = (now - self.current_goal_sent_at).nanoseconds / 1e9
        dist_to_goal = math.hypot(self.current_goal[0] - robot_pose[0], self.current_goal[1] - robot_pose[1])

        if dist_to_goal <= self.goal_reached_radius_m:
            self.get_logger().info('goal reached locally, scheduling new frontier')
            self.goal_handle = None
            self.current_goal = None
            self.current_goal_sent_at = None
            return False

        if goal_age > self.goal_timeout_sec:
            self.get_logger().warn('frontier goal timed out, blacklisting target')
            self.blacklist.append(self.current_goal)
            self.goal_handle = None
            self.current_goal = None
            self.current_goal_sent_at = None
            return False

        return True

    def _lookup_robot_pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.2),
            )
        except TransformException:
            return None

        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        q = transform.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return tx, ty, yaw

    def _select_frontier_goal(self, robot_pose: Tuple[float, float, float]) -> Optional[Tuple[float, float]]:
        grid = self.map_msg
        assert grid is not None

        width = grid.info.width
        height = grid.info.height
        resolution = grid.info.resolution
        data = grid.data

        frontier_cells: List[GridCell] = []
        for y in range(1, height - 1):
            row_offset = y * width
            for x in range(1, width - 1):
                idx = row_offset + x
                if data[idx] != 0:
                    continue
                if self._has_unknown_neighbor(data, width, height, x, y):
                    frontier_cells.append((x, y))

        if not frontier_cells:
            return None

        frontier_set = set(frontier_cells)
        visited = set()
        robot_x, robot_y, _ = robot_pose
        best_goal = None
        best_score = None

        for cell in frontier_cells:
            if cell in visited:
                continue
            cluster = self._grow_cluster(cell, frontier_set, visited)
            if len(cluster) < self.min_frontier_cluster_size:
                continue

            goal = self._cluster_centroid_world(cluster, grid.info)
            if goal is None:
                continue

            distance = math.hypot(goal[0] - robot_x, goal[1] - robot_y)
            if distance < self.min_goal_distance_m or distance > self.max_goal_distance_m:
                continue
            if self._is_blacklisted(goal):
                continue

            score = self.frontier_weight_distance * distance - self.frontier_weight_size * len(cluster)
            if best_score is None or score < best_score:
                best_score = score
                best_goal = goal

        return best_goal

    def _has_unknown_neighbor(self, data, width: int, height: int, x: int, y: int) -> bool:
        for ny in range(max(0, y - 1), min(height, y + 2)):
            for nx in range(max(0, x - 1), min(width, x + 2)):
                if nx == x and ny == y:
                    continue
                if data[ny * width + nx] == -1:
                    return True
        return False

    def _grow_cluster(self, start: GridCell, frontier_set, visited) -> List[GridCell]:
        cluster: List[GridCell] = []
        queue = deque([start])
        visited.add(start)

        while queue:
            cell = queue.popleft()
            cluster.append(cell)
            cx, cy = cell
            for ny in range(cy - 1, cy + 2):
                for nx in range(cx - 1, cx + 2):
                    neighbor = (nx, ny)
                    if neighbor in frontier_set and neighbor not in visited:
                        visited.add(neighbor)
                        queue.append(neighbor)
        return cluster

    def _cluster_centroid_world(self, cluster: List[GridCell], info) -> Optional[Tuple[float, float]]:
        if not cluster:
            return None

        mean_x = sum(cell[0] for cell in cluster) / len(cluster)
        mean_y = sum(cell[1] for cell in cluster) / len(cluster)
        world_x = info.origin.position.x + (mean_x + 0.5) * info.resolution
        world_y = info.origin.position.y + (mean_y + 0.5) * info.resolution
        return world_x, world_y

    def _is_blacklisted(self, goal: Tuple[float, float]) -> bool:
        for bx, by in self.blacklist:
            if math.hypot(goal[0] - bx, goal[1] - by) <= self.blacklist_radius_m:
                return True
        return False

    def _send_goal(self, robot_pose: Tuple[float, float, float], goal_xy: Tuple[float, float]) -> None:
        if not self.navigate_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn('navigate_to_pose action server not available yet')
            return

        robot_x, robot_y, _ = robot_pose
        goal_yaw = math.atan2(goal_xy[1] - robot_y, goal_xy[0] - robot_x)

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = goal_xy[0]
        goal.pose.pose.position.y = goal_xy[1]
        goal.pose.pose.orientation.z = math.sin(goal_yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(goal_yaw / 2.0)

        self.get_logger().info(
            'sending frontier goal x=%.2f y=%.2f yaw=%.1f deg',
            goal_xy[0],
            goal_xy[1],
            math.degrees(goal_yaw),
        )
        future = self.navigate_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)
        self.current_goal = goal_xy
        self.current_goal_sent_at = self.get_clock().now()

    def _goal_response_cb(self, future) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('frontier goal rejected')
            if self.current_goal is not None:
                self.blacklist.append(self.current_goal)
            self.goal_handle = None
            self.current_goal = None
            self.current_goal_sent_at = None
            return

        self.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future) -> None:
        status = future.result().status
        if status != 4:
            self.get_logger().info('frontier goal finished with status=%s', status)
        else:
            self.get_logger().warn('frontier goal aborted, blacklisting target')
            if self.current_goal is not None:
                self.blacklist.append(self.current_goal)

        self.goal_handle = None
        self.current_goal = None
        self.current_goal_sent_at = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrontierExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
