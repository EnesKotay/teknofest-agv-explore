#!/usr/bin/env python3
"""
Advanced Frontier Explorer for Autonomous Exploration
Teknofest Sanayi Dijital Teknolojiler - İleri Seviye Yarışması

Özellikler:
- Information Gain based frontier selection
- Frontier clustering for better goal selection
- Stuck detection and recovery
- Real-time exploration metrics
- Visualization with MarkerArray
- Performance optimizations (ROI-based scanning)
"""

import math
from typing import Optional, Tuple, List, Set, Dict
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32, Header, ColorRGBA

import tf2_ros


def neighbors4(x: int, y: int):
    """4-connected neighbors"""
    yield x + 1, y
    yield x - 1, y
    yield x, y + 1
    yield x, y - 1


def neighbors8(x: int, y: int):
    """8-connected neighbors for clustering"""
    for dx in [-1, 0, 1]:
        for dy in [-1, 0, 1]:
            if dx == 0 and dy == 0:
                continue
            yield x + dx, y + dy


class FrontierCluster:
    """Frontier cluster representation"""
    def __init__(self, x: int, y: int):
        self.cells: List[Tuple[int, int]] = [(x, y)]
        self.center_x = x
        self.center_y = y
        self.size = 1
        self.information_gain = 0.0  # Will be calculated
    
    def add_cell(self, x: int, y: int):
        self.cells.append((x, y))
        self.size = len(self.cells)
        # Update center (simple average)
        self.center_x = sum(c[0] for c in self.cells) // self.size
        self.center_y = sum(c[1] for c in self.cells) // self.size
    
    def contains(self, x: int, y: int) -> bool:
        return (x, y) in self.cells


class FrontierExplorer(Node):
    """
    Advanced Frontier Explorer with:
    - Information gain based selection
    - Frontier clustering
    - Stuck detection
    - Metrics and visualization
    """

    def __init__(self):
        super().__init__("frontier_explorer")

        # Params
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("goal_frame", "map")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("nav_action", "navigate_to_pose")

        self.declare_parameter("min_goal_distance", 0.5)
        self.declare_parameter("goal_cooldown_sec", 0.5)
        self.declare_parameter("goal_timeout_sec", 60.0)
        self.declare_parameter("replan_period_sec", 1.5)

        self.declare_parameter("free_value_max", 10)
        self.declare_parameter("occupied_value_min", 65)
        self.declare_parameter("safety_cell_radius", 3)
        
        self.declare_parameter("return_to_start", True)
        self.declare_parameter("start_position_timeout", 10.0)
        
        # New parameters for advanced features
        self.declare_parameter("use_information_gain", True)
        self.declare_parameter("information_gain_weight", 2.0)  # Weight for information gain vs distance
        self.declare_parameter("frontier_cluster_distance", 5)  # Pixels - max distance for clustering
        self.declare_parameter("min_cluster_size", 3)  # Minimum cells in a cluster
        
        self.declare_parameter("stuck_detection_enabled", True)
        self.declare_parameter("stuck_distance_threshold", 0.5)  # meters
        self.declare_parameter("stuck_time_threshold", 30.0)  # seconds
        
        self.declare_parameter("roi_radius", 15.0)  # meters - Region of Interest radius
        self.declare_parameter("publish_visualization", True)
        self.declare_parameter("publish_metrics", True)
        
        self.declare_parameter("visualization_topic", "/frontier_markers")
        self.declare_parameter("metrics_topic_prefix", "/exploration_metrics")

        # Get parameters
        self.map_topic = self.get_parameter("map_topic").value
        self.goal_frame = self.get_parameter("goal_frame").value
        self.robot_frame = self.get_parameter("robot_frame").value
        self.nav_action = self.get_parameter("nav_action").value

        self.min_goal_distance = float(self.get_parameter("min_goal_distance").value)
        self.goal_cooldown = float(self.get_parameter("goal_cooldown_sec").value)
        self.goal_timeout = float(self.get_parameter("goal_timeout_sec").value)
        self.replan_period = float(self.get_parameter("replan_period_sec").value)

        self.free_value_max = int(self.get_parameter("free_value_max").value)
        self.occupied_value_min = int(self.get_parameter("occupied_value_min").value)
        self.safety_cell_radius = int(self.get_parameter("safety_cell_radius").value)
        self.return_to_start = bool(self.get_parameter("return_to_start").value)
        self.start_position_timeout = float(self.get_parameter("start_position_timeout").value)
        
        self.use_information_gain = bool(self.get_parameter("use_information_gain").value)
        self.information_gain_weight = float(self.get_parameter("information_gain_weight").value)
        self.frontier_cluster_distance = int(self.get_parameter("frontier_cluster_distance").value)
        self.min_cluster_size = int(self.get_parameter("min_cluster_size").value)
        
        self.stuck_detection_enabled = bool(self.get_parameter("stuck_detection_enabled").value)
        self.stuck_distance_threshold = float(self.get_parameter("stuck_distance_threshold").value)
        self.stuck_time_threshold = float(self.get_parameter("stuck_time_threshold").value)
        
        self.roi_radius = float(self.get_parameter("roi_radius").value)
        self.publish_visualization = bool(self.get_parameter("publish_visualization").value)
        self.publish_metrics = bool(self.get_parameter("publish_metrics").value)

        # TF
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Nav2 action
        self.nav_client = ActionClient(self, NavigateToPose, self.nav_action)

        # Map
        self.map_msg: Optional[OccupancyGrid] = None
        self.create_subscription(OccupancyGrid, self.map_topic, self._map_cb, 10)

        # Visualization
        if self.publish_visualization:
            self.marker_pub = self.create_publisher(
                MarkerArray, self.get_parameter("visualization_topic").value, 10
            )

        # Metrics publishers
        if self.publish_metrics:
            metrics_prefix = self.get_parameter("metrics_topic_prefix").value
            self.metrics_pubs = {
                'coverage': self.create_publisher(Float32, f"{metrics_prefix}/coverage", 10),
                'frontier_count': self.create_publisher(Float32, f"{metrics_prefix}/frontier_count", 10),
                'total_distance': self.create_publisher(Float32, f"{metrics_prefix}/total_distance", 10),
                'exploration_time': self.create_publisher(Float32, f"{metrics_prefix}/exploration_time", 10),
            }

        # State
        self.active_goal_handle = None
        self.active_goal_started = None
        self.last_goal_time = self.get_clock().now()
        self.last_goal_xy: Optional[Tuple[float, float]] = None
        self.start_position: Optional[Tuple[float, float]] = None
        self.start_position_saved = False
        self.start_time = self.get_clock().now()
        self.exploration_complete = False
        self.returning_to_start = False
        
        # Stuck detection
        self.robot_positions = deque(maxlen=50)  # Store last 50 positions
        self.last_significant_movement = self.get_clock().now()
        self.failed_goals: Set[Tuple[float, float]] = set()  # Track failed goals
        
        # Metrics
        self.total_distance = 0.0
        self.last_robot_xy: Optional[Tuple[float, float]] = None
        self.last_metrics_publish = self.get_clock().now()
        
        # Cached frontiers
        self.last_frontier_scan_time = None
        self.cached_frontier_clusters: List[FrontierCluster] = []

        self.timer = self.create_timer(self.replan_period, self._tick)

        # Metrics timer (publish every 2 seconds)
        if self.publish_metrics:
            self.metrics_timer = self.create_timer(2.0, self._publish_metrics)

        self.get_logger().info("Advanced FrontierExplorer started with:")
        self.get_logger().info(f"  - Information Gain: {self.use_information_gain}")
        self.get_logger().info(f"  - Stuck Detection: {self.stuck_detection_enabled}")
        self.get_logger().info(f"  - Visualization: {self.publish_visualization}")
        self.get_logger().info(f"  - Metrics: {self.publish_metrics}")

    def _map_cb(self, msg: OccupancyGrid):
        self.map_msg = msg
        # Invalidate cache when map updates
        self.cached_frontier_clusters = []

    def _get_robot_xy_in_map(self) -> Optional[Tuple[float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.goal_frame, self.robot_frame, rclpy.time.Time()
            )
            return (tf.transform.translation.x, tf.transform.translation.y)
        except Exception as e:
            self.get_logger().warn(f"TF not ready ({self.goal_frame}->{self.robot_frame}): {e}")
            return None

    def _cell_is_safe(self, data, w, h, cx, cy) -> bool:
        """Check if cell is safe (no obstacles in radius)"""
        r = self.safety_cell_radius
        for yy in range(cy - r, cy + r + 1):
            for xx in range(cx - r, cx + r + 1):
                if 0 <= xx < w and 0 <= yy < h:
                    v = data[yy * w + xx]
                    if v >= self.occupied_value_min:
                        return False
        return True

    def _calculate_information_gain(self, data, w, h, cx, cy, radius_cells: int = 5) -> float:
        """Calculate information gain: number of unknown cells in radius"""
        unknown_count = 0
        for yy in range(cy - radius_cells, cy + radius_cells + 1):
            for xx in range(cx - radius_cells, cx + radius_cells + 1):
                if 0 <= xx < w and 0 <= yy < h:
                    idx = yy * w + xx
                    if data[idx] == -1:  # Unknown
                        unknown_count += 1
        return float(unknown_count)

    def _is_frontier_cell(self, data, w, h, x, y) -> bool:
        """Check if cell is a frontier (free cell with unknown neighbor)"""
        idx = y * w + x
        v = data[idx]
        if v == -1 or v > self.free_value_max:
            return False
        
        # Check if has unknown neighbor
        for nx, ny in neighbors4(x, y):
            if 0 <= nx < w and 0 <= ny < h:
                if data[ny * w + nx] == -1:
                    return True
        return False

    def _cluster_frontiers(self, frontier_cells: List[Tuple[int, int]]) -> List[FrontierCluster]:
        """Cluster nearby frontier cells together"""
        if not frontier_cells:
            return []
        
        clusters: List[FrontierCluster] = []
        used = set()
        
        for fx, fy in frontier_cells:
            if (fx, fy) in used:
                continue
            
            # Start new cluster
            cluster = FrontierCluster(fx, fy)
            used.add((fx, fy))
            
            # BFS to find nearby cells
            queue = deque([(fx, fy)])
            while queue:
                cx, cy = queue.popleft()
                for nx, ny in neighbors8(cx, cy):
                    if (nx, ny) in used:
                        continue
                    if (nx, ny) not in frontier_cells:
                        continue
                    # Check distance from cluster center
                    dist = math.sqrt((nx - cluster.center_x)**2 + (ny - cluster.center_y)**2)
                    if dist <= self.frontier_cluster_distance:
                        cluster.add_cell(nx, ny)
                        used.add((nx, ny))
                        queue.append((nx, ny))
            
            if cluster.size >= self.min_cluster_size:
                clusters.append(cluster)
        
        return clusters

    def _find_frontier_clusters(self, robot_xy: Tuple[float, float]) -> List[FrontierCluster]:
        """Find and cluster frontiers with ROI optimization"""
        m = self.map_msg
        if m is None:
            return []

        w = m.info.width
        h = m.info.height
        res = m.info.resolution
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y
        data = m.data

        def idx(x, y): return y * w + x

        rx, ry = robot_xy

        # ROI: Only scan within radius
        roi_cells = int(self.roi_radius / res)
        robot_cell_x = int((rx - ox) / res)
        robot_cell_y = int((ry - oy) / res)
        
        # Bounds for ROI
        min_x = max(1, robot_cell_x - roi_cells)
        max_x = min(w - 1, robot_cell_x + roi_cells)
        min_y = max(1, robot_cell_y - roi_cells)
        max_y = min(h - 1, robot_cell_y + roi_cells)

        frontier_cells = []
        for y in range(min_y, max_y):
            for x in range(min_x, max_x):
                if not self._is_frontier_cell(data, w, h, x, y):
                    continue
                if not self._cell_is_safe(data, w, h, x, y):
                    continue
                frontier_cells.append((x, y))

        # Cluster frontiers
        clusters = self._cluster_frontiers(frontier_cells)
        
        # Calculate information gain for each cluster
        for cluster in clusters:
            cluster.information_gain = self._calculate_information_gain(
                data, w, h, cluster.center_x, cluster.center_y
            )

        return clusters

    def _pick_frontier_goal(self, robot_xy: Tuple[float, float]) -> Optional[Tuple[float, float]]:
        """Select best frontier goal using information gain"""
        m = self.map_msg
        if m is None:
            return None

        res = m.info.resolution
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y

        rx, ry = robot_xy

        # Find frontier clusters (use cache if recent)
        current_time = self.get_clock().now()
        use_cache = (self.last_frontier_scan_time is not None and
                    (current_time - self.last_frontier_scan_time).nanoseconds / 1e9 < 1.0)
        
        if not use_cache or not self.cached_frontier_clusters:
            self.cached_frontier_clusters = self._find_frontier_clusters(robot_xy)
            self.last_frontier_scan_time = current_time

        clusters = self.cached_frontier_clusters

        if not clusters:
            return None

        best_cluster = None
        best_score = -1e9

        for cluster in clusters:
            # Convert cluster center to world coordinates
            gx = ox + (cluster.center_x + 0.5) * res
            gy = oy + (cluster.center_y + 0.5) * res

            # Distance to robot
            dist = math.hypot(gx - rx, gy - ry)
            if dist < self.min_goal_distance:
                continue

            # Check if this goal was recently failed
            if (gx, gy) in self.failed_goals:
                continue

            # Check if too close to last goal (prevent going to same area)
            if self.last_goal_xy is not None:
                if math.hypot(gx - self.last_goal_xy[0], gy - self.last_goal_xy[1]) < 1.0:
                    continue

            # Calculate score
            if self.use_information_gain:
                # Information gain / distance ratio (higher is better)
                # Normalize: info_gain / (dist + 1.0) to avoid division by zero
                score = (cluster.information_gain * self.information_gain_weight) / (dist + 1.0)
                # Bonus for larger clusters
                score += cluster.size * 0.1
            else:
                # Simple: closest frontier
                score = -dist

            if score > best_score:
                best_score = score
                best_cluster = cluster

        if best_cluster is None:
            return None

        # Convert to world coordinates
        gx = ox + (best_cluster.center_x + 0.5) * res
        gy = oy + (best_cluster.center_y + 0.5) * res

        return (gx, gy)

    def _check_stuck(self, robot_xy: Tuple[float, float]) -> bool:
        """Check if robot is stuck (not moving)"""
        if not self.stuck_detection_enabled:
            return False
        
        if len(self.robot_positions) < 10:
            return False
        
        # Check if robot has moved significantly
        recent_positions = list(self.robot_positions)[-10:]
        min_x = min(p[0] for p in recent_positions)
        max_x = max(p[0] for p in recent_positions)
        min_y = min(p[1] for p in recent_positions)
        max_y = max(p[1] for p in recent_positions)
        
        movement_range = math.hypot(max_x - min_x, max_y - min_y)
        
        if movement_range < self.stuck_distance_threshold:
            time_stuck = (self.get_clock().now() - self.last_significant_movement).nanoseconds / 1e9
            if time_stuck > self.stuck_time_threshold:
                self.get_logger().warn(f"Robot appears stuck! Movement range: {movement_range:.2f}m, Time stuck: {time_stuck:.1f}s")
                return True
        else:
            self.last_significant_movement = self.get_clock().now()
        
        return False

    def _make_goal(self, gx: float, gy: float, robot_xy: Tuple[float, float]) -> PoseStamped:
        rx, ry = robot_xy
        yaw = math.atan2(gy - ry, gx - rx)

        pose = PoseStamped()
        pose.header.frame_id = self.goal_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = gx
        pose.pose.position.y = gy
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _send_goal(self, pose: PoseStamped):
        if not self.nav_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn("Nav2 action server not ready yet.")
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.get_logger().info(f"Sending frontier goal: x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}")
        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)

        self.last_goal_time = self.get_clock().now()
        self.active_goal_started = self.get_clock().now()
        self.last_goal_xy = (pose.pose.position.x, pose.pose.position.y)

    def _on_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"send_goal failed: {e}")
            self.active_goal_handle = None
            return

        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected.")
            # Mark as failed (except for start position goals)
            if self.last_goal_xy and not self.returning_to_start:
                self.failed_goals.add(self.last_goal_xy)
            self.active_goal_handle = None
            return

        self.active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        try:
            res = future.result()
            status = res.status
        except Exception as e:
            self.get_logger().error(f"get_result failed: {e}")
            status = None

        self.get_logger().info(f"Goal finished with status: {status}")
        
        # Status 4 = SUCCEEDED
        if status == 4:  # SUCCESS
            if self.returning_to_start:
                self.get_logger().info("Successfully returned to start position. Mission complete!")
                self.returning_to_start = False
                self.exploration_complete = True
                # Goal başarılı oldu, artık yeni goal göndermeyeceğiz
                self._mission_complete_logged = False  # Reset for logging
        else:
            # Failed - Don't mark start position goals as failed (we want to retry them)
            if self.last_goal_xy and not self.returning_to_start:
                self.failed_goals.add(self.last_goal_xy)
                # Clean old failed goals (keep last 20)
                if len(self.failed_goals) > 20:
                    self.failed_goals.pop()
        
        self.active_goal_handle = None
        self.active_goal_started = None

    def _cancel_active_goal(self):
        if self.active_goal_handle is None:
            return
        self.get_logger().warn("Cancelling active goal (timeout/stuck).")
        cancel_future = self.active_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(lambda f: None)
        self.active_goal_handle = None
        self.active_goal_started = None
        # Mark as failed (except for start position goals)
        if self.last_goal_xy and not self.returning_to_start:
            self.failed_goals.add(self.last_goal_xy)

    def _publish_visualization(self, robot_xy: Tuple[float, float]):
        """Publish frontier markers for visualization"""
        if not self.publish_visualization or not self.map_msg:
            return

        m = self.map_msg
        res = m.info.resolution
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y

        marker_array = MarkerArray()
        
        # Clear old markers - DELETEALL marker must have ID 0
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        clear_marker.header.frame_id = self.goal_frame
        clear_marker.header.stamp = self.get_clock().now().to_msg()
        clear_marker.id = 0
        clear_marker.ns = "frontiers"
        marker_array.markers.append(clear_marker)

        clusters = self.cached_frontier_clusters
        for i, cluster in enumerate(clusters[:50]):  # Limit to 50 clusters
            gx = ox + (cluster.center_x + 0.5) * res
            gy = oy + (cluster.center_y + 0.5) * res
            
            marker = Marker()
            marker.header.frame_id = self.goal_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "frontiers"  # Namespace for grouping
            marker.id = i + 1  # Start from 1 (0 is DELETEALL)
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = gx
            marker.pose.position.y = gy
            marker.pose.position.z = 0.1
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            
            # Color based on information gain (red = high, blue = low)
            max_gain = max(c.information_gain for c in clusters) if clusters else 1.0
            normalized_gain = cluster.information_gain / max_gain if max_gain > 0 else 0.0
            marker.color.r = float(normalized_gain)
            marker.color.g = 0.0
            marker.color.b = float(1.0 - normalized_gain)
            marker.color.a = 0.8
            marker.lifetime = Duration(seconds=2.0).to_msg()
            
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

    def _calculate_coverage(self) -> float:
        """Calculate explored area percentage"""
        if not self.map_msg:
            return 0.0
        
        data = self.map_msg.data
        total_cells = len([v for v in data if v != -1])  # Known cells
        free_cells = len([v for v in data if 0 <= v <= self.free_value_max])
        
        if total_cells == 0:
            return 0.0
        
        return (free_cells / total_cells) * 100.0

    def _publish_metrics(self):
        """Publish exploration metrics"""
        if not self.publish_metrics:
            return
        
        # Coverage
        coverage = self._calculate_coverage()
        msg = Float32()
        msg.data = coverage
        self.metrics_pubs['coverage'].publish(msg)
        
        # Frontier count
        msg = Float32()
        msg.data = float(len(self.cached_frontier_clusters))
        self.metrics_pubs['frontier_count'].publish(msg)
        
        # Total distance
        msg = Float32()
        msg.data = self.total_distance
        self.metrics_pubs['total_distance'].publish(msg)
        
        # Exploration time
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        msg = Float32()
        msg.data = elapsed
        self.metrics_pubs['exploration_time'].publish(msg)

    def _save_start_position(self, robot_xy: Tuple[float, float]):
        """Save start position"""
        if not self.start_position_saved and robot_xy is not None:
            self.start_position = robot_xy
            self.start_position_saved = True
            self.last_robot_xy = robot_xy
            self.get_logger().info(f"Start position saved: x={robot_xy[0]:.2f}, y={robot_xy[1]:.2f}")

    def _tick(self):
        # Map yoksa bekle
        if self.map_msg is None:
            return

        # TF yoksa bekle
        robot_xy = self._get_robot_xy_in_map()
        if robot_xy is None:
            return

        # Update distance tracking
        if self.last_robot_xy is not None:
            dist = math.hypot(robot_xy[0] - self.last_robot_xy[0], 
                            robot_xy[1] - self.last_robot_xy[1])
            self.total_distance += dist
        self.last_robot_xy = robot_xy

        # Track robot positions for stuck detection
        self.robot_positions.append(robot_xy)

        # Başlangıç pozisyonunu kaydet
        if not self.start_position_saved:
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            if elapsed > self.start_position_timeout:
                self._save_start_position(robot_xy)
            return

        # Stuck detection
        if self._check_stuck(robot_xy) and self.active_goal_handle is not None:
            self.get_logger().warn("Robot stuck detected! Cancelling current goal.")
            self._cancel_active_goal()
            # Clear failed goals to try different paths
            self.failed_goals.clear()
            self.robot_positions.clear()
            return

        # Aktif goal timeout kontrol (başlangıca dönüş için daha uzun timeout)
        # ÖNEMLİ: Sadece frontier exploration için return et, return-to-start için etme!
        if self.active_goal_started is not None and not self.returning_to_start:
            timeout = self.goal_timeout
            elapsed = (self.get_clock().now() - self.active_goal_started).nanoseconds / 1e9
            if elapsed > timeout:
                self._cancel_active_goal()
            return

        # Cooldown
        since_last = (self.get_clock().now() - self.last_goal_time).nanoseconds / 1e9
        if since_last < self.goal_cooldown:
            return

        # Başlangıca dönme kontrolü (exploration_complete kontrolünden ÖNCE yapılmalı!)
        if self.returning_to_start:
            if self.start_position is not None:
                dist_to_start = math.hypot(robot_xy[0] - self.start_position[0], 
                                          robot_xy[1] - self.start_position[1])
                if dist_to_start < 0.7:  # Biraz daha toleranslı threshold
                    self.get_logger().info("Returned to start position. Exploration complete!")
                    self.returning_to_start = False
                    self.exploration_complete = True
                    return
                # Goal yoksa veya başarısız olduysa tekrar gönder
                if self.active_goal_handle is None:
                    self.get_logger().info(f"Retrying start goal. Distance: {dist_to_start:.2f}m")
                    pose = self._make_goal(self.start_position[0], self.start_position[1], robot_xy)
                    self._send_goal(pose)
            return

        # Exploration complete ise frontier arama - sadece başlangıca dönüş yapılabilir
        if self.exploration_complete:
            # Exploration complete ama returning_to_start False ise, başlangıca dönmeyi başlat
            if self.return_to_start and self.start_position is not None and not self.returning_to_start:
                self.get_logger().info(f"Exploration complete. Starting return to start: x={self.start_position[0]:.2f}, y={self.start_position[1]:.2f}")
                self.returning_to_start = True
                pose = self._make_goal(self.start_position[0], self.start_position[1], robot_xy)
                self._send_goal(pose)
                return
            
            # Mission tamamlandı (exploration + return to start) - aktif goal'ı iptal et
            if not self.returning_to_start:
                if self.active_goal_handle is not None:
                    self.get_logger().info("Mission complete. Cancelling any active goal.")
                    self._cancel_active_goal()
                # Sadece bir kez log yaz
                if not hasattr(self, '_mission_complete_logged'):
                    self.get_logger().info("MISSION COMPLETE ✓")
                    self._mission_complete_logged = True
            return

        # Frontier ara
        goal_xy = self._pick_frontier_goal(robot_xy)
        
        # Publish visualization
        if self.publish_visualization:
            self._publish_visualization(robot_xy)
        
        if goal_xy is None:
            # Frontier bulunamadı
            if not self.exploration_complete:
                coverage = self._calculate_coverage()
                self.get_logger().info(f"No frontier found. Exploration complete! Coverage: {coverage:.1f}%")
                self.exploration_complete = True
                
                if self.return_to_start and self.start_position is not None:
                    self.get_logger().info(f"Returning to start position: x={self.start_position[0]:.2f}, y={self.start_position[1]:.2f}")
                    self.returning_to_start = True
                    pose = self._make_goal(self.start_position[0], self.start_position[1], robot_xy)
                    self._send_goal(pose)
            return

        pose = self._make_goal(goal_xy[0], goal_xy[1], robot_xy)
        self._send_goal(pose)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
