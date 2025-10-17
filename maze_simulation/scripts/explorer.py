#!/usr/bin/env python3
import rclpy
import time
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Quaternion
from tf_transformations import quaternion_from_euler
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

# Tunables
MIN_FRONTIER_SIZE = 5
GOAL_REACHED_DISTANCE = 0.35
MAP_UNKNOWN = -1
MAP_FREE = 0
MAP_OCC = 100

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        # Subscribe to /map with TRANSIENT_LOCAL so we receive latched maps
        map_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST,
                             depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_msg = None
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self._map_cb, qos_profile=map_qos)
        # Action client for Nav2
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('FrontierExplorer initialized (waiting for map)...')

    def _map_cb(self, msg: OccupancyGrid):
        # store latest map
        self.map_msg = msg

    def wait_for_map(self, timeout=None):
        """Wait until we have a valid map (non-empty data and non-zero stamp)."""
        t0 = time.time()
        self.get_logger().info('Waiting for /map (transient_local) ...')
        while rclpy.ok():
            if self.map_msg is not None and getattr(self.map_msg, 'data', None):
                ts = self.map_msg.header.stamp
                if ts.sec != 0 or ts.nanosec != 0:
                    self.get_logger().info(f'Map received: {len(self.map_msg.data)} cells')
                    return True
            if timeout and (time.time() - t0) > timeout:
                self.get_logger().warn('Timeout waiting for /map')
                return False
            time.sleep(0.2)
        return False

    def grid_to_world(self, i, j):
        origin = self.map_msg.info.origin
        res = self.map_msg.info.resolution
        x = origin.position.x + (j + 0.5) * res
        y = origin.position.y + (i + 0.5) * res
        return x, y

    def detect_frontiers(self):
        data = self.map_msg.data
        if not data:
            return []
        import numpy as np
        arr = np.array(data, dtype=np.int8).reshape((self.map_msg.info.height, self.map_msg.info.width))
        h, w = arr.shape
        frontiers = []
        visited = np.zeros_like(arr, dtype=bool)
        for i in range(1, h-1):
            for j in range(1, w-1):
                if visited[i, j]:
                    continue
                if arr[i, j] == MAP_FREE:
                    neigh = arr[i-1:i+2, j-1:j+2]
                    if (neigh == MAP_UNKNOWN).any():
                        cluster = []
                        stack = [(i, j)]
                        visited[i, j] = True
                        while stack:
                            ci, cj = stack.pop()
                            cluster.append((ci, cj))
                            for di in (-1, 0, 1):
                                for dj in (-1, 0, 1):
                                    ni, nj = ci + di, cj + dj
                                    if ni <= 0 or ni >= h-1 or nj <= 0 or nj >= w-1:
                                        continue
                                    if visited[ni, nj]:
                                        continue
                                    if arr[ni, nj] == MAP_FREE:
                                        neigh2 = arr[ni-1:ni+2, nj-1:nj+2]
                                        if (neigh2 == MAP_UNKNOWN).any():
                                            visited[ni, nj] = True
                                            stack.append((ni, nj))
                        if len(cluster) >= MIN_FRONTIER_SIZE:
                            frontiers.append(cluster)
        return frontiers

    def frontier_centroid(self, cluster):
        xs, ys = [], []
        for i, j in cluster:
            x, y = self.grid_to_world(i, j)
            xs.append(x); ys.append(y)
        return (sum(xs)/len(xs), sum(ys)/len(ys))

    def send_goal(self, x, y, frame='map'):
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = frame
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        q = quaternion_from_euler(0, 0, 0)
        ps.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        if not self.nav_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('NavigateToPose action server not available')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = ps
        send_goal_future = self.nav_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            return False

        self.get_logger().info(f'Goal accepted: {x:.2f}, {y:.2f}')
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result().result
        status = get_result_future.result().status
        self.get_logger().info('Navigate result received')
        # status codes: 4 = CANCELED, 3 = SUCCEEDED typically
        return status == 3

    def explore_loop(self):
        if not self.wait_for_map(timeout=120.0):
            self.get_logger().error('No map received; aborting exploration.')
            return

        while rclpy.ok():
            frontiers = self.detect_frontiers()
            self.get_logger().info(f'Found {len(frontiers)} frontiers')
            if not frontiers:
                self.get_logger().info('Exploration complete: no frontiers left')
                return

            # choose largest frontier
            frontiers.sort(key=lambda c: -len(c))
            chosen = frontiers[0]
            cx, cy = self.frontier_centroid(chosen)
            self.get_logger().info(f'Navigating to frontier centroid: ({cx:.2f}, {cy:.2f})')

            ok = self.send_goal(cx, cy, frame=self.map_msg.header.frame_id)
            if not ok:
                self.get_logger().warn('Goal failed — removing frontier and continuing')
                # naive: just continue
                continue
            time.sleep(1.0)

def main():
    # Initialize rclpy context explicitly here
    rclpy.init()
    node = FrontierExplorer()
    try:
        node.explore_loop()
    except KeyboardInterrupt:
        node.get_logger().info('Explorer interrupted by user')
    finally:
        node.destroy_node()
        # Shutdown only if init() succeeded
        try:
            rclpy.shutdown()
        except Exception as e:
            node.get_logger().warn(f'Error shutting down rclpy: {e}')

if __name__ == '__main__':
    main()
