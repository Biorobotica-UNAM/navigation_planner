#!/usr/bin/env python3
"""Full navigation stack on the actual laboratory/prohibition maps; no hardware."""
import array
import math
import os
from pathlib import Path as FilePath
import subprocess
import time
import unittest

import numpy as np
from PIL import Image
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from ament_index_python.packages import get_package_prefix
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid
from path_planner.srv import PlanPath
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, Float64MultiArray
from tf2_ros import TransformBroadcaster

os.environ['CYCLONEDDS_URI'] = (
    '<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast></General>'
    '<Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer Address="127.0.0.1"/>'
    '</Peers></Discovery></Domain></CycloneDDS>')


class LaboratoryNavigation(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not os.environ.get('ROS_DOMAIN_ID'):
            raise RuntimeError('Use an isolated ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1')
        rclpy.init()
        cls.node = Node('laboratory_navigation_fixture')
        cls.logs, cls.processes = [], []
        cls.addClassCleanup(cls.cleanup)
        maps = FilePath(__file__).resolve().parents[1]/'maps'
        cls.grids = []
        for relative in ('maps/lab_2026/map.yaml', 'prohibition_maps/lab_2026/map.yaml'):
            metadata = maps/relative
            config = yaml.safe_load(metadata.read_text())
            pixels = np.flipud(np.asarray(Image.open(metadata.parent/config['image'])))
            probability = 1-pixels.astype(float)/255
            values = np.full(pixels.shape, -1, dtype=np.int8)
            values[probability < config['free_thresh']] = 0
            values[probability > config['occupied_thresh']] = 100
            grid = OccupancyGrid()
            grid.header.frame_id = 'map'
            grid.info.resolution = config['resolution']
            grid.info.height, grid.info.width = values.shape
            grid.info.origin.position.x, grid.info.origin.position.y, yaw = config['origin']
            grid.info.origin.orientation.z, grid.info.origin.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
            grid.data = array.array('b', values.ravel().tobytes())
            cls.grids.append(grid)
            if len(cls.grids) == 1:
                cls.physical_map = values
        cls.x, cls.y, cls.yaw = 0.041, 0.940, 1.635
        cls.head_pan = cls.head_target = 0.0
        cls.command = Twist()
        cls.statuses, cls.trace = [], []
        cls.minimum_clearance = float('inf')
        cls.dynamic_obstacle = None
        cls.integrate = False
        cls.last_maps = cls.last_scan = 0.0
        cls.tf = TransformBroadcaster(cls.node)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        cls.raw_pub = cls.node.create_publisher(OccupancyGrid, '/lab_test/raw_map', qos)
        cls.prohibition_pub = cls.node.create_publisher(OccupancyGrid, '/lab_test/prohibition', qos)
        cls.goal_pub = cls.node.create_publisher(PoseStamped, '/lab_test/goal', 10)
        cls.stop_pub = cls.node.create_publisher(Empty, '/lab_test/stop', 10)
        cls.head_pub = cls.node.create_publisher(Float64MultiArray, '/hardware/head/current_pose', 10)
        cls.scan_pub = cls.node.create_publisher(LaserScan, '/lab_scan', 10)
        cls.subs = [
            cls.node.create_subscription(Twist, '/lab_cmd_vel', lambda m: setattr(cls, 'command', m), 10),
            cls.node.create_subscription(GoalStatus, '/lab_test/status', lambda m: cls.statuses.append(m), 10),
            cls.node.create_subscription(Float64MultiArray, '/hardware/head/goal_pose',
                                         lambda m: setattr(cls, 'head_target', m.data[0]), 10),
        ]
        configurations = [
            ('map_augmenter', 'map_augmenter_node', ['-p', 'base_map_topic:=raw_map',
                '-p', 'prohibition_map_topic:=prohibition', '-p', 'use_prohibition_map:=true',
                '-p', 'static_obstacle_inflation_radius:=0.25', '-p', 'dynamic_obstacle_inflation_radius:=0.25',
                '-p', 'circular_inflation:=true', '-p', 'cost_radius:=0.9', '-p', 'cost_peak:=15']),
            ('map_augmenter', 'obstacle_detector_node', ['-p', 'use_lidar:=true',
                '-p', 'base_link_name:=base_link', '-p', 'laser_scan_topic:=/lab_scan',
                '-p', 'laser_min_x:=0.1', '-p', 'laser_max_x:=3.0', '-p', 'laser_min_y:=-3.0',
                '-p', 'laser_max_y:=3.0', '-p', 'laser_min_z:=-0.1', '-p', 'laser_max_z:=0.5',
                '-p', 'static_obstacle_ignore_radius:=0.08']),
            ('path_planner', 'path_planner_node', ['-p', 'diagonal_paths:=true']),
            ('simple_move', 'simple_move_node', ['-p', 'max_linear_speed:=0.9',
                '-p', 'control_alpha:=0.2', '-p', 'control_beta:=0.6', '-p', 'move_head:=true',
                '-p', 'use_pot_fields:=true', '-r', '/cmd_vel:=/lab_cmd_vel']),
            ('potential_fields', 'potential_fields_node', ['-p', 'use_lidar:=true',
                '-p', 'use_point_cloud:=false', '-p', 'laser_scan_topic:=/lab_scan',
                '-p', 'laser_min_z:=-0.1', '-p', 'laser_pot_fields_d0:=0.75',
                '-p', 'laser_pot_fields_k_rej:=0.4', '-p', 'lidar_points_threshold:=2',
                '-r', '/cmd_vel:=/lab_cmd_vel']),
            ('mvn_pln', 'mvn_pln_node', ['-r', '/nav_control/goal:=/lab_test/goal']),
        ]
        for package, executable, extra in configurations:
            log = open(f'/tmp/laboratory_{executable}.log', 'w')
            cls.logs.append(log)
            binary = FilePath(get_package_prefix(package))/'lib'/package/executable
            cls.processes.append(subprocess.Popen([str(binary), '--ros-args', '-r', '__ns:=/lab_test',
                '-p', 'use_namespace:=true']+extra, stdout=log, stderr=subprocess.STDOUT))
        cls.client = cls.node.create_client(PlanPath, '/lab_test/path_planner/plan_navigation')
        cls.timer = cls.node.create_timer(0.02, cls.tick)
        cls.spin_for(2)
        if any(p.poll() is not None for p in cls.processes):
            raise RuntimeError('A node exited; see /tmp/laboratory_*.log')

    @classmethod
    def cleanup(cls):
        for p in cls.processes:
            p.terminate()
        for p in cls.processes:
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()
                p.wait(timeout=5)
        for log in cls.logs:
            log.close()
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def tick(cls):
        if cls.integrate:
            cls.x += (math.cos(cls.yaw)*cls.command.linear.x-math.sin(cls.yaw)*cls.command.linear.y)*0.02
            cls.y += (math.sin(cls.yaw)*cls.command.linear.x+math.cos(cls.yaw)*cls.command.linear.y)*0.02
            cls.yaw += cls.command.angular.z*0.02
            cls.trace.append((cls.x, cls.y, cls.yaw, math.hypot(cls.command.linear.x, cls.command.linear.y), cls.command.angular.z))
            if cls.dynamic_obstacle is not None:
                cls.minimum_clearance = min(cls.minimum_clearance,
                    math.hypot(cls.x-cls.dynamic_obstacle[0], cls.y-cls.dynamic_obstacle[1])-0.08)
        cls.head_pan += max(-0.03, min(0.03, cls.head_target-cls.head_pan))
        cls.head_pub.publish(Float64MultiArray(data=[cls.head_pan, -0.5]))
        stamp = cls.node.get_clock().now().to_msg()
        transforms = []
        for parent, child in [('map', 'odom'), ('odom', 'base_link')]:
            tf = TransformStamped()
            tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, parent, child
            tf.transform.rotation.w = 1.0
            if child == 'base_link':
                tf.transform.translation.x, tf.transform.translation.y = cls.x, cls.y
                tf.transform.rotation.z, tf.transform.rotation.w = math.sin(cls.yaw/2), math.cos(cls.yaw/2)
            transforms.append(tf)
        cls.tf.sendTransform(transforms)
        if time.monotonic()-cls.last_maps >= 0.5:
            cls.last_maps = time.monotonic()
            for publisher, grid in zip((cls.raw_pub, cls.prohibition_pub), cls.grids):
                grid.header.stamp = stamp
                publisher.publish(grid)
        info = cls.grids[0].info
        gx, gy = (cls.x-info.origin.position.x)/info.resolution, (cls.y-info.origin.position.y)/info.resolution
        # Exact point-to-occupied-cell-square distance around the physical base.
        ix, iy = int(gx), int(gy)
        for cy, cx in np.argwhere(cls.physical_map[max(0, iy-7):iy+8, max(0, ix-7):ix+8] > 0):
            cx += max(0, ix-7); cy += max(0, iy-7)
            distance = math.hypot(max(cx-gx, 0, gx-cx-1), max(cy-gy, 0, gy-cy-1))*info.resolution
            cls.minimum_clearance = min(cls.minimum_clearance, distance)
        if time.monotonic()-cls.last_scan >= 0.05:
            cls.last_scan = time.monotonic()
            angles = np.linspace(-math.pi, math.pi, 361)+cls.yaw
            distances = np.arange(0.10, 3.01, 0.025)
            wx = cls.x+np.cos(angles)[:, None]*distances
            wy = cls.y+np.sin(angles)[:, None]*distances
            mx = ((wx-info.origin.position.x)/info.resolution).astype(int)
            my = ((wy-info.origin.position.y)/info.resolution).astype(int)
            valid = (mx >= 0)&(mx < info.width)&(my >= 0)&(my < info.height)
            hits = valid & (cls.physical_map[np.clip(my, 0, info.height-1), np.clip(mx, 0, info.width-1)] > 0)
            if cls.dynamic_obstacle is not None:
                ox, oy = cls.dynamic_obstacle
                hits |= (wx-ox)**2+(wy-oy)**2 < 0.08**2
            ranges = np.where(hits.any(axis=1), distances[hits.argmax(axis=1)], float('inf'))
            scan = LaserScan()
            scan.header.stamp, scan.header.frame_id = stamp, 'base_link'
            scan.angle_min, scan.angle_max, scan.angle_increment = -math.pi, math.pi, math.pi/180
            scan.range_min, scan.range_max, scan.ranges = 0.1, 3.0, ranges.astype(float).tolist()
            cls.scan_pub.publish(scan)

    @classmethod
    def spin_for(cls, seconds):
        until = time.monotonic()+seconds
        while time.monotonic() < until:
            rclpy.spin_once(cls.node, timeout_sec=0.01)

    def setUp(self):
        self.stop_pub.publish(Empty())
        type(self).integrate = False
        self.spin_for(0.2)
        type(self).dynamic_obstacle = None
        self.statuses.clear()
        self.trace.clear()
        type(self).minimum_clearance = float('inf')

    def navigate(self, start, goal, timeout=45, introduce_obstacle=False):
        cls = type(self)
        cls.x, cls.y, cls.yaw = start
        cls.command = Twist()
        cls.head_pan = cls.head_target = 0.0
        self.spin_for(0.6)
        message = PoseStamped()
        message.header.frame_id = 'map'
        message.pose.position.x, message.pose.position.y, angle = goal
        message.pose.orientation.z, message.pose.orientation.w = math.sin(angle/2), math.cos(angle/2)
        cls.integrate = True
        self.goal_pub.publish(message)
        until = time.monotonic()+timeout
        inserted = False
        while time.monotonic() < until:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            if introduce_obstacle and not inserted and math.hypot(self.x-start[0], self.y-start[1]) > 0.3:
                cls.dynamic_obstacle = (cls.x+0.7*math.cos(cls.yaw), cls.y+0.7*math.sin(cls.yaw))
                inserted = True
            if any(s.status in (3, 4) for s in self.statuses):
                break
        cls.integrate = False
        self.assertTrue(any(s.status == 3 for s in self.statuses),
                        [(s.status, s.text) for s in self.statuses]+[('pose', (self.x,self.y,self.yaw))])
        self.assertLess(math.hypot(self.x-goal[0], self.y-goal[1]), 0.04)
        self.assertLess(abs(math.remainder(self.yaw-goal[2], 2*math.pi)), 0.06)
        self.assertGreaterEqual(self.minimum_clearance, 0.225)
        print(f' peak={max(t[3] for t in self.trace):.3f}m/s clearance={self.minimum_clearance:.3f}m', end='')

    def test_logged_corner_route_at_launch_speed(self):
        self.navigate((0.041, 0.940, 1.635), (-1.253, -3.962, 1.573))

    def test_logged_terminal_pose_next_to_wall(self):
        self.navigate((-1.253, -3.962, 1.573), (-0.279, -4.110, 1.503), timeout=25)
        changes = [a[4]*b[4] < -0.01 for a,b in zip(self.trace, self.trace[1:])]
        self.assertLess(sum(changes), 4)

    def test_straight_corridor_preserves_speed(self):
        self.navigate((-2.245, -1.008, -math.pi/2), (-2.009, -2.660, -math.pi/2), timeout=25)
        self.assertGreater(max(t[3] for t in self.trace), 0.45)

    def test_small_start_changes_do_not_require_nudging_robot(self):
        for dx, dy in ((0, 0), (-0.02, 0), (0.02, 0), (0, -0.02), (0, 0.02)):
            request = PlanPath.Request()
            request.start.header.frame_id = request.goal.header.frame_id = 'map'
            request.start.pose.position.x, request.start.pose.position.y = 0.041+dx, 0.940+dy
            request.start.pose.orientation.w = 1.0
            request.goal.pose.position.x, request.goal.pose.position.y = -0.279, -4.110
            request.goal.pose.orientation.w = 1.0
            future = self.client.call_async(request)
            until = time.monotonic()+4
            while not future.done() and time.monotonic() < until:
                rclpy.spin_once(self.node, timeout_sec=0.01)
            self.assertTrue(future.done())
            self.assertEqual(future.result().reason, '', (dx,dy,future.result().reason))
            self.assertTrue(future.result().path.poses)

    def test_obstacle_appearing_during_motion_is_avoided(self):
        self.navigate((-2.245, -1.008, -math.pi/2), (-2.009, -2.660, -math.pi/2),
                      timeout=35, introduce_obstacle=True)


if __name__ == '__main__':
    unittest.main(verbosity=2)
